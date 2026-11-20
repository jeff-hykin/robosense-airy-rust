//! Live sensor: joins the Airy's multicast group (or listens unicast), decodes on a receive thread, and hands complete
//! scans and IMU samples to the consumer through bounded queues that drop the newest item when full.

use crate::decoder::{Decoder, DecoderConfig, ImuSample, Scan};
use crate::filter::{self, FilterConfig};
use crate::packet::Difop;
use socket2::{Domain, Protocol, SockAddr, Socket, Type};
use std::io;
use std::mem::MaybeUninit;
use std::net::{Ipv4Addr, Shutdown, SocketAddrV4};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

#[derive(Debug, Clone, PartialEq)]
pub struct SensorConfig {
    /// Multicast group the lidar sends to; None listens for unicast on `interface`.
    pub group: Option<Ipv4Addr>,
    /// Local address of the NIC to join the group on (or to bind, for unicast). 0.0.0.0 lets the OS pick.
    pub interface: Ipv4Addr,
    pub msop_port: u16,
    pub difop_port: u16,
    /// None skips the IMU.
    pub imu_port: Option<u16>,
    /// Carried on every scan and IMU sample, for the consumer's messages.
    pub frame_id: String,
    pub decoder: DecoderConfig,
    pub filter: FilterConfig,
    /// Complete scans waiting for the consumer; a scan that finds it full is dropped.
    pub scan_queue: usize,
    /// IMU samples waiting for the consumer; a sample that finds it full is dropped.
    pub imu_queue: usize,
    /// Rotate IMU samples into the lidar frame with the DIFOP's IMU extrinsic; samples before the first DIFOP are
    /// dropped. false yields them in the IMU chip's own axes.
    pub imu_in_lidar_frame: bool,
    /// SO_RCVBUF for the MSOP socket in bytes; None keeps the OS default.
    pub recv_buffer_bytes: Option<usize>,
}

impl Default for SensorConfig {
    /// rs_driver's input defaults (unicast, MSOP 6699, DIFOP 7788, no IMU) with one-deep queues.
    fn default() -> Self {
        SensorConfig {
            group: None,
            interface: Ipv4Addr::UNSPECIFIED,
            msop_port: 6699,
            difop_port: 7788,
            imu_port: None,
            frame_id: "rslidar".into(),
            decoder: DecoderConfig::default(),
            filter: FilterConfig::default(),
            scan_queue: 1,
            imu_queue: 1,
            imu_in_lidar_frame: false,
            recv_buffer_bytes: None,
        }
    }
}

/// Packet and drop counters since `open`.
#[derive(Debug, Default)]
pub struct Stats {
    pub msop_packets: AtomicU64,
    pub difop_packets: AtomicU64,
    pub imu_packets: AtomicU64,
    /// Datagrams that failed to decode (wrong length/id, no DIFOP yet, ...).
    pub decode_errors: AtomicU64,
    pub scans: AtomicU64,
    /// Scans dropped because the consumer's queue was full.
    pub scans_dropped: AtomicU64,
    /// Scans skipped by `decimate_scans` or as the partial first scan.
    pub scans_skipped: AtomicU64,
    pub imu_samples: AtomicU64,
    pub imu_dropped: AtomicU64,
    /// IMU samples dropped while waiting for the DIFOP extrinsic (`imu_in_lidar_frame`).
    pub imu_waiting: AtomicU64,
}

/// A scan plus the frame it is in.
#[derive(Debug, Clone)]
pub struct FramedScan {
    pub frame_id: Arc<str>,
    pub scan: Scan,
}

pub struct Sensor {
    scans: Receiver<FramedScan>,
    imu: Receiver<ImuSample>,
    stats: Arc<Stats>,
    stop: Arc<AtomicBool>,
    sockets: Vec<Socket>,
    threads: Vec<JoinHandle<()>>,
    frame_id: Arc<str>,
}

fn open_socket(config: &SensorConfig, port: u16, recv_buffer: Option<usize>) -> io::Result<Socket> {
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    // share the port with the vendor driver, which binds it too
    socket.set_reuse_address(true)?;
    #[cfg(all(unix, not(any(target_os = "solaris", target_os = "illumos"))))]
    socket.set_reuse_port(true)?;
    if let Some(bytes) = recv_buffer {
        socket.set_recv_buffer_size(bytes)?;
    }
    let bind = match config.group {
        // binding the group address keeps other groups' datagrams on this port out (Linux); macOS needs ANY
        Some(group) if cfg!(target_os = "linux") => group,
        Some(_) => Ipv4Addr::UNSPECIFIED,
        None => config.interface,
    };
    socket.bind(&SockAddr::from(SocketAddrV4::new(bind, port)))?;
    if let Some(group) = config.group {
        socket.join_multicast_v4(&group, &config.interface)?;
    }
    Ok(socket)
}

fn recv(socket: &Socket, buf: &mut [MaybeUninit<u8>]) -> io::Result<usize> {
    socket.recv(buf)
}

fn bytes(buf: &[MaybeUninit<u8>], n: usize) -> &[u8] {
    // SAFETY: recv initialized the first n bytes
    unsafe { std::slice::from_raw_parts(buf.as_ptr() as *const u8, n) }
}

// large enough for any Airy datagram (1248 bytes) plus slack for an unexpected larger one to be seen as wrong-length
const RECV_BUF: usize = 2048;

impl Sensor {
    /// Opens the sockets and starts the receive threads.
    pub fn open(config: SensorConfig) -> io::Result<Sensor> {
        let msop = open_socket(&config, config.msop_port, config.recv_buffer_bytes)?;
        let difop = open_socket(&config, config.difop_port, None)?;
        let imu = config.imu_port.map(|port| open_socket(&config, port, None)).transpose()?;
        let stats = Arc::new(Stats::default());
        let stop = Arc::new(AtomicBool::new(false));
        let frame_id: Arc<str> = config.frame_id.as_str().into();
        let (scan_tx, scans) = mpsc::sync_channel(config.scan_queue);
        let (imu_tx, imu_rx) = mpsc::sync_channel(config.imu_queue);
        let (difop_tx, difop_rx) = mpsc::channel::<Vec<u8>>();
        let mut sockets = vec![msop.try_clone()?, difop.try_clone()?];
        let mut threads = Vec::new();
        let imu_extrinsic: Arc<Mutex<Option<[f32; 4]>>> = Arc::new(Mutex::new(None));

        {
            let (stats, stop, imu_extrinsic) = (stats.clone(), stop.clone(), imu_extrinsic.clone());
            threads.push(std::thread::Builder::new().name("airy-difop".into()).spawn(move || {
                let mut buf = [MaybeUninit::uninit(); RECV_BUF];
                while let Ok(n) = recv(&difop, &mut buf) {
                    if stop.load(Ordering::Relaxed) || n == 0 {
                        break;
                    }
                    stats.difop_packets.fetch_add(1, Ordering::Relaxed);
                    if let Ok(d) = Difop::parse(bytes(&buf, n)) {
                        *imu_extrinsic.lock().unwrap() = Some(d.imu_quat_xyzw);
                    }
                    if difop_tx.send(bytes(&buf, n).to_vec()).is_err() {
                        break;
                    }
                }
            })?);
        }

        if let Some(imu) = imu {
            sockets.push(imu.try_clone()?);
            let (stats, stop) = (stats.clone(), stop.clone());
            let decoder = Decoder::new(config.decoder.clone());
            let in_lidar_frame = config.imu_in_lidar_frame;
            threads.push(std::thread::Builder::new().name("airy-imu".into()).spawn(move || {
                let mut buf = [MaybeUninit::uninit(); RECV_BUF];
                while let Ok(n) = recv(&imu, &mut buf) {
                    if stop.load(Ordering::Relaxed) || n == 0 {
                        break;
                    }
                    stats.imu_packets.fetch_add(1, Ordering::Relaxed);
                    let sample = decoder.decode_imu(bytes(&buf, n)).map(|sample| {
                        if !in_lidar_frame {
                            return Some(sample);
                        }
                        let q = *imu_extrinsic.lock().unwrap();
                        q.map(|q| sample.rotated(q))
                    });
                    match sample {
                        Ok(None) => {
                            stats.imu_waiting.fetch_add(1, Ordering::Relaxed);
                        }
                        Ok(Some(sample)) => match imu_tx.try_send(sample) {
                            Ok(()) => {
                                stats.imu_samples.fetch_add(1, Ordering::Relaxed);
                            }
                            Err(TrySendError::Full(_)) => {
                                stats.imu_dropped.fetch_add(1, Ordering::Relaxed);
                            }
                            Err(TrySendError::Disconnected(_)) => break,
                        },
                        Err(_) => {
                            stats.decode_errors.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
            })?);
        }

        {
            let (stats, stop, frame_id) = (stats.clone(), stop.clone(), frame_id.clone());
            let filter_config = config.filter.clone();
            let mut decoder = Decoder::new(config.decoder.clone());
            threads.push(std::thread::Builder::new().name("airy-msop".into()).spawn(move || {
                let mut buf = [MaybeUninit::uninit(); RECV_BUF];
                let mut scan_index: usize = 0;
                while let Ok(n) = recv(&msop, &mut buf) {
                    if stop.load(Ordering::Relaxed) || n == 0 {
                        break;
                    }
                    while let Ok(d) = difop_rx.try_recv() {
                        if decoder.decode_difop(&d).is_err() {
                            stats.decode_errors.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    stats.msop_packets.fetch_add(1, Ordering::Relaxed);
                    let scan = match decoder.decode_msop(bytes(&buf, n)) {
                        Ok(Some(scan)) => scan,
                        Ok(None) => continue,
                        Err(_) => {
                            stats.decode_errors.fetch_add(1, Ordering::Relaxed);
                            continue;
                        }
                    };
                    stats.scans.fetch_add(1, Ordering::Relaxed);
                    let keep = scan.complete && scan_index.is_multiple_of(filter_config.decimate_scans.max(1));
                    if scan.complete {
                        scan_index += 1;
                    }
                    if !keep {
                        stats.scans_skipped.fetch_add(1, Ordering::Relaxed);
                        continue;
                    }
                    let mut scan = scan;
                    filter::apply(&mut scan.points, &filter_config);
                    match scan_tx.try_send(FramedScan { frame_id: frame_id.clone(), scan }) {
                        Ok(()) => {}
                        Err(TrySendError::Full(_)) => {
                            stats.scans_dropped.fetch_add(1, Ordering::Relaxed);
                        }
                        Err(TrySendError::Disconnected(_)) => break,
                    }
                }
            })?);
        }

        Ok(Sensor { scans, imu: imu_rx, stats, stop, sockets, threads, frame_id })
    }

    /// Complete scans, filtered per the config. Blocks in `recv`; ends when the sensor stops.
    pub fn scans(&self) -> &Receiver<FramedScan> {
        &self.scans
    }

    /// IMU samples (empty when `imu_port` is None).
    pub fn imu(&self) -> &Receiver<ImuSample> {
        &self.imu
    }

    /// Moves the IMU receiver out, for a consumer on another thread; `imu()` is empty afterwards.
    pub fn take_imu(&mut self) -> Receiver<ImuSample> {
        std::mem::replace(&mut self.imu, mpsc::sync_channel(0).1)
    }

    pub fn stats(&self) -> &Stats {
        &self.stats
    }

    /// The counters, shareable with other threads.
    pub fn stats_handle(&self) -> Arc<Stats> {
        self.stats.clone()
    }

    pub fn frame_id(&self) -> &str {
        &self.frame_id
    }

    /// A handle that stops the sensor from another thread (e.g. while a consumer is parked on `scans()`); the
    /// queues then disconnect and the consumer's loop ends.
    pub fn stop_handle(&self) -> io::Result<StopHandle> {
        Ok(StopHandle {
            stop: self.stop.clone(),
            sockets: self.sockets.iter().map(|s| s.try_clone()).collect::<io::Result<_>>()?,
        })
    }

    /// Stops the threads. On Linux, shutting a socket down wakes a thread parked in recv (even though shutdown on an
    /// unconnected UDP socket reports ENOTCONN), so the threads are joined; elsewhere they are left to exit on their
    /// next datagram.
    pub fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        for s in &self.sockets {
            let _ = s.shutdown(Shutdown::Read);
        }
        let threads = std::mem::take(&mut self.threads);
        if cfg!(target_os = "linux") {
            for t in threads {
                let _ = t.join();
            }
        }
    }
}

impl Drop for Sensor {
    fn drop(&mut self) {
        self.stop();
    }
}

pub struct StopHandle {
    stop: Arc<AtomicBool>,
    sockets: Vec<Socket>,
}

impl StopHandle {
    /// Sets the stop flag and wakes the receive threads (see `Sensor::stop`); does not join them.
    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
        for s in &self.sockets {
            let _ = s.shutdown(Shutdown::Read);
        }
    }
}

