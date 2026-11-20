//! Live sensor: joins the Airy's multicast group (or listens unicast), decodes on a receive thread, and hands complete
//! scans and IMU samples to the consumer through bounded queues that drop the newest item when full.

use crate::decoder::{DecodeError, Decoder, DecoderConfig, ImuSample, Scan};
use crate::filter::{self, FilterConfig};
use crate::packet::Difop;
use socket2::{Domain, Protocol, SockAddr, Socket, Type};
use std::io;
use std::mem::MaybeUninit;
use std::net::{Ipv4Addr, SocketAddrV4};
#[cfg(unix)]
use std::os::unix::{io::AsRawFd, net::UnixStream};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, TrySendError};
use std::sync::{Arc, Mutex, PoisonError};
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
    /// rs_driver's input defaults (unicast, MSOP 6699, DIFOP 7788, no IMU); a one-scan queue and an eight-sample IMU
    /// queue (200 Hz samples arrive in bursts; on the M20 a one-deep queue dropped ~1%, eight dropped none).
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
            imu_queue: 8,
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
    /// MSOP packets dropped before the first DIFOP gave the beam angles (`wait_for_difop`).
    pub msop_waiting: AtomicU64,
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
    stopper: Arc<Stopper>,
    threads: Vec<JoinHandle<()>>,
    frame_id: Arc<str>,
}

// The stop flag plus what wakes receive threads parked waiting for a datagram.
struct Stopper {
    stopped: AtomicBool,
    // a byte written to the second end leaves the first readable, waking every thread polling it at once
    #[cfg(unix)]
    wake: (UnixStream, UnixStream),
    // elsewhere a socket shutdown is the only wakeup there is
    #[cfg(not(unix))]
    sockets: Vec<Socket>,
}

impl Stopper {
    #[cfg(unix)]
    fn new(_sockets: &[&Socket]) -> io::Result<Stopper> {
        let wake = UnixStream::pair()?;
        // a full buffer (many stops) must not block a stop; the bytes already there keep the read end readable
        wake.1.set_nonblocking(true)?;
        Ok(Stopper { stopped: AtomicBool::new(false), wake })
    }

    #[cfg(not(unix))]
    fn new(sockets: &[&Socket]) -> io::Result<Stopper> {
        let sockets = sockets.iter().map(|s| s.try_clone()).collect::<io::Result<_>>()?;
        Ok(Stopper { stopped: AtomicBool::new(false), sockets })
    }

    fn stop(&self) {
        self.stopped.store(true, Ordering::Release);
        #[cfg(unix)]
        {
            use std::io::Write;
            let _ = (&self.wake.1).write_all(&[1]);
        }
        #[cfg(not(unix))]
        for s in &self.sockets {
            let _ = s.shutdown(std::net::Shutdown::Read);
        }
    }

    fn stopped(&self) -> bool {
        self.stopped.load(Ordering::Acquire)
    }
}

// Blocks until a datagram arrives (Some(its length)) or the sensor stops (None); EINTR and spurious wakeups retry.
fn recv(socket: &Socket, stopper: &Stopper, buf: &mut [MaybeUninit<u8>]) -> io::Result<Option<usize>> {
    loop {
        if stopper.stopped() {
            return Ok(None);
        }
        #[cfg(unix)]
        {
            let mut fds = [
                libc::pollfd { fd: socket.as_raw_fd(), events: libc::POLLIN, revents: 0 },
                libc::pollfd { fd: stopper.wake.0.as_raw_fd(), events: libc::POLLIN, revents: 0 },
            ];
            // SAFETY: fds is a live array of fds.len() pollfds; -1 waits without a timeout
            let ready = unsafe { libc::poll(fds.as_mut_ptr(), fds.len() as libc::nfds_t, -1) };
            if ready < 0 {
                let error = io::Error::last_os_error();
                if error.kind() == io::ErrorKind::Interrupted {
                    continue;
                }
                return Err(error);
            }
            if fds[1].revents != 0 {
                return Ok(None);
            }
            if fds[0].revents == 0 {
                continue;
            }
        }
        match socket.recv(buf) {
            Ok(n) => return Ok(Some(n)),
            Err(e) if matches!(e.kind(), io::ErrorKind::Interrupted | io::ErrorKind::WouldBlock) => continue,
            Err(e) => return Err(e),
        }
    }
}

fn lock<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
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
    // recv runs only after poll reports a datagram, so a spurious readiness must not park the thread
    #[cfg(unix)]
    socket.set_nonblocking(true)?;
    Ok(socket)
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
        let stopper = Arc::new(Stopper::new(&[Some(&msop), Some(&difop), imu.as_ref()].into_iter().flatten().collect::<Vec<_>>())?);
        let frame_id: Arc<str> = config.frame_id.as_str().into();
        let (scan_tx, scans) = mpsc::sync_channel(config.scan_queue);
        let (imu_tx, imu_rx) = mpsc::sync_channel(config.imu_queue);
        let (difop_tx, difop_rx) = mpsc::channel::<Vec<u8>>();
        let mut threads = Vec::new();
        let imu_extrinsic: Arc<Mutex<Option<[f32; 4]>>> = Arc::new(Mutex::new(None));

        {
            let (stats, stopper, imu_extrinsic) = (stats.clone(), stopper.clone(), imu_extrinsic.clone());
            threads.push(std::thread::Builder::new().name("airy-difop".into()).spawn(move || {
                let mut buf = [MaybeUninit::uninit(); RECV_BUF];
                while let Ok(Some(n)) = recv(&difop, &stopper, &mut buf) {
                    stats.difop_packets.fetch_add(1, Ordering::Relaxed);
                    if let Ok(d) = Difop::parse(bytes(&buf, n)) {
                        *lock(&imu_extrinsic) = Some(d.imu_quat_xyzw);
                    }
                    if difop_tx.send(bytes(&buf, n).to_vec()).is_err() {
                        break;
                    }
                }
            })?);
        }

        if let Some(imu) = imu {
            let (stats, stopper) = (stats.clone(), stopper.clone());
            let decoder = Decoder::new(config.decoder.clone());
            let in_lidar_frame = config.imu_in_lidar_frame;
            threads.push(std::thread::Builder::new().name("airy-imu".into()).spawn(move || {
                let mut buf = [MaybeUninit::uninit(); RECV_BUF];
                while let Ok(Some(n)) = recv(&imu, &stopper, &mut buf) {
                    stats.imu_packets.fetch_add(1, Ordering::Relaxed);
                    let sample = decoder.decode_imu(bytes(&buf, n)).map(|sample| {
                        if !in_lidar_frame {
                            return Some(sample);
                        }
                        let q = *lock(&imu_extrinsic);
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
            let (stats, stopper, frame_id) = (stats.clone(), stopper.clone(), frame_id.clone());
            let filter_config = config.filter.clone();
            let mut decoder = Decoder::new(config.decoder.clone());
            threads.push(std::thread::Builder::new().name("airy-msop".into()).spawn(move || {
                let mut buf = [MaybeUninit::uninit(); RECV_BUF];
                let mut scan_index: usize = 0;
                while let Ok(Some(n)) = recv(&msop, &stopper, &mut buf) {
                    while let Ok(d) = difop_rx.try_recv() {
                        if decoder.decode_difop(&d).is_err() {
                            stats.decode_errors.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    stats.msop_packets.fetch_add(1, Ordering::Relaxed);
                    let scan = match decoder.decode_msop(bytes(&buf, n)) {
                        Ok(Some(scan)) => scan,
                        Ok(None) => continue,
                        Err(DecodeError::NoDifop) => {
                            stats.msop_waiting.fetch_add(1, Ordering::Relaxed);
                            continue;
                        }
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

        Ok(Sensor { scans, imu: imu_rx, stats, stopper, threads, frame_id })
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
        Ok(StopHandle { stopper: self.stopper.clone() })
    }

    /// Stops the threads; on unix (where they wait on their socket and a wake pipe together) also joins them.
    pub fn stop(&mut self) {
        self.stopper.stop();
        let threads = std::mem::take(&mut self.threads);
        if cfg!(unix) {
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
    stopper: Arc<Stopper>,
}

impl StopHandle {
    /// Sets the stop flag and wakes the receive threads (see `Sensor::stop`); does not join them.
    pub fn stop(&self) {
        self.stopper.stop();
    }
}


#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use std::os::unix::thread::JoinHandleExt;
    use std::time::Duration;

    extern "C" fn ignore(_: libc::c_int) {}

    #[test]
    fn a_signal_mid_wait_is_retried_not_the_end_of_the_thread() {
        // a handler without SA_RESTART makes the parked syscall fail with EINTR, which used to end the receive loop
        // SAFETY: installs a handler that does nothing, for SIGUSR1 only, which nothing else in the tests uses
        unsafe {
            let mut action: libc::sigaction = std::mem::zeroed();
            action.sa_sigaction = ignore as extern "C" fn(libc::c_int) as libc::sighandler_t;
            action.sa_flags = 0;
            assert_eq!(libc::sigaction(libc::SIGUSR1, &action, std::ptr::null_mut()), 0);
        }
        let config = SensorConfig { interface: Ipv4Addr::LOCALHOST, ..Default::default() };
        let socket = open_socket(&config, 0, None).unwrap();
        let port = socket.local_addr().unwrap().as_socket_ipv4().unwrap().port();
        let stopper = Arc::new(Stopper::new(&[&socket]).unwrap());
        let thread_stopper = stopper.clone();
        let waiter = std::thread::spawn(move || {
            let mut buf = [MaybeUninit::uninit(); RECV_BUF];
            recv(&socket, &thread_stopper, &mut buf).map(|n| n.map(|n| bytes(&buf, n).to_vec()))
        });
        for _ in 0..20 {
            // SAFETY: the thread is alive until it receives the datagram sent below
            unsafe { libc::pthread_kill(waiter.as_pthread_t(), libc::SIGUSR1) };
            std::thread::sleep(Duration::from_millis(5));
        }
        std::net::UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap().send_to(b"hi", (Ipv4Addr::LOCALHOST, port)).unwrap();
        assert_eq!(waiter.join().unwrap().unwrap(), Some(b"hi".to_vec()));
        stopper.stop();
    }
}
