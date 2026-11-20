//! Prints scan statistics from a live Airy or a pcap capture.
//!
//! airy-stats live --lidar front:224.10.10.201:6691:7781:6681 [--lidar ...] --interface 10.21.33.103 [--seconds 30]
//!     [--scan-queue N] [--imu-queue N]
//! airy-stats pcap capture.pcap [more.pcap ...] [--msop-port 6691 --difop-port 7781 --imu-port 6681]
//! common: --voxel M  --decimate-points N  --decimate-scans N  --host-clock  --min-distance M --max-distance M
//! live also: --interval S (seconds between report lines, default 1)

#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used, clippy::panic))]

use robosense_airy::packet::PacketKind;
use robosense_airy::{classify, pcap, Decoder, DecoderConfig, FilterConfig, ImuSample, Scan, Sensor, SensorConfig};
use std::net::Ipv4Addr;
use std::process::exit;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

fn usage() -> ! {
    eprintln!(
        "usage:\n  airy-stats live --lidar NAME:GROUP:MSOP:DIFOP[:IMU] [--lidar ...] [--interface ADDR] [--seconds S] [--scan-queue N] [--imu-queue N] [--interval S]\n  \
         airy-stats pcap FILE... [--msop-port P] [--difop-port P] [--imu-port P]\n\
         options: --voxel M --decimate-points N --decimate-scans N --host-clock --min-distance M --max-distance M"
    );
    exit(2)
}

#[derive(Default, Clone)]
struct Window {
    scans: u64,
    points: u64,
    min_points: u64,
    max_points: u64,
    lo: [f32; 3],
    hi: [f32; 3],
    max_range: f32,
    rings: u16,
    imu: u64,
    accel_norm_sum: f64,
    gyro_norm_max: f32,
    // host clock minus the scan's stamp, seconds
    lag_sum: f64,
    scan_span_sum: f64,
}

impl Window {
    fn add_scan(&mut self, scan: &Scan) {
        let n = scan.points.len() as u64;
        if self.scans == 0 {
            self.min_points = n;
            self.lo = [f32::INFINITY; 3];
            self.hi = [f32::NEG_INFINITY; 3];
        }
        self.scans += 1;
        self.points += n;
        self.min_points = self.min_points.min(n);
        self.max_points = self.max_points.max(n);
        self.rings = self.rings.max(scan.points.iter().map(|p| p.ring + 1).max().unwrap_or(0));
        for p in scan.points.iter().filter(|p| p.x.is_finite()) {
            for (i, v) in [p.x, p.y, p.z].into_iter().enumerate() {
                self.lo[i] = self.lo[i].min(v);
                self.hi[i] = self.hi[i].max(v);
            }
            self.max_range = self.max_range.max((p.x * p.x + p.y * p.y + p.z * p.z).sqrt());
        }
        if let (Some(a), Some(b)) = (scan.points.first(), scan.points.last()) {
            self.scan_span_sum += b.time - a.time;
        }
        self.lag_sum += now() - scan.timestamp;
    }

    fn add_imu(&mut self, s: &ImuSample) {
        self.imu += 1;
        let a = s.accel_g;
        self.accel_norm_sum += ((a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) as f64).sqrt();
        let g = s.gyro_rad_s;
        self.gyro_norm_max = self.gyro_norm_max.max((g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt());
    }

    fn merge_imu(&mut self, other: &Window) {
        self.imu += other.imu;
        self.accel_norm_sum += other.accel_norm_sum;
        self.gyro_norm_max = self.gyro_norm_max.max(other.gyro_norm_max);
    }

    fn line(&self, name: &str, seconds: f64, imu_seconds: f64, with_lag: bool) -> String {
        let mean = |v: f64, n: u64| if n > 0 { v / n as f64 } else { 0.0 };
        let mut s = format!(
            "{name}: {:.2} scans/s, points/scan mean {:.0} min {} max {}, rings {}, x [{:.2}, {:.2}] y [{:.2}, {:.2}] \
             z [{:.2}, {:.2}], max range {:.2} m, scan span {:.1} ms, imu {:.1} Hz (|a| {:.3} g, max |w| {:.3} rad/s)",
            self.scans as f64 / seconds,
            mean(self.points as f64, self.scans),
            self.min_points,
            self.max_points,
            self.rings,
            self.lo[0],
            self.hi[0],
            self.lo[1],
            self.hi[1],
            self.lo[2],
            self.hi[2],
            self.max_range,
            mean(self.scan_span_sum, self.scans) * 1e3,
            self.imu as f64 / imu_seconds,
            mean(self.accel_norm_sum, self.imu),
            self.gyro_norm_max,
        );
        if with_lag {
            s += &format!(", host - stamp {:.1} ms", mean(self.lag_sum, self.scans) * 1e3);
        }
        s
    }
}

fn now() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default().as_secs_f64()
}

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

struct Args {
    mode: String,
    rest: Vec<String>,
    lidars: Vec<String>,
    interface: Ipv4Addr,
    seconds: Option<f64>,
    interval: f64,
    queues: (usize, usize),
    ports: [u16; 3],
    decoder: DecoderConfig,
    filter: FilterConfig,
}

fn parse() -> Args {
    let mut it = std::env::args().skip(1);
    let mode = it.next().unwrap_or_else(|| usage());
    let mut a = Args {
        mode,
        rest: vec![],
        lidars: vec![],
        interface: Ipv4Addr::UNSPECIFIED,
        seconds: None,
        interval: 1.0,
        queues: (1, 8),
        ports: [6699, 7788, 6688],
        decoder: DecoderConfig { use_lidar_clock: true, dense_points: true, ts_first_point: true, ..Default::default() },
        filter: FilterConfig::default(),
    };
    fn val<T: std::str::FromStr>(it: &mut impl Iterator<Item = String>) -> T {
        it.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| usage())
    }
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--lidar" => a.lidars.push(val(&mut it)),
            "--interface" => a.interface = val(&mut it),
            "--seconds" => a.seconds = Some(val(&mut it)),
            "--interval" => a.interval = val(&mut it),
            "--scan-queue" => a.queues.0 = val(&mut it),
            "--imu-queue" => a.queues.1 = val(&mut it),
            "--msop-port" => a.ports[0] = val(&mut it),
            "--difop-port" => a.ports[1] = val(&mut it),
            "--imu-port" => a.ports[2] = val(&mut it),
            "--voxel" => a.filter.voxel = val(&mut it),
            "--decimate-points" => a.filter.decimate_points = val(&mut it),
            "--decimate-scans" => a.filter.decimate_scans = val(&mut it),
            "--host-clock" => a.decoder.use_lidar_clock = false,
            "--min-distance" => a.decoder.min_distance = val(&mut it),
            "--max-distance" => a.decoder.max_distance = val(&mut it),
            "-h" | "--help" => usage(),
            other if other.starts_with("--") => usage(),
            other => a.rest.push(other.to_string()),
        }
    }
    a
}

fn run_pcap(a: &Args) {
    let mut decoder = Decoder::new(a.decoder.clone());
    let mut window = Window::default();
    let (mut first, mut last) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut imu_first, mut imu_last) = (f64::INFINITY, f64::NEG_INFINITY);
    let mut imu_window = Window::default();
    let mut datagrams = Vec::new();
    for path in &a.rest {
        datagrams.extend(pcap::read_udp(path).unwrap_or_else(|e| {
            eprintln!("{path}: {e}");
            exit(1)
        }));
    }
    // DIFOP first: a capture's DIFOP may come after its first MSOP packets
    datagrams.sort_by(|x, y| x.time.total_cmp(&y.time));
    for d in datagrams.iter().filter(|d| d.dst_port == a.ports[1]) {
        let _ = decoder.decode_difop(&d.payload);
    }
    let mut scan_index = 0usize;
    for d in &datagrams {
        match classify(&d.payload) {
            Some(PacketKind::Msop) if d.dst_port == a.ports[0] => {
                first = first.min(d.time);
                last = last.max(d.time);
                if let Ok(Some(mut scan)) = decoder.decode_msop(&d.payload) {
                    if !scan.complete {
                        continue;
                    }
                    scan_index += 1;
                    if !(scan_index - 1).is_multiple_of(a.filter.decimate_scans.max(1)) {
                        continue;
                    }
                    robosense_airy::filter::apply(&mut scan.points, &a.filter);
                    println!("scan {}: {} points, stamp {:.6}", scan.seq, scan.points.len(), scan.timestamp);
                    window.add_scan(&scan);
                }
            }
            Some(PacketKind::Imu) if d.dst_port == a.ports[2] => {
                if let Ok(s) = decoder.decode_imu(&d.payload) {
                    imu_first = imu_first.min(d.time);
                    imu_last = imu_last.max(d.time);
                    imu_window.add_imu(&s);
                }
            }
            _ => {}
        }
    }
    if let Some(difop) = decoder.difop() {
        println!(
            "difop: rpm {} install_mode {} return_mode {} fov [{}, {}] sn {:02x?}",
            difop.rpm, difop.install_mode, difop.return_mode, difop.fov_start, difop.fov_end, difop.serial
        );
    }
    // the IMU's rate over its own capture span, which can differ from the points'
    let imu_span = (imu_last - imu_first).max(1e-9);
    let scan_span = (last - first).max(1e-9);
    window.merge_imu(&imu_window);
    println!("{}", window.line("pcap", scan_span, imu_span, false));
}

fn run_live(a: &Args) {
    if a.lidars.is_empty() {
        usage();
    }
    let mut sensors = Vec::new();
    for spec in &a.lidars {
        let parts: Vec<&str> = spec.split(':').collect();
        if parts.len() < 4 {
            usage();
        }
        let port = |s: &str| s.parse::<u16>().unwrap_or_else(|_| usage());
        let config = SensorConfig {
            group: Some(parts[1].parse().unwrap_or_else(|_| usage())),
            interface: a.interface,
            msop_port: port(parts[2]),
            difop_port: port(parts[3]),
            imu_port: parts.get(4).map(|p| port(p)),
            frame_id: parts[0].to_string(),
            decoder: a.decoder.clone(),
            filter: a.filter.clone(),
            scan_queue: a.queues.0,
            imu_queue: a.queues.1,
            ..Default::default()
        };
        let mut sensor = Sensor::open(config).unwrap_or_else(|e| {
            eprintln!("{spec}: {e}");
            exit(1)
        });
        // scans and IMU samples tally under separate locks, so a scan being tallied never holds up the IMU consumer
        let windows: [Arc<Mutex<Window>>; 4] = Default::default();
        let stats = sensor.stats_handle();
        let imu = sensor.take_imu();
        {
            let (window, total) = (windows[2].clone(), windows[3].clone());
            std::thread::spawn(move || {
                for s in imu.iter() {
                    lock(&window).add_imu(&s);
                    lock(&total).add_imu(&s);
                }
            });
        }
        {
            let (window, total) = (windows[0].clone(), windows[1].clone());
            std::thread::spawn(move || {
                for f in sensor.scans().iter() {
                    lock(&window).add_scan(&f.scan);
                    lock(&total).add_scan(&f.scan);
                }
            });
        }
        sensors.push((parts[0].to_string(), stats, windows));
    }
    if !(a.interval.is_finite() && a.interval > 0.0) {
        usage();
    }
    let period = Duration::from_secs_f64(a.interval);
    let start = Instant::now();
    let mut tick = Instant::now();
    let mut deadline = start;
    loop {
        // report cadence only (nothing waits on it); deadline-based so the lines do not drift
        deadline += period;
        std::thread::sleep(deadline.saturating_duration_since(Instant::now()));
        let dt = tick.elapsed().as_secs_f64();
        tick = Instant::now();
        for (name, _, windows) in &sensors {
            let mut w = std::mem::take(&mut *lock(&windows[0]));
            w.merge_imu(&std::mem::take(&mut *lock(&windows[2])));
            println!("{}", w.line(name, dt, dt, true));
        }
        if a.seconds.is_some_and(|s| start.elapsed().as_secs_f64() >= s) {
            break;
        }
    }
    let secs = start.elapsed().as_secs_f64();
    println!("--- {secs:.1} s total");
    for (name, s, windows) in &sensors {
        let mut total = lock(&windows[1]).clone();
        total.merge_imu(&lock(&windows[3]));
        let get = |c: &std::sync::atomic::AtomicU64| c.load(std::sync::atomic::Ordering::Relaxed);
        println!("{}", total.line(name, secs, secs, true));
        println!(
            "{name}: msop {} ({:.0}/s, {} before the first difop) difop {} imu {} decode_errors {} scans {} dropped {} \
             skipped {} imu_dropped {}",
            get(&s.msop_packets),
            get(&s.msop_packets) as f64 / secs,
            get(&s.msop_waiting),
            get(&s.difop_packets),
            get(&s.imu_packets),
            get(&s.decode_errors),
            get(&s.scans),
            get(&s.scans_dropped),
            get(&s.scans_skipped),
            get(&s.imu_dropped),
        );
    }
    exit(0);
}

fn main() {
    let a = parse();
    match a.mode.as_str() {
        "pcap" => run_pcap(&a),
        "live" => run_live(&a),
        _ => usage(),
    }
}
