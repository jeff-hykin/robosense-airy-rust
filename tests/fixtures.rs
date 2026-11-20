//! Decoding real packets captured from the Deep Robotics M20's two Airys (tcpdump on its AOS board, eth0).

use robosense_airy::filter::{decimate, voxel_downsample};
use robosense_airy::pcap::{parse_udp, UdpDatagram};
use robosense_airy::{Decoder, DecoderConfig, LidarModel, Scan};


struct Lidar {
    msop_file: &'static str,
    msop_port: u16,
    difop_port: u16,
    imu_port: u16,
    ip: [u8; 4],
}

const FRONT: Lidar = Lidar { msop_file: "airy_msop_front.pcap", msop_port: 6691, difop_port: 7781, imu_port: 6681, ip: [10, 21, 33, 201] };
const REAR: Lidar = Lidar { msop_file: "airy_msop_rear.pcap", msop_port: 6692, difop_port: 7782, imu_port: 6682, ip: [10, 21, 33, 202] };

// embedded, so the test binary runs without the source tree (nix runs tests in a separate derivation)
fn datagrams(file: &str) -> Vec<UdpDatagram> {
    let bytes: &[u8] = match file {
        "airy_difop_imu.pcap" => include_bytes!("fixtures/airy_difop_imu.pcap"),
        "airy_msop_front.pcap" => include_bytes!("fixtures/airy_msop_front.pcap"),
        "airy_msop_rear.pcap" => include_bytes!("fixtures/airy_msop_rear.pcap"),
        other => panic!("no fixture {other}"),
    };
    parse_udp(bytes).unwrap()
}

fn decoder_with_difop(lidar: &Lidar, config: DecoderConfig) -> Decoder {
    let mut decoder = Decoder::new(config);
    let difop = datagrams("airy_difop_imu.pcap").into_iter().find(|d| d.dst_port == lidar.difop_port).unwrap();
    decoder.decode_difop(&difop.payload).unwrap();
    decoder
}

fn complete_scans(lidar: &Lidar, config: DecoderConfig) -> (Decoder, Vec<Scan>) {
    let mut decoder = decoder_with_difop(lidar, config);
    let mut scans = Vec::new();
    for d in datagrams(lidar.msop_file).iter().filter(|d| d.dst_port == lidar.msop_port) {
        if let Some(scan) = decoder.decode_msop(&d.payload).unwrap() {
            scans.push(scan);
        }
    }
    assert!(!scans[0].complete, "the capture starts mid-revolution");
    (decoder, scans.into_iter().filter(|s| s.complete).collect())
}

fn vendor_config() -> DecoderConfig {
    // the M20's /opt/robot/share/node_driver/config/config.yaml
    DecoderConfig { min_distance: 0.2, max_distance: 60.0, use_lidar_clock: true, dense_points: true, ts_first_point: true, ..Default::default() }
}

#[test]
fn difop_parses() {
    for lidar in [FRONT, REAR] {
        let decoder = decoder_with_difop(&lidar, DecoderConfig::default());
        let difop = decoder.difop().unwrap();
        assert_eq!(difop.rpm, 600);
        assert_eq!(decoder.rps(), 10);
        assert_eq!(difop.lidar_ip, lidar.ip);
        assert_eq!((difop.msop_port, difop.difop_port), (lidar.msop_port, lidar.difop_port));
        assert_eq!((difop.fov_start, difop.fov_end), (0, 36000));
        assert_eq!(difop.install_mode, 1, "side-mounted firing table");
        assert!(decoder.angles_ready());
        let (vert, horiz, rings) = decoder.beam_angles();
        assert_eq!(vert.len(), 96);
        // a hemispherical Airy: beams from just above 0 to just under 90 degrees, in firing order bottom to top
        assert!(vert.windows(2).all(|w| w[0] < w[1]));
        assert!(vert[0] > -200 && vert[95] < 9000);
        assert!(horiz.iter().all(|h| h.abs() < 4000));
        let mut sorted = rings.to_vec();
        sorted.sort();
        assert_eq!(sorted, (0..96).collect::<Vec<u16>>());
        // IMU extrinsic is a unit quaternion
        let q = difop.imu_quat_xyzw;
        assert!(((q.iter().map(|v| v * v).sum::<f32>()).sqrt() - 1.0).abs() < 1e-3);
    }
}

#[test]
fn msop_waits_for_difop() {
    let mut decoder = Decoder::new(DecoderConfig::default());
    let pkt = datagrams(FRONT.msop_file).into_iter().next().unwrap();
    assert_eq!(decoder.decode_msop(&pkt.payload), Err(robosense_airy::decoder::DecodeError::NoDifop));
}

#[test]
fn full_scans_have_every_return() {
    // not dense: one point (NaN when out of range) per beam per firing, 96 beams x 900 firings at 10 Hz
    for lidar in [FRONT, REAR] {
        let (decoder, scans) = complete_scans(&lidar, DecoderConfig { dense_points: false, ..vendor_config() });
        assert_eq!(decoder.model(), LidarModel::Channel96);
        assert!(scans.len() >= 2, "{} complete scans", scans.len());
        for scan in &scans {
            assert_eq!(scan.rings, 96);
            assert!((scan.points.len() as i64 - 96 * 900).abs() <= 96 * 2, "{} points", scan.points.len());
            assert_eq!(scan.points.len() % 96, 0);
        }
    }
}

#[test]
fn dense_scans_are_plausible() {
    for lidar in [FRONT, REAR] {
        let (_, scans) = complete_scans(&lidar, vendor_config());
        for pair in scans.windows(2) {
            let dt = pair[1].timestamp - pair[0].timestamp;
            assert!((dt - 0.1).abs() < 0.002, "scan period {dt}");
            assert_eq!(pair[1].seq, pair[0].seq + 1);
        }
        for scan in &scans {
            // the robot's surroundings fill roughly half the dome
            assert!((20_000..86_400).contains(&scan.points.len()), "{} points", scan.points.len());
            let first = scan.points.first().unwrap().time;
            let last = scan.points.last().unwrap().time;
            assert!((last - first - 0.1).abs() < 0.002, "scan span {}", last - first);
            assert!((first - scan.timestamp).abs() < 0.001);
            let mut max_range = 0f32;
            for p in &scan.points {
                let range = (p.x * p.x + p.y * p.y + p.z * p.z).sqrt();
                // lens-center offsets put the origin a few cm off the optical center
                assert!((0.1..60.2).contains(&range), "range {range}");
                assert!(p.ring < 96);
                assert!(p.time >= first - 1e-9 && p.time <= last + 1e-9);
                max_range = max_range.max(range);
            }
            assert!(max_range > 3.0, "max range {max_range}");
            // the hemisphere: everything is on the dome's side of the lidar's xy plane (z up in its own frame), the
            // lowest beam a fraction of a degree below it
            assert!(scan.points.iter().all(|p| p.z > -0.05 - 0.035 * (p.x * p.x + p.y * p.y).sqrt()));
            // lidar time is PTP-synced to the capture host's clock: stamps near the capture times
            let capture = datagrams(lidar.msop_file);
            let (lo, hi) = (capture.first().unwrap().time, capture.last().unwrap().time);
            assert!(scan.timestamp > lo - 0.5 && scan.timestamp < hi + 0.5);
        }
    }
}

#[test]
fn imu_rate_and_gravity() {
    for lidar in [FRONT, REAR] {
        let decoder = decoder_with_difop(&lidar, vendor_config());
        let q = decoder.difop().unwrap().imu_quat_xyzw;
        let samples: Vec<_> = datagrams("airy_difop_imu.pcap")
            .iter()
            .filter(|d| d.dst_port == lidar.imu_port)
            .map(|d| decoder.decode_imu(&d.payload).unwrap())
            .collect();
        assert!(samples.len() > 500);
        let span = samples.last().unwrap().timestamp - samples[0].timestamp;
        let rate = (samples.len() - 1) as f64 / span;
        assert!((rate - 200.0).abs() < 5.0, "imu rate {rate}");
        assert!(samples.windows(2).all(|w| w[1].cnt == w[0].cnt.wrapping_add(1)));
        for s in &samples {
            let a = s.accel_g;
            let norm = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
            assert!((norm - 1.0).abs() < 0.05, "|a| = {norm} g while standing");
            assert!(s.gyro_rad_s.iter().all(|w| w.abs() < 0.2));
            // both Airys look out an end of the body with their lidar x axis up
            let up = s.rotated(q).accel_g;
            assert!(up[0] > 0.95 && up[1].abs() < 0.1 && up[2].abs() < 0.1, "{up:?}");
        }
    }
}

#[test]
fn host_clock_option() {
    let mut decoder = decoder_with_difop(&FRONT, DecoderConfig { use_lidar_clock: false, ..vendor_config() });
    let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_secs_f64();
    let scan = datagrams(FRONT.msop_file).iter().find_map(|d| decoder.decode_msop(&d.payload).unwrap()).unwrap();
    assert!(scan.points.iter().all(|p| (p.time - now).abs() < 5.0));
}

#[test]
fn filters() {
    let (_, scans) = complete_scans(&FRONT, vendor_config());
    let mut points = scans[0].points.clone();
    let n = points.len();
    decimate(&mut points, 4);
    assert_eq!(points.len(), n.div_ceil(4));
    assert_eq!(points[1], scans[0].points[4]);

    let mut voxels = scans[0].points.clone();
    voxel_downsample(&mut voxels, 0.2);
    assert!(voxels.len() < n / 3 && voxels.len() > 500, "{} of {n}", voxels.len());
    let key = |p: &robosense_airy::Point| ((p.x / 0.2).floor() as i32, (p.y / 0.2).floor() as i32, (p.z / 0.2).floor() as i32);
    let unique: std::collections::HashSet<_> = voxels.iter().map(key).collect();
    assert_eq!(unique.len(), voxels.len());
    let mut unchanged = scans[0].points.clone();
    voxel_downsample(&mut unchanged, 0.0);
    decimate(&mut unchanged, 1);
    assert_eq!(unchanged.len(), n);
}

#[test]
fn angle_window_and_range_filter() {
    let (_, all) = complete_scans(&FRONT, vendor_config());
    let (_, near) = complete_scans(&FRONT, DecoderConfig { max_distance: 2.0, ..vendor_config() });
    assert!(near[0].points.len() < all[0].points.len());
    assert!(near[0].points.iter().all(|p| (p.x * p.x + p.y * p.y + p.z * p.z).sqrt() < 2.1));
    let (_, half) = complete_scans(&FRONT, DecoderConfig { start_angle: 0.0, end_angle: 180.0, ..vendor_config() });
    assert!(half[0].points.len() < all[0].points.len());
}

#[test]
fn both_lidars_slice_at_the_same_azimuths() {
    // ten sectors a turn: each lidar's slices cover 36 degrees and start where the other's do, so two lidars spinning
    // in phase hand over slices measured over the same instants
    let config = DecoderConfig { split: robosense_airy::SplitMode::Sectors(10), ..vendor_config() };
    let mut starts = Vec::new();
    for lidar in [FRONT, REAR] {
        let (_, scans) = complete_scans(&lidar, config.clone());
        assert!(scans.len() >= 5, "{} slices", scans.len());
        let spans: Vec<f64> = scans
            .iter()
            .map(|s| (s.points.iter().map(|p| p.time).fold(f64::MIN, f64::max) - s.timestamp) * 1e3)
            .collect();
        assert!(spans.iter().all(|ms| *ms <= 10.5), "{spans:?}");
        // the front capture lost 5 ms of packets at one turn's start, which leaves one short slice
        assert!(spans.iter().filter(|ms| **ms < 9.5).count() <= 1, "{spans:?}");
        starts.push(scans.iter().map(|s| s.timestamp).collect::<Vec<_>>());
    }
    // the two Airys spin in phase on one PTP clock (a turn starts on the 100 ms), so 36 degree slices start on the
    // same 10 ms grid: the captures were taken apart, so compare where each slice falls on that grid
    let phase_ms = |t: &f64| (t * 1e3).rem_euclid(10.0);
    for (lidar, ts) in starts.iter().enumerate() {
        for t in ts {
            let off = phase_ms(t).min(10.0 - phase_ms(t));
            assert!(off < 0.2, "lidar {lidar}: a slice starts {off:.3} ms off the 10 ms grid");
        }
    }
}
