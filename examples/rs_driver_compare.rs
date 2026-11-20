// Writes a capture's datagrams as [port u16][len u16][payload] (rs_driver harness input) and prints this crate's
// decode in the harness's text format: rs_driver_compare DUMP_OUT MSOP DIFOP IMU DENSE PCAP...
use robosense_airy::{Decoder, DecoderConfig};
use std::io::Write;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let port = |i: usize| a[i].parse::<u16>().unwrap();
    let (msop, difop, imu) = (port(2), port(3), port(4));
    let mut all: Vec<_> = a[6..].iter().flat_map(|p| robosense_airy::pcap::read_udp(p).unwrap()).collect();
    all.sort_by(|x, y| x.time.total_cmp(&y.time));
    let mut dump = std::fs::File::create(&a[1]).unwrap();
    let config = DecoderConfig { min_distance: 0.2, max_distance: 60.0, use_lidar_clock: true, dense_points: a[5] == "1", ts_first_point: true, ..Default::default() };
    let mut d = Decoder::new(config);
    let out = std::io::stdout();
    let mut out = std::io::BufWriter::new(out.lock());
    for g in &all {
        dump.write_all(&g.dst_port.to_le_bytes()).unwrap();
        dump.write_all(&(g.payload.len() as u16).to_le_bytes()).unwrap();
        dump.write_all(&g.payload).unwrap();
        if g.dst_port == difop {
            d.decode_difop(&g.payload).unwrap();
        } else if g.dst_port == msop {
            if let Ok(Some(s)) = d.decode_msop(&g.payload) {
                writeln!(out, "S {} {:.6} {}", s.seq, s.timestamp, s.points.len()).unwrap();
                for p in &s.points {
                    writeln!(out, "{:.6} {:.6} {:.6} {} {} {:.6} {}", p.x, p.y, p.z, p.intensity, p.ring, p.time, p.feature).unwrap();
                }
            }
        } else if g.dst_port == imu {
            let m = d.decode_imu(&g.payload).unwrap();
            writeln!(out, "I {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6}", m.timestamp, m.accel_g[0], m.accel_g[1], m.accel_g[2], m.gyro_rad_s[0], m.gyro_rad_s[1], m.gyro_rad_s[2]).unwrap();
        }
    }
}
