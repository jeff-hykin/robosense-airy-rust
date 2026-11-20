// Prints the DIFOP and the first IMU samples of a capture: dump_difop FILE DIFOP_PORT IMU_PORT
fn main() {
    let args: Vec<String> = std::env::args().collect();
    let (difop_port, imu_port): (u16, u16) = (args[2].parse().unwrap(), args[3].parse().unwrap());
    let all = robosense_airy::pcap::read_udp(&args[1]).unwrap();
    let mut decoder = robosense_airy::Decoder::new(Default::default());
    if let Some(d) = all.iter().find(|d| d.dst_port == difop_port) {
        decoder.decode_difop(&d.payload).unwrap();
        println!("{:?}", decoder.difop().unwrap());
        let (v, h, r) = decoder.beam_angles();
        println!("vert {v:?}\nhoriz {h:?}\nring {r:?}");
    }
    let mut prev: Option<robosense_airy::ImuSample> = None;
    for d in all.iter().filter(|d| d.dst_port == imu_port).take(12) {
        let s = robosense_airy::packet::ImuPacket::parse(&d.payload).unwrap();
        let t = s.timestamp_us as f64 * 1e-6;
        println!("{s:?} dt {:?} cap {:.6}", prev.map(|p| t - p.timestamp), d.time);
        prev = Some(robosense_airy::ImuSample { timestamp: t, accel_g: s.accel_g, gyro_rad_s: s.gyro_rad_s, cnt: s.cnt });
    }
}
