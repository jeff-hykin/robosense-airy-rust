//! Regressions: split angles outside 0..360, an empty datagram, stopping a sensor that receives nothing.

use robosense_airy::pcap::{parse_udp, UdpDatagram};
use robosense_airy::{Decoder, DecoderConfig, Scan, Sensor, SensorConfig, SplitMode};
use std::net::{Ipv4Addr, UdpSocket};
use std::time::Duration;

fn front_msop() -> Vec<UdpDatagram> {
    parse_udp(include_bytes!("fixtures/airy_msop_front.pcap")).unwrap().into_iter().filter(|d| d.dst_port == 6691).collect()
}

fn front_difop() -> UdpDatagram {
    parse_udp(include_bytes!("fixtures/airy_difop_imu.pcap")).unwrap().into_iter().find(|d| d.dst_port == 7781).unwrap()
}

fn scans(split: SplitMode) -> Vec<Scan> {
    let mut decoder = Decoder::new(DecoderConfig { split, dense_points: true, ..Default::default() });
    decoder.decode_difop(&front_difop().payload).unwrap();
    front_msop().iter().filter_map(|d| decoder.decode_msop(&d.payload).unwrap()).collect()
}

#[test]
fn split_angles_are_taken_mod_360() {
    let at_zero = scans(SplitMode::Angle(0.0));
    assert!(at_zero.len() >= 2);
    let sizes = |s: &[Scan]| s.iter().map(|s| s.points.len()).collect::<Vec<_>>();
    // 360 never matched an azimuth (0..36000) before, so the scan grew forever
    assert_eq!(sizes(&scans(SplitMode::Angle(360.0))), sizes(&at_zero));
    assert_eq!(sizes(&scans(SplitMode::Angle(-270.0))), sizes(&scans(SplitMode::Angle(90.0))));
}

fn free_port() -> u16 {
    UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap().local_addr().unwrap().port()
}

fn unicast_config() -> SensorConfig {
    SensorConfig {
        interface: Ipv4Addr::LOCALHOST,
        msop_port: free_port(),
        difop_port: free_port(),
        imu_port: Some(free_port()),
        decoder: DecoderConfig { split: SplitMode::CustomBlocks(16), dense_points: true, ..Default::default() },
        scan_queue: 64,
        ..Default::default()
    }
}

#[test]
fn an_empty_datagram_does_not_end_the_receive_thread() {
    let config = unicast_config();
    let sensor = Sensor::open(config.clone()).unwrap();
    let tx = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    tx.send_to(&front_difop().payload, (Ipv4Addr::LOCALHOST, config.difop_port)).unwrap();
    // an empty datagram used to read as the shutdown wakeup and end the thread
    tx.send_to(&[], (Ipv4Addr::LOCALHOST, config.msop_port)).unwrap();
    // the DIFOP reaches the decoder with the next MSOP datagram, so resend until a scan arrives
    let packets = front_msop();
    for d in packets.iter().cycle().take(packets.len() * 4) {
        tx.send_to(&d.payload, (Ipv4Addr::LOCALHOST, config.msop_port)).unwrap();
        if sensor.scans().try_recv().is_ok() {
            return;
        }
    }
    sensor.scans().recv_timeout(Duration::from_secs(5)).expect("no scan after an empty datagram");
}

#[test]
fn a_stop_handle_wakes_a_silent_sensor() {
    let mut sensor = Sensor::open(unicast_config()).unwrap();
    let stopper = sensor.stop_handle().unwrap();
    let imu = sensor.take_imu();
    let consumer = std::thread::spawn(move || {
        // parked: nothing is sent; the queue disconnects once the threads end
        let scans_ended = sensor.scans().recv().is_err();
        (scans_ended, imu.recv().is_err())
    });
    stopper.stop();
    let (done_tx, done_rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || done_tx.send(consumer.join().unwrap()));
    // before, only Linux woke a thread parked in recv; macOS's stayed parked until the next datagram
    assert_eq!(done_rx.recv_timeout(Duration::from_secs(5)).expect("receive threads did not wake"), (true, true));
}
