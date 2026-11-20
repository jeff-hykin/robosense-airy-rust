# robosense-airy

Rust driver for the RoboSense Airy (`RSAIRY`) lidar: decodes MSOP (points), DIFOP (device info + beam calibration) and
IMU packets, a faithful port of RoboSense's [rs_driver](https://github.com/RoboSense-LiDAR/rs_driver), and a
multicast/unicast receiver that yields complete scans and IMU samples.

```rust
use robosense_airy::{Sensor, SensorConfig};

let mut config = SensorConfig::default();
config.group = Some("224.10.10.201".parse()?);    // None = unicast
config.interface = "10.21.33.103".parse()?;       // NIC address to join the group on
(config.msop_port, config.difop_port, config.imu_port) = (6691, 7781, Some(6681));
config.frame_id = "front_lidar".into();
config.decoder.use_lidar_clock = true;            // PTP-synced lidar time (false: host time at receipt)
config.decoder.dense_points = true;               // drop out-of-range returns instead of NaN points
config.filter.voxel = 0.1;                        // metres, 0 = off: first point per voxel kept
config.filter.decimate_points = 1;                // keep every Nth point of a scan
config.filter.decimate_scans = 1;                 // keep every Nth scan
let sensor = Sensor::open(config)?;
for framed in sensor.scans() {
    // framed.scan.points: x, y, z (m, lidar frame), intensity, ring (0 = lowest beam), time (s), feature
}
```

The sockets set `SO_REUSEADDR`/`SO_REUSEPORT`, so a vendor driver bound to the same ports keeps receiving. Scans and
IMU samples go through bounded queues (`scan_queue`, `imu_queue`); when the consumer is behind, the new item is dropped
and counted in `Sensor::stats()`, never queued up or throttled by a timer. The first scan after startup (begun
mid-revolution) is skipped. `imu_in_lidar_frame` rotates IMU samples by the DIFOP's IMU extrinsic. `Decoder` can be
used directly on packets from any source; `pcap::read_udp` reads tcpdump captures.

`airy-stats` prints scan rate, points per scan, bounds and IMU rate:

```
airy-stats live --lidar front:224.10.10.201:6691:7781:6681 --lidar rear:224.10.10.202:6692:7782:6682 --interface 10.21.33.103 --seconds 30
airy-stats pcap capture.pcap --msop-port 6691 --difop-port 7781 --imu-port 6681
```

## Tracks

rs_driver [@ 897b14d](https://github.com/RoboSense-LiDAR/rs_driver/tree/897b14d3bdb6186a75df27ba51b65b5bd5557723/src/rs_driver/driver/decoder):
[decoder_RSAIRY.hpp](https://github.com/RoboSense-LiDAR/rs_driver/blob/897b14d3bdb6186a75df27ba51b65b5bd5557723/src/rs_driver/driver/decoder/decoder_RSAIRY.hpp),
[decoder_mech.hpp](https://github.com/RoboSense-LiDAR/rs_driver/blob/897b14d3bdb6186a75df27ba51b65b5bd5557723/src/rs_driver/driver/decoder/decoder_mech.hpp),
[decoder.hpp](https://github.com/RoboSense-LiDAR/rs_driver/blob/897b14d3bdb6186a75df27ba51b65b5bd5557723/src/rs_driver/driver/decoder/decoder.hpp),
[chan_angles.hpp](https://github.com/RoboSense-LiDAR/rs_driver/blob/897b14d3bdb6186a75df27ba51b65b5bd5557723/src/rs_driver/driver/decoder/chan_angles.hpp),
[block_iterator.hpp](https://github.com/RoboSense-LiDAR/rs_driver/blob/897b14d3bdb6186a75df27ba51b65b5bd5557723/src/rs_driver/driver/decoder/block_iterator.hpp),
[split_strategy.hpp](https://github.com/RoboSense-LiDAR/rs_driver/blob/897b14d3bdb6186a75df27ba51b65b5bd5557723/src/rs_driver/driver/decoder/split_strategy.hpp),
[section.hpp](https://github.com/RoboSense-LiDAR/rs_driver/blob/897b14d3bdb6186a75df27ba51b65b5bd5557723/src/rs_driver/driver/decoder/section.hpp),
[trigon.hpp](https://github.com/RoboSense-LiDAR/rs_driver/blob/897b14d3bdb6186a75df27ba51b65b5bd5557723/src/rs_driver/driver/decoder/trigon.hpp),
[basic_attr.hpp](https://github.com/RoboSense-LiDAR/rs_driver/blob/897b14d3bdb6186a75df27ba51b65b5bd5557723/src/rs_driver/driver/decoder/basic_attr.hpp).
Not ported: the 192-beam model, pcap/raw/jumbo inputs, packet-level `isNewFrame` pre-splitting, CRC32 checks.

Checked against rs_driver itself: `tools/rs_driver_harness.cpp` builds rs_driver's `DecoderRSAIRY` on a capture
(`examples/rs_driver_compare.rs` writes its input and this crate's output, `tools/compare.py` diffs them). On the
fixtures every scan, point count, intensity, ring, per-point time, feature and IMU value is identical and x/y/z agree to
2 um (float rounding). Tests (`cargo test`) decode packets captured from the two Airys on a Deep Robotics M20
(`tests/fixtures/`).

`nix build` builds the library and `airy-stats` (crate2nix); `nix build .#aarch64-linux` cross-compiles it from
x86_64 Linux.

## License

Apache-2.0. The decoder is ported from rs_driver (BSD-3-Clause, RoboSense); see `LICENSE-THIRD-PARTY`.
