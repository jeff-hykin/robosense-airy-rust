//! RoboSense Airy (RSAIRY) lidar driver.
//!
//! [`Decoder`] is a port of RoboSense's rs_driver `DecoderRSAIRY`: MSOP packets to points (x, y, z, intensity, ring,
//! per-point time) split into scans, beam angles from DIFOP, IMU packets to accel/gyro. [`Sensor`] receives a live
//! lidar's packets (multicast or unicast) on threads and yields complete scans and IMU samples, busy-dropping when the
//! consumer falls behind. [`pcap`] reads tcpdump captures for tests and replay.
//!
//! ```no_run
//! use robosense_airy::{Sensor, SensorConfig};
//! let mut config = SensorConfig::default();
//! config.group = Some("224.10.10.201".parse().unwrap());
//! config.interface = "10.21.33.103".parse().unwrap();
//! (config.msop_port, config.difop_port, config.imu_port) = (6691, 7781, Some(6681));
//! config.decoder.use_lidar_clock = true;
//! let sensor = Sensor::open(config).unwrap();
//! for framed in sensor.scans().iter().take(10) {
//!     println!("{} points at {:.3}", framed.scan.points.len(), framed.scan.timestamp);
//! }
//! ```

pub mod decoder;
pub mod filter;
pub mod packet;
pub mod pcap;
pub mod sensor;
mod trigon;

pub use decoder::{Decoder, DecoderConfig, EchoMode, ImuSample, LidarModel, Point, Scan, SplitMode, Transform};
pub use filter::FilterConfig;
pub use packet::{classify, Difop, PacketKind};
pub use sensor::{FramedScan, Sensor, SensorConfig, Stats};
