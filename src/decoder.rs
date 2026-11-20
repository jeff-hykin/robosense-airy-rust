// Ported from RoboSense's rs_driver (BSD-3-Clause), see LICENSE-THIRD-PARTY.
//! Port of rs_driver's `DecoderRSAIRY` (+ the `DecoderMech`/`Decoder` parts it uses): MSOP packets to points, split
//! into scans, with the per-beam angles from DIFOP.

use crate::packet::{self, Difop, ImuPacket, MsopPacket, PacketError, BLOCKS_PER_PKT, BLOCK_ID, CHANNELS_PER_BLOCK};
use crate::trigon::Trigon;
use std::time::{SystemTime, UNIX_EPOCH};

/// rs_driver's RS_ONE_ROUND (0.01 degree units).
const ONE_ROUND: i32 = 36000;

// Airy lens center (rs_driver's RX, RY, RZ for RSAIRY).
const LENS_RX: f32 = 0.0075;
const LENS_RY: f32 = 0.00664;
const LENS_RZ: f32 = 0.04532;

// The Airy's distance resolution and valid range (rs_driver's RSAIRY const params).
const DISTANCE_RES: f32 = 0.005;
const DISTANCE_MIN: f32 = 0.1;
const DISTANCE_MAX: f32 = 60.0;

// Firing period of one block pair, microseconds.
const BLOCK_TS_US: f32 = 111.080;

// Channel firing offsets within a block, microseconds, per install mode (rs_driver's firing_tss_* tables).
const FIRING_AIRY_M: [f32; 96] = expand8([
    0.00, 7.616, 16.184, 24.752, 33.320, 42.840, 52.360, 61.880, 71.400, 79.968, 88.536, 97.104,
]);
const FIRING_SIDE: [f32; 96] = expand8([
    0.00, 11.424, 22.848, 34.272, 45.696, 54.264, 62.832, 71.400, 79.016, 85.680, 92.344, 99.008,
]);
const FIRING_NORMAL: [f32; 96] = expand8([
    0.00, 5.712, 12.376, 19.040, 25.704, 33.320, 41.888, 50.456, 59.024, 70.448, 81.872, 93.296,
]);
const FIRING_48L: [f32; 48] = expand4([
    0.00, 7.616, 16.184, 24.752, 33.320, 42.840, 52.360, 61.880, 71.400, 79.968, 88.536, 97.104,
]);

const fn expand8(groups: [f32; 12]) -> [f32; 96] {
    let mut out = [0.0; 96];
    let mut i = 0;
    while i < 96 {
        out[i] = groups[i / 8];
        i += 1;
    }
    out
}

const fn expand4(groups: [f32; 12]) -> [f32; 48] {
    let mut out = [0.0; 48];
    let mut i = 0;
    while i < 48 {
        out[i] = groups[i / 4];
        i += 1;
    }
    out
}

/// How a scan ends (rs_driver's SplitFrameMode).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SplitMode {
    /// A new scan starts when a block's azimuth crosses this angle in degrees (rs_driver's default, 0).
    Angle(f32),
    /// A new scan every revolution's worth of blocks, from the DIFOP rpm.
    FixedBlocks,
    /// A new scan every this many blocks.
    CustomBlocks(u16),
}

/// Optional rigid transform applied to every point (rs_driver's ENABLE_TRANSFORM + RSTransformParam).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Transform {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    /// Radians; the DIFOP's own roll/pitch/yaw are added, as rs_driver does.
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

/// rs_driver's RSDecoderParam fields that apply to the Airy. Defaults are rs_driver's defaults.
#[derive(Debug, Clone, PartialEq)]
pub struct DecoderConfig {
    /// Range filter in metres. Both 0 keeps the Airy's own 0.1..60 m.
    pub min_distance: f32,
    pub max_distance: f32,
    /// true stamps with the lidar's clock (PTP), false with the host clock at receipt.
    pub use_lidar_clock: bool,
    /// true drops out-of-range returns, false keeps them as NaN points.
    pub dense_points: bool,
    /// true stamps a scan with its first point, false with its last.
    pub ts_first_point: bool,
    /// true decodes nothing until a DIFOP packet has given the beam angles.
    pub wait_for_difop: bool,
    /// Azimuth window in degrees; start == end (mod 360) keeps everything.
    pub start_angle: f32,
    pub end_angle: f32,
    pub split: SplitMode,
    pub transform: Option<Transform>,
}

impl Default for DecoderConfig {
    fn default() -> Self {
        DecoderConfig {
            min_distance: 0.0,
            max_distance: 0.0,
            use_lidar_clock: false,
            dense_points: false,
            ts_first_point: false,
            wait_for_difop: true,
            start_angle: 0.0,
            end_angle: 360.0,
            split: SplitMode::Angle(0.0),
            transform: None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: u8,
    /// Beam index ordered by vertical angle, 0 = lowest (rs_driver's `toUserChan`).
    pub ring: u16,
    /// Absolute time of this return, seconds (lidar or host clock per `use_lidar_clock`).
    pub time: f64,
    /// The two "feature" bits above the 14-bit distance.
    pub feature: u8,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Scan {
    pub seq: u32,
    /// Seconds: the first point's time when `ts_first_point`, else the last point's.
    pub timestamp: f64,
    /// Beams of the model (48, 96 or 192): the cloud's height when not dense.
    pub rings: u16,
    pub points: Vec<Point>,
    /// False for the first scan after startup, which began mid-revolution.
    pub complete: bool,
}

/// An IMU sample with its timestamp in seconds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    pub timestamp: f64,
    pub accel_g: [f32; 3],
    pub gyro_rad_s: [f32; 3],
    pub cnt: u32,
}

impl ImuSample {
    /// Linear acceleration in m/s^2 (standard gravity times rs_driver's g).
    pub fn accel_m_s2(&self) -> [f32; 3] {
        const STANDARD_GRAVITY: f32 = 9.80665;
        self.accel_g.map(|a| a * STANDARD_GRAVITY)
    }

    /// The sample with accel and gyro rotated by the unit quaternion `q` (x, y, z, w), e.g. the DIFOP's IMU extrinsic
    /// ([`Difop::imu_quat_xyzw`]) to express it in the lidar frame.
    pub fn rotated(&self, q: [f32; 4]) -> ImuSample {
        ImuSample { accel_g: rotate(q, self.accel_g), gyro_rad_s: rotate(q, self.gyro_rad_s), ..*self }
    }
}

// q * v * q^-1 for a unit quaternion (x, y, z, w)
fn rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let (u, w) = ([q[0], q[1], q[2]], q[3]);
    let cross = |a: [f32; 3], b: [f32; 3]| [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    let t = cross(u, v).map(|c| 2.0 * c);
    let ut = cross(u, t);
    [v[0] + w * t[0] + ut[0], v[1] + w * t[1] + ut[1], v[2] + w * t[2] + ut[2]]
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EchoMode {
    Single,
    Dual,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LidarModel {
    Channel48,
    Channel96,
    Channel192,
}

/// Why a packet produced nothing.
#[derive(Debug, Clone, PartialEq)]
pub enum DecodeError {
    Packet(PacketError),
    /// `wait_for_difop` and no usable DIFOP yet.
    NoDifop,
    /// data_type[0] != 0 (not point data).
    NotPointData,
    WrongBlockId,
    UnsupportedModel,
}

impl std::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DecodeError::Packet(e) => write!(f, "{e}"),
            DecodeError::NoDifop => write!(f, "no DIFOP received yet"),
            DecodeError::NotPointData => write!(f, "not a point data packet"),
            DecodeError::WrongBlockId => write!(f, "wrong MSOP block id"),
            DecodeError::UnsupportedModel => write!(f, "unsupported Airy model (192 channels)"),
        }
    }
}

impl std::error::Error for DecodeError {}

impl From<PacketError> for DecodeError {
    fn from(e: PacketError) -> Self {
        DecodeError::Packet(e)
    }
}

struct AzimuthSection {
    full_round: bool,
    start: i32,
    end: i32,
    cross_zero: bool,
}

impl AzimuthSection {
    fn new(start: i32, end: i32) -> Self {
        let round = |v: i32| (v % ONE_ROUND + ONE_ROUND) % ONE_ROUND;
        let (s, e) = (round(start), round(end));
        AzimuthSection { full_round: round(end - start) == 0, start: s, end: e, cross_zero: s > e }
    }

    fn contains(&self, angle: i32) -> bool {
        if self.full_round {
            true
        } else if self.cross_zero {
            angle >= self.start || angle < self.end
        } else {
            angle >= self.start && angle < self.end
        }
    }
}

enum Splitter {
    Angle { split_angle: i32, prev_angle: i32 },
    Blocks { blocks: u16 },
}

impl Splitter {
    fn new(mode: SplitMode) -> Self {
        match mode {
            SplitMode::Angle(deg) => {
                let a = (deg * 100.0) as u16 as i32;
                Splitter::Angle { split_angle: a, prev_angle: a }
            }
            _ => Splitter::Blocks { blocks: 0 },
        }
    }

    fn new_block(&mut self, angle: i32, max_blocks: u16) -> bool {
        match self {
            Splitter::Angle { split_angle, prev_angle } => {
                if angle < *prev_angle {
                    *prev_angle -= ONE_ROUND;
                }
                let v = *prev_angle < *split_angle && *split_angle <= angle;
                *prev_angle = angle;
                v
            }
            Splitter::Blocks { blocks } => {
                *blocks += 1;
                if *blocks >= max_blocks {
                    *blocks = 0;
                    true
                } else {
                    false
                }
            }
        }
    }
}

/// Per-sensor decoding state. Feed it DIFOP, MSOP and IMU packets; it returns a scan each time one ends.
pub struct Decoder {
    config: DecoderConfig,
    trigon: Trigon,
    // per-beam calibration, 0.01 degree
    vert_angles: Vec<i32>,
    horiz_angles: Vec<i32>,
    user_chans: Vec<u16>,
    angles_ready: bool,
    chan_tss: [f64; 128],
    chan_azis: [f32; 128],
    block_duration: f64,
    rps: u16,
    blks_per_frame: u16,
    split_blks_per_frame: u16,
    block_az_diff: u16,
    fov_blind_ts_diff: f64,
    lens_center_rxy: f32,
    scan_section: AzimuthSection,
    distance_min: f32,
    distance_max: f32,
    splitter: Splitter,
    echo_mode: EchoMode,
    model: LidarModel,
    channel_num: u16,
    initialized: bool,
    install_mode: Option<u8>,
    transform: Option<[[f32; 4]; 3]>,
    difop_rpy_deg: [f32; 3],
    temperature_c: f32,
    first_point_ts: f64,
    prev_point_ts: f64,
    points: Vec<Point>,
    seq: u32,
    scans_split: u32,
    difop: Option<Difop>,
}

fn host_time_us() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_micros() as u64).unwrap_or(0)
}

impl Decoder {
    pub fn new(config: DecoderConfig) -> Self {
        let block_duration = BLOCK_TS_US as f64 / 1_000_000.0;
        let mut chan_tss = [0.0; 128];
        let mut chan_azis = [0.0; 128];
        for (i, &t) in FIRING_AIRY_M.iter().enumerate() {
            chan_tss[i] = t as f64 / 1_000_000.0;
            chan_azis[i] = t / BLOCK_TS_US;
        }
        let blks_per_frame = (1.0 / (10.0 * block_duration)) as u16;
        // rs_driver's DistanceSection: a user range replaces the device range when either bound is non-zero
        let (umin, umax) = (config.min_distance.max(0.0), config.max_distance.max(0.0));
        let (distance_min, distance_max) =
            if umin != 0.0 || umax != 0.0 { (umin, umax) } else { (DISTANCE_MIN, DISTANCE_MAX) };
        let mut d = Decoder {
            scan_section: AzimuthSection::new((config.start_angle * 100.0) as i32, (config.end_angle * 100.0) as i32),
            splitter: Splitter::new(config.split),
            trigon: Trigon::new(),
            vert_angles: vec![0; packet::LASER_NUM],
            horiz_angles: vec![0; packet::LASER_NUM],
            user_chans: vec![0; packet::LASER_NUM],
            angles_ready: false,
            chan_tss,
            chan_azis,
            block_duration,
            rps: 10,
            blks_per_frame,
            split_blks_per_frame: blks_per_frame,
            block_az_diff: 20,
            fov_blind_ts_diff: 0.0,
            lens_center_rxy: (LENS_RX * LENS_RX + LENS_RY * LENS_RY).sqrt(),
            distance_min,
            distance_max,
            echo_mode: EchoMode::Single,
            model: LidarModel::Channel96,
            channel_num: 96,
            initialized: false,
            install_mode: None,
            transform: None,
            difop_rpy_deg: [0.0; 3],
            temperature_c: 0.0,
            first_point_ts: 0.0,
            prev_point_ts: 0.0,
            points: Vec::new(),
            seq: 0,
            scans_split: 0,
            difop: None,
            config,
        };
        d.update_transform();
        d
    }

    pub fn config(&self) -> &DecoderConfig {
        &self.config
    }

    /// The last DIFOP decoded.
    pub fn difop(&self) -> Option<&Difop> {
        self.difop.as_ref()
    }

    pub fn angles_ready(&self) -> bool {
        self.angles_ready
    }

    pub fn rps(&self) -> u16 {
        self.rps
    }

    pub fn echo_mode(&self) -> EchoMode {
        self.echo_mode
    }

    pub fn model(&self) -> LidarModel {
        self.model
    }

    /// Main-board temperature from the last MSOP header, degrees C.
    pub fn temperature_c(&self) -> f32 {
        self.temperature_c
    }

    /// Vertical / horizontal beam angles (0.01 degree) and ring index per channel.
    pub fn beam_angles(&self) -> (&[i32], &[i32], &[u16]) {
        (&self.vert_angles, &self.horiz_angles, &self.user_chans)
    }

    /// rs_driver's `decodeDifopCommon` + `DecoderRSAIRY::decodeDifopPkt`.
    pub fn decode_difop(&mut self, bytes: &[u8]) -> Result<(), DecodeError> {
        let difop = Difop::parse(bytes)?;
        self.rps = difop.rps();
        self.blks_per_frame = (1.0 / (self.rps as f64 * self.block_duration)) as u16;
        self.block_az_diff = (ONE_ROUND as f64 * self.rps as f64 * self.block_duration).round() as u16;
        let (fs, fe) = (difop.fov_start as i32, difop.fov_end as i32);
        let fov_range = (if fs < fe { fe - fs } else { fe + ONE_ROUND - fs }) as u16;
        let fov_blind_range = (ONE_ROUND as u16).wrapping_sub(fov_range);
        self.fov_blind_ts_diff = fov_blind_range as f64 / (ONE_ROUND as f64 * self.rps as f64);
        if !self.angles_ready {
            if let Some((vert, horiz)) = difop.angles() {
                self.user_chans = gen_user_chans(&vert);
                self.vert_angles = vert;
                self.horiz_angles = horiz;
                self.angles_ready = true;
            }
        }
        if difop.install_mode != 0xFF {
            self.install_mode = Some(difop.install_mode);
        }
        self.difop_rpy_deg = [difop.roll_deg, difop.pitch_deg, difop.yaw_deg];
        self.difop = Some(difop);
        self.update_transform();
        Ok(())
    }

    /// rs_driver's `DecoderRSAIRY::decodeImuPkt`.
    pub fn decode_imu(&self, bytes: &[u8]) -> Result<ImuSample, DecodeError> {
        let pkt = ImuPacket::parse(bytes)?;
        let timestamp_us = if self.config.use_lidar_clock { pkt.timestamp_us } else { host_time_us() };
        Ok(ImuSample {
            timestamp: timestamp_us as f64 * 1e-6,
            accel_g: pkt.accel_g,
            gyro_rad_s: pkt.gyro_rad_s,
            cnt: pkt.cnt,
        })
    }

    // T * Rz(yaw) * Ry(pitch) * Rx(roll), the DIFOP angles added to the user's (rs_driver's ENABLE_TRANSFORM)
    fn update_transform(&mut self) {
        self.transform = self.config.transform.map(|t| {
            let rad = |deg: f32| (deg as f64).to_radians();
            let roll = rad(self.difop_rpy_deg[0]) + t.roll as f64;
            let pitch = rad(self.difop_rpy_deg[1]) + t.pitch as f64;
            let yaw = rad(self.difop_rpy_deg[2]) + t.yaw as f64;
            let (sr, cr) = roll.sin_cos();
            let (sp, cp) = pitch.sin_cos();
            let (sy, cy) = yaw.sin_cos();
            let r = [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ];
            let tr = [t.x, t.y, t.z];
            let mut m = [[0.0f32; 4]; 3];
            for i in 0..3 {
                for j in 0..3 {
                    m[i][j] = r[i][j] as f32;
                }
                m[i][3] = tr[i];
            }
            m
        });
    }

    fn set_firing(&mut self, table: &[f32]) {
        for (i, &t) in table.iter().enumerate() {
            self.chan_tss[i] = t as f64 / 1_000_000.0;
            self.chan_azis[i] = t / BLOCK_TS_US;
        }
    }

    // rs_driver's one-time setup at the first MSOP packet (echo mode, model, firing table)
    fn init_from_msop(&mut self, pkt: &MsopPacket) {
        self.echo_mode = if pkt.data_type()[1] == 0x03 { EchoMode::Dual } else { EchoMode::Single };
        self.split_blks_per_frame =
            if self.echo_mode == EchoMode::Dual { self.blks_per_frame << 1 } else { self.blks_per_frame };
        (self.model, self.channel_num) = match pkt.lidar_mode() {
            0x01 => (LidarModel::Channel48, 48),
            0x03 => (LidarModel::Channel192, 192),
            _ => (LidarModel::Channel96, 96),
        };
        self.block_duration = BLOCK_TS_US as f64 / 1_000_000.0;
        match (self.model, self.install_mode) {
            (LidarModel::Channel48, _) => {
                self.set_firing(&FIRING_48L);
                self.initialized = true;
            }
            (LidarModel::Channel96, Some(0x1)) => {
                self.set_firing(&FIRING_SIDE);
                self.initialized = true;
            }
            (LidarModel::Channel96, Some(0x0)) => {
                self.set_firing(&FIRING_NORMAL);
                self.initialized = true;
            }
            // Airy M keeps the default table
            (LidarModel::Channel96, Some(0x2)) => self.initialized = true,
            _ => {}
        }
    }

    /// rs_driver's `processMsopPkt` + `DecoderRSAIRY::decodeMsopPkt`. Returns the scan this packet closed, if any.
    pub fn decode_msop(&mut self, bytes: &[u8]) -> Result<Option<Scan>, DecodeError> {
        if self.config.wait_for_difop && !self.angles_ready {
            return Err(DecodeError::NoDifop);
        }
        let pkt = MsopPacket::parse(bytes)?;
        if pkt.data_type()[0] != 0 {
            return Err(DecodeError::NotPointData);
        }
        if !self.initialized {
            self.init_from_msop(&pkt);
        }
        // block pairing per (model, echo): SingleReturn / DualReturn / TwoInOne / FourInOne iterators
        let stride = match (self.model, self.echo_mode) {
            (LidarModel::Channel48, EchoMode::Single) => 1,
            (LidarModel::Channel48, EchoMode::Dual) => 2,
            (LidarModel::Channel96, EchoMode::Single) => 2,
            (LidarModel::Channel96, EchoMode::Dual) => 4,
            (LidarModel::Channel192, _) => return Err(DecodeError::UnsupportedModel),
        };
        self.intern_decode(&pkt, stride)
    }

    // block azimuth deltas and time offsets, rs_driver's BlockIterator family with `stride` blocks per firing
    fn block_timing(&self, pkt: &MsopPacket, stride: usize) -> ([i32; BLOCKS_PER_PKT], [f64; BLOCKS_PER_PKT]) {
        let mut az_diffs = [0i32; BLOCKS_PER_PKT];
        let mut tss = [0f64; BLOCKS_PER_PKT];
        let mut ts = 0.0;
        let mut blk = 0;
        while blk < BLOCKS_PER_PKT - stride {
            let mut ts_diff = self.block_duration;
            let mut az_diff = pkt.block_azimuth(blk + stride) as i32 - pkt.block_azimuth(blk) as i32;
            if az_diff < 0 {
                az_diff += ONE_ROUND;
            }
            // skip the FOV blind zone
            if az_diff > 100 {
                az_diff = self.block_az_diff as i32;
                ts_diff = self.fov_blind_ts_diff;
            }
            for k in 0..stride {
                az_diffs[blk + k] = az_diff;
                tss[blk + k] = ts;
            }
            ts += ts_diff;
            blk += stride;
        }
        for k in 0..stride {
            az_diffs[blk + k] = self.block_az_diff as i32;
            tss[blk + k] = ts;
        }
        (az_diffs, tss)
    }

    fn intern_decode(&mut self, pkt: &MsopPacket, stride: usize) -> Result<Option<Scan>, DecodeError> {
        self.temperature_c = pkt.temperature_c();
        let packet_duration = self.block_duration * BLOCKS_PER_PKT as f64;
        let pkt_ts = if self.config.use_lidar_clock {
            pkt.timestamp_us() as f64 * 1e-6
        } else {
            // roll back to the first block to approach the lidar's time
            host_time_us() as f64 * 1e-6 - packet_duration
        };
        let (az_diffs, tss) = self.block_timing(pkt, stride);
        let mut closed = None;
        let mut result = Ok(());
        for blk in 0..BLOCKS_PER_PKT {
            if pkt.block_id(blk) != BLOCK_ID {
                result = Err(DecodeError::WrongBlockId);
                break;
            }
            let block_az_diff = az_diffs[blk];
            let block_ts = pkt_ts + tss[blk];
            let block_az = pkt.block_azimuth(blk) as i32;
            let max_blocks = match self.config.split {
                SplitMode::CustomBlocks(n) => n,
                _ => self.split_blks_per_frame,
            };
            if self.splitter.new_block(block_az, max_blocks) {
                closed = self.split_frame().or(closed);
                self.first_point_ts = block_ts;
            }
            for chan in 0..CHANNELS_PER_BLOCK {
                let channel = pkt.channel(blk, chan);
                let chan_id = if self.model == LidarModel::Channel96 && blk % 2 == 1 { chan + 48 } else { chan };
                let chan_ts = block_ts + self.chan_tss[chan_id];
                let angle_horiz = block_az + (block_az_diff as f32 * self.chan_azis[chan_id]) as i32;
                let angle_vert = self.vert_angles[chan_id];
                let angle_horiz_final = angle_horiz + (self.horiz_angles[chan_id] as f64).round() as i32;
                let distance_raw = channel.raw_distance & 0x3FFF;
                let feature = ((channel.raw_distance >> 14) & 0x03) as u8;
                let distance = distance_raw as f32 * DISTANCE_RES;
                let ring = self.user_chans[chan_id];
                if distance >= self.distance_min
                    && distance <= self.distance_max
                    && self.scan_section.contains(angle_horiz_final)
                {
                    let t = &self.trigon;
                    let cos_v = t.cos(angle_vert);
                    let mut x =
                        distance * cos_v * t.cos(angle_horiz_final) + self.lens_center_rxy * t.cos(angle_horiz);
                    let mut y =
                        -distance * cos_v * t.sin(angle_horiz_final) - self.lens_center_rxy * t.sin(angle_horiz);
                    let mut z = distance * t.sin(angle_vert) + LENS_RZ;
                    if let Some(m) = &self.transform {
                        let (px, py, pz) = (x, y, z);
                        x = m[0][0] * px + m[0][1] * py + m[0][2] * pz + m[0][3];
                        y = m[1][0] * px + m[1][1] * py + m[1][2] * pz + m[1][3];
                        z = m[2][0] * px + m[2][1] * py + m[2][2] * pz + m[2][3];
                    }
                    self.points.push(Point { x, y, z, intensity: channel.intensity, ring, time: chan_ts, feature });
                } else if !self.config.dense_points {
                    self.points.push(Point {
                        x: f32::NAN,
                        y: f32::NAN,
                        z: f32::NAN,
                        intensity: 0,
                        ring,
                        time: chan_ts,
                        feature,
                    });
                }
                self.prev_point_ts = chan_ts;
            }
        }
        // a bad block ends the packet early (as rs_driver's `break`); a scan it already closed is still returned
        match (closed, result) {
            (Some(scan), _) => Ok(Some(scan)),
            (None, r) => r.map(|_| None),
        }
    }

    // rs_driver's cb_split_frame_ (LidarDriverImpl::splitFrame): emit the cloud if it has points
    fn split_frame(&mut self) -> Option<Scan> {
        let first = self.scans_split == 0;
        self.scans_split += 1;
        if self.points.is_empty() {
            return None;
        }
        let timestamp = if self.config.ts_first_point { self.first_point_ts } else { self.prev_point_ts };
        let scan = Scan {
            seq: self.seq,
            timestamp,
            rings: self.channel_num,
            points: std::mem::take(&mut self.points),
            complete: !first,
        };
        self.seq += 1;
        Some(scan)
    }
}

// rs_driver's `genUserChan`: each beam's rank by vertical angle
fn gen_user_chans(vert: &[i32]) -> Vec<u16> {
    vert.iter().map(|&a| vert.iter().filter(|&&b| b < a).count() as u16).collect()
}
