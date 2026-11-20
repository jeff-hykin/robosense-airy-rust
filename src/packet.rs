// Ported from RoboSense's rs_driver (BSD-3-Clause), see LICENSE-THIRD-PARTY.
//! Wire layouts of the Airy's three UDP packets, after rs_driver's `decoder_RSAIRY.hpp` structs (packed, big endian).

use std::fmt;

pub const MSOP_LEN: usize = 1248;
pub const DIFOP_LEN: usize = 1248;
pub const IMU_LEN: usize = 51;
pub const MSOP_ID: [u8; 4] = [0x55, 0xAA, 0x05, 0x5A];
pub const DIFOP_ID: [u8; 8] = [0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55];
pub const IMU_ID: [u8; 4] = [0xAA, 0x55, 0x5A, 0x05];
pub const BLOCK_ID: [u8; 2] = [0xFF, 0xEE];
pub const BLOCKS_PER_PKT: usize = 8;
pub const CHANNELS_PER_BLOCK: usize = 48;
/// Calibration rows in a DIFOP packet (the 96-beam model's beam count).
pub const LASER_NUM: usize = 96;

// RSAIRYHeader field offsets
const HDR_DATA_TYPE: usize = 16;
const HDR_TIMESTAMP: usize = 20;
const HDR_LIDAR_TYPE: usize = 31;
const HDR_LIDAR_MODE: usize = 32;
const HDR_TEMP: usize = 38;
const HDR_TOPBOARD_TEMP: usize = 40;
const HDR_LEN: usize = 42;
// RSAIRYMsopBlock: id[2], azimuth u16, 48 x (distance u16, intensity u8)
const BLOCK_LEN: usize = 4 + CHANNELS_PER_BLOCK * 3;

// RSAIRYDifopPkt field offsets
const DIFOP_RPM: usize = 8;
const DIFOP_ETH: usize = 10;
const DIFOP_FOV: usize = 32;
const DIFOP_VERSION: usize = 40;
const DIFOP_INSTALL_MODE: usize = 289;
const DIFOP_SN: usize = 292;
const DIFOP_ZERO_CAL: usize = 298;
const DIFOP_RETURN_MODE: usize = 300;
const DIFOP_VERT_CALI: usize = 468;
const DIFOP_HORIZ_CALI: usize = 756;
const DIFOP_STATUS: usize = 1066;
const DIFOP_QUAT: usize = 1092;
const DIFOP_XYZ: usize = 1108;
const DIFOP_PTP_ERR: usize = 1120;
const DIFOP_PITCH: usize = 1124;
const DIFOP_YAW: usize = 1128;
const DIFOP_ROLL: usize = 1132;

// RSAIRYImuPkt field offsets
const IMU_TIMESTAMP: usize = 4;
const IMU_ACCEL: usize = 14;
const IMU_GYRO: usize = 26;
const IMU_TEMPERATURE: usize = 38;
const IMU_ODR: usize = 42;
const IMU_ACCEL_FSR: usize = 43;
const IMU_GYRO_FSR: usize = 44;
const IMU_CNT: usize = 45;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PacketError {
    WrongLength { expected: usize, got: usize },
    WrongId,
}

impl fmt::Display for PacketError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PacketError::WrongLength { expected, got } => write!(f, "packet is {got} bytes, expected {expected}"),
            PacketError::WrongId => write!(f, "packet id does not match"),
        }
    }
}

impl std::error::Error for PacketError {}

fn be16(b: &[u8], at: usize) -> u16 {
    u16::from_be_bytes([b[at], b[at + 1]])
}

fn be32(b: &[u8], at: usize) -> u32 {
    u32::from_be_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
}

fn be_f32(b: &[u8], at: usize) -> f32 {
    f32::from_bits(be32(b, at))
}

/// rs_driver's `parseTimeUTCWithUs`: 6 bytes of seconds then 4 bytes of microseconds, both big endian.
pub fn parse_time_utc_us(b: &[u8]) -> u64 {
    let sec = b[..6].iter().fold(0u64, |acc, &v| (acc << 8) + v as u64);
    let us = b[6..10].iter().fold(0u64, |acc, &v| (acc << 8) + v as u64);
    sec * 1_000_000 + us
}

/// rs_driver's `parseTempInLe`: | lsb 5 | padding 3 | neg 1 | msb 7 | (bits), in units of 0.0625 C.
pub fn parse_temp_le(b: &[u8]) -> i16 {
    let lsb = b[0] >> 3;
    let neg = b[1] & 0x80;
    let msb = b[1] & 0x7F;
    let t = ((msb as i16) << 5) + lsb as i16;
    if neg != 0 {
        -t
    } else {
        t
    }
}

/// rs_driver's TEMPERATURE_RES for the Airy (degrees C per count).
pub const TEMPERATURE_RES: f32 = 0.0625;

/// One MSOP (point data) packet, borrowed.
#[derive(Clone, Copy)]
pub struct MsopPacket<'a> {
    pub bytes: &'a [u8],
}

#[derive(Debug, Clone, Copy)]
pub struct Channel {
    /// Raw 16-bit field: distance in the low 14 bits, "feature" in the top 2.
    pub raw_distance: u16,
    pub intensity: u8,
}

impl<'a> MsopPacket<'a> {
    pub fn parse(bytes: &'a [u8]) -> Result<Self, PacketError> {
        if bytes.len() != MSOP_LEN {
            return Err(PacketError::WrongLength { expected: MSOP_LEN, got: bytes.len() });
        }
        if bytes[..4] != MSOP_ID {
            return Err(PacketError::WrongId);
        }
        Ok(MsopPacket { bytes })
    }

    pub fn data_type(&self) -> [u8; 2] {
        [self.bytes[HDR_DATA_TYPE], self.bytes[HDR_DATA_TYPE + 1]]
    }

    pub fn timestamp_us(&self) -> u64 {
        parse_time_utc_us(&self.bytes[HDR_TIMESTAMP..HDR_TIMESTAMP + 10])
    }

    pub fn lidar_type(&self) -> u8 {
        self.bytes[HDR_LIDAR_TYPE]
    }

    pub fn lidar_mode(&self) -> u8 {
        self.bytes[HDR_LIDAR_MODE]
    }

    pub fn temperature_c(&self) -> f32 {
        parse_temp_le(&self.bytes[HDR_TEMP..HDR_TEMP + 2]) as f32 * TEMPERATURE_RES
    }

    pub fn topboard_temperature_c(&self) -> f32 {
        parse_temp_le(&self.bytes[HDR_TOPBOARD_TEMP..HDR_TOPBOARD_TEMP + 2]) as f32 * TEMPERATURE_RES
    }

    fn block(&self, blk: usize) -> &'a [u8] {
        let at = HDR_LEN + blk * BLOCK_LEN;
        &self.bytes[at..at + BLOCK_LEN]
    }

    pub fn block_id(&self, blk: usize) -> [u8; 2] {
        let b = self.block(blk);
        [b[0], b[1]]
    }

    /// Block azimuth in 0.01 degree.
    pub fn block_azimuth(&self, blk: usize) -> u16 {
        be16(self.block(blk), 2)
    }

    pub fn channel(&self, blk: usize, chan: usize) -> Channel {
        let b = self.block(blk);
        let at = 4 + chan * 3;
        Channel { raw_distance: be16(b, at), intensity: b[at + 2] }
    }
}

/// One calibration row: sign byte (non-zero = negative, 0xFF = unset) and magnitude in 0.01 degree.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CalibrationAngle {
    pub sign: u8,
    pub value: u16,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Difop {
    pub rpm: u16,
    pub lidar_ip: [u8; 4],
    pub dest_ip: [u8; 4],
    pub mac: [u8; 6],
    pub msop_port: u16,
    pub difop_port: u16,
    /// FOV start/end in 0.01 degree.
    pub fov_start: u16,
    pub fov_end: u16,
    pub top_version: [u8; 5],
    pub bottom_version: [u8; 5],
    /// 0 normal, 1 side, 2 "Airy M" (selects the firing time table), 0xFF unset.
    pub install_mode: u8,
    pub serial: [u8; 6],
    pub zero_cal: u16,
    pub return_mode: u8,
    pub vert_angle_cali: Vec<CalibrationAngle>,
    pub horiz_angle_cali: Vec<CalibrationAngle>,
    pub vol_main: u16,
    pub vol_12v: u16,
    pub vol_mcu: u16,
    pub vol_1v: u16,
    pub current_main: u16,
    /// Lidar-to-IMU extrinsic as the device reports it (rs_driver's device_info qx..qw, x, y, z).
    pub imu_quat_xyzw: [f32; 4],
    pub imu_xyz: [f32; 3],
    pub ptp_err: [u8; 4],
    /// Mount angles in degrees, added to the user transform when rs_driver's ENABLE_TRANSFORM is on.
    pub pitch_deg: f32,
    pub yaw_deg: f32,
    pub roll_deg: f32,
}

impl Difop {
    pub fn parse(b: &[u8]) -> Result<Self, PacketError> {
        if b.len() != DIFOP_LEN {
            return Err(PacketError::WrongLength { expected: DIFOP_LEN, got: b.len() });
        }
        if b[..8] != DIFOP_ID {
            return Err(PacketError::WrongId);
        }
        let arr = |at: usize, n: usize| b[at..at + n].to_vec();
        let cali = |at: usize| {
            (0..LASER_NUM)
                .map(|i| CalibrationAngle { sign: b[at + i * 3], value: be16(b, at + i * 3 + 1) })
                .collect::<Vec<_>>()
        };
        Ok(Difop {
            rpm: be16(b, DIFOP_RPM),
            lidar_ip: arr(DIFOP_ETH, 4).try_into().unwrap(),
            dest_ip: arr(DIFOP_ETH + 4, 4).try_into().unwrap(),
            mac: arr(DIFOP_ETH + 8, 6).try_into().unwrap(),
            msop_port: be16(b, DIFOP_ETH + 14),
            difop_port: be16(b, DIFOP_ETH + 18),
            fov_start: be16(b, DIFOP_FOV),
            fov_end: be16(b, DIFOP_FOV + 2),
            top_version: arr(DIFOP_VERSION, 5).try_into().unwrap(),
            bottom_version: arr(DIFOP_VERSION + 5, 5).try_into().unwrap(),
            install_mode: b[DIFOP_INSTALL_MODE],
            serial: arr(DIFOP_SN, 6).try_into().unwrap(),
            zero_cal: be16(b, DIFOP_ZERO_CAL),
            return_mode: b[DIFOP_RETURN_MODE],
            vert_angle_cali: cali(DIFOP_VERT_CALI),
            horiz_angle_cali: cali(DIFOP_HORIZ_CALI),
            vol_main: be16(b, DIFOP_STATUS),
            vol_12v: be16(b, DIFOP_STATUS + 2),
            vol_mcu: be16(b, DIFOP_STATUS + 4),
            vol_1v: be16(b, DIFOP_STATUS + 6),
            current_main: be16(b, DIFOP_STATUS + 8),
            imu_quat_xyzw: [
                be_f32(b, DIFOP_QUAT),
                be_f32(b, DIFOP_QUAT + 4),
                be_f32(b, DIFOP_QUAT + 8),
                be_f32(b, DIFOP_QUAT + 12),
            ],
            imu_xyz: [be_f32(b, DIFOP_XYZ), be_f32(b, DIFOP_XYZ + 4), be_f32(b, DIFOP_XYZ + 8)],
            ptp_err: arr(DIFOP_PTP_ERR, 4).try_into().unwrap(),
            pitch_deg: be_f32(b, DIFOP_PITCH),
            yaw_deg: be_f32(b, DIFOP_YAW),
            roll_deg: be_f32(b, DIFOP_ROLL),
        })
    }

    /// Rounds per second, as rs_driver derives it (rpm / 60, 10 when the device reports 0).
    pub fn rps(&self) -> u16 {
        match self.rpm / 60 {
            0 => 10,
            rps => rps,
        }
    }

    /// rs_driver's `ChanAngles::loadFromDifop`: signed angles in 0.01 degree, None if any row is unset or out of range.
    pub fn angles(&self) -> Option<(Vec<i32>, Vec<i32>)> {
        let check = |v: i32| (-9000..18000).contains(&v);
        let mut vert = Vec::with_capacity(LASER_NUM);
        let mut horiz = Vec::with_capacity(LASER_NUM);
        for (v, h) in self.vert_angle_cali.iter().zip(&self.horiz_angle_cali) {
            if v.sign == 0xFF {
                return None;
            }
            let signed = |a: &CalibrationAngle| if a.sign != 0 { -(a.value as i32) } else { a.value as i32 };
            let (vv, hh) = (signed(v), signed(h));
            if !check(vv) || !check(hh) {
                return None;
            }
            vert.push(vv);
            horiz.push(hh);
        }
        Some((vert, horiz))
    }
}

/// One IMU packet, decoded the way rs_driver's `DecoderRSAIRY::decodeImuPkt` does.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuPacket {
    pub timestamp_us: u64,
    /// Linear acceleration in g (rs_driver's unit: full scale range / 32768).
    pub accel_g: [f32; 3],
    /// Angular velocity in rad/s.
    pub gyro_rad_s: [f32; 3],
    pub raw_temperature: i32,
    pub odr: u8,
    pub accel_fsr: u8,
    pub gyro_fsr: u8,
    pub cnt: u32,
}

/// rs_driver's FSR_BASE.
const FSR_BASE: f32 = 32768.0;

impl ImuPacket {
    pub fn parse(b: &[u8]) -> Result<Self, PacketError> {
        if b.len() != IMU_LEN {
            return Err(PacketError::WrongLength { expected: IMU_LEN, got: b.len() });
        }
        if b[..4] != IMU_ID {
            return Err(PacketError::WrongId);
        }
        let accel_fsr = b[IMU_ACCEL_FSR];
        let gyro_fsr = b[IMU_GYRO_FSR];
        // the C++ computes these in uint8_t / uint16_t
        let accel_unit = ((1u32 << (accel_fsr as u32 + 1)) as u8) as f32 / FSR_BASE;
        let gyro_unit = ((250u32 * (1u32 << gyro_fsr as u32)) as u16) as f32 / FSR_BASE * std::f32::consts::PI / 180.0;
        let i32_at = |at: usize| be32(b, at) as i32;
        Ok(ImuPacket {
            timestamp_us: parse_time_utc_us(&b[IMU_TIMESTAMP..IMU_TIMESTAMP + 10]),
            accel_g: [
                i32_at(IMU_ACCEL) as f32 * accel_unit,
                i32_at(IMU_ACCEL + 4) as f32 * accel_unit,
                i32_at(IMU_ACCEL + 8) as f32 * accel_unit,
            ],
            gyro_rad_s: [
                i32_at(IMU_GYRO) as f32 * gyro_unit,
                i32_at(IMU_GYRO + 4) as f32 * gyro_unit,
                i32_at(IMU_GYRO + 8) as f32 * gyro_unit,
            ],
            raw_temperature: be32(b, IMU_TEMPERATURE) as i32,
            odr: b[IMU_ODR],
            accel_fsr,
            gyro_fsr,
            cnt: u32::from_le_bytes(b[IMU_CNT..IMU_CNT + 4].try_into().unwrap()),
        })
    }
}

/// What a datagram is, by its id bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PacketKind {
    Msop,
    Difop,
    Imu,
}

pub fn classify(b: &[u8]) -> Option<PacketKind> {
    if b.len() == MSOP_LEN && b[..4] == MSOP_ID {
        Some(PacketKind::Msop)
    } else if b.len() == DIFOP_LEN && b[..8] == DIFOP_ID {
        Some(PacketKind::Difop)
    } else if b.len() == IMU_LEN && b[..4] == IMU_ID {
        Some(PacketKind::Imu)
    } else {
        None
    }
}
