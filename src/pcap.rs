//! Minimal classic-pcap reader for UDP/IPv4 captures (tcpdump `-w`), for tests and replay.

use std::fmt;
use std::io::Read;
use std::path::Path;

#[derive(Debug, Clone, PartialEq)]
pub struct UdpDatagram {
    /// Capture time, seconds since the epoch.
    pub time: f64,
    pub src: [u8; 4],
    pub dst: [u8; 4],
    pub src_port: u16,
    pub dst_port: u16,
    pub payload: Vec<u8>,
}

#[derive(Debug)]
pub enum PcapError {
    Io(std::io::Error),
    NotPcap,
    UnsupportedLinkType(u32),
}

impl fmt::Display for PcapError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PcapError::Io(e) => write!(f, "{e}"),
            PcapError::NotPcap => write!(f, "not a classic pcap file (pcapng is not supported)"),
            PcapError::UnsupportedLinkType(t) => write!(f, "unsupported pcap link type {t}"),
        }
    }
}

impl std::error::Error for PcapError {}

impl From<std::io::Error> for PcapError {
    fn from(e: std::io::Error) -> Self {
        PcapError::Io(e)
    }
}

const LINKTYPE_ETHERNET: u32 = 1;
const LINKTYPE_RAW: u32 = 101;
const LINKTYPE_LINUX_SLL: u32 = 113;
const LINKTYPE_LINUX_SLL2: u32 = 276;
const ETHERTYPE_IPV4: u16 = 0x0800;
const ETHERTYPE_VLAN: u16 = 0x8100;
const IPPROTO_UDP: u8 = 17;

/// Reads every UDP/IPv4 datagram (unfragmented) in a pcap file, in capture order.
pub fn read_udp(path: impl AsRef<Path>) -> Result<Vec<UdpDatagram>, PcapError> {
    let mut bytes = Vec::new();
    std::fs::File::open(path)?.read_to_end(&mut bytes)?;
    parse_udp(&bytes)
}

pub fn parse_udp(bytes: &[u8]) -> Result<Vec<UdpDatagram>, PcapError> {
    if bytes.len() < 24 {
        return Err(PcapError::NotPcap);
    }
    let magic = u32::from_le_bytes(bytes[0..4].try_into().unwrap());
    let (le, nanos) = match magic {
        0xa1b2c3d4 => (true, false),
        0xa1b23c4d => (true, true),
        0xd4c3b2a1 => (false, false),
        0x4d3cb2a1 => (false, true),
        _ => return Err(PcapError::NotPcap),
    };
    let u32_at = |at: usize| {
        let b: [u8; 4] = bytes[at..at + 4].try_into().unwrap();
        if le {
            u32::from_le_bytes(b)
        } else {
            u32::from_be_bytes(b)
        }
    };
    let linktype = u32_at(20) & 0x0FFF_FFFF;
    if ![LINKTYPE_ETHERNET, LINKTYPE_RAW, LINKTYPE_LINUX_SLL, LINKTYPE_LINUX_SLL2].contains(&linktype) {
        return Err(PcapError::UnsupportedLinkType(linktype));
    }
    let mut out = Vec::new();
    let mut at = 24;
    while at + 16 <= bytes.len() {
        let sec = u32_at(at) as f64;
        let frac = u32_at(at + 4) as f64;
        let incl = u32_at(at + 8) as usize;
        let frame_start = at + 16;
        at = frame_start + incl;
        if at > bytes.len() {
            break;
        }
        let time = sec + frac * if nanos { 1e-9 } else { 1e-6 };
        if let Some(d) = udp_of_frame(&bytes[frame_start..at], linktype, time) {
            out.push(d);
        }
    }
    Ok(out)
}

fn be16(b: &[u8], at: usize) -> u16 {
    u16::from_be_bytes([b[at], b[at + 1]])
}

fn udp_of_frame(frame: &[u8], linktype: u32, time: f64) -> Option<UdpDatagram> {
    let (mut ethertype, mut ip_at) = match linktype {
        LINKTYPE_ETHERNET if frame.len() >= 14 => (be16(frame, 12), 14),
        LINKTYPE_LINUX_SLL if frame.len() >= 16 => (be16(frame, 14), 16),
        LINKTYPE_LINUX_SLL2 if frame.len() >= 20 => (be16(frame, 0), 20),
        LINKTYPE_RAW => (ETHERTYPE_IPV4, 0),
        _ => return None,
    };
    while ethertype == ETHERTYPE_VLAN && frame.len() >= ip_at + 4 {
        ethertype = be16(frame, ip_at + 2);
        ip_at += 4;
    }
    if ethertype != ETHERTYPE_IPV4 || frame.len() < ip_at + 20 {
        return None;
    }
    let ip = &frame[ip_at..];
    let ihl = (ip[0] & 0x0F) as usize * 4;
    let total = be16(ip, 2) as usize;
    let fragmented = be16(ip, 6) & 0x3FFF != 0;
    if ip[0] >> 4 != 4 || ip[9] != IPPROTO_UDP || fragmented || ip.len() < total || total < ihl + 8 {
        return None;
    }
    let udp = &ip[ihl..total];
    let udp_len = (be16(udp, 4) as usize).min(udp.len());
    Some(UdpDatagram {
        time,
        src: ip[12..16].try_into().unwrap(),
        dst: ip[16..20].try_into().unwrap(),
        src_port: be16(udp, 0),
        dst_port: be16(udp, 2),
        payload: udp[8..udp_len].to_vec(),
    })
}
