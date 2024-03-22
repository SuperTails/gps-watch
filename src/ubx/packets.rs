use core::iter::once;

use crate::Position;
use bytemuck::{Pod, Zeroable};
use chrono::{DateTime, NaiveDate, NaiveDateTime, NaiveTime, Utc};

use super::{cfg::CfgItem, generator::SendablePacket};

////////////////////////////////////////////////////////////////////////////////
// Packets which can be parsed /////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// SAFETY: All fields are naturally aligned, so there is no padding.
// Also, this device has the same endianness as UBX (little)
#[repr(C)]
#[derive(defmt::Format, Pod, Zeroable, Copy, Clone, Debug)]
pub struct NavPvt {
    pub i_tow: u32,
    pub year: u16,
    pub month: u8,
    pub day: u8,

    pub hour: u8,
    pub min: u8,
    pub sec: u8,
    pub valid: u8,

    pub t_acc: u32,
    pub nano: i32,

    pub fix_type: u8,
    pub flags: u8,
    pub flags2: u8,
    pub sum_sv: u8,

    pub lon: i32,
    pub lat: i32,
    pub height: i32,
    pub h_msl: i32,
    pub h_acc: u32,
    pub v_acc: u32,
    pub vel_n: i32,
    pub vel_e: i32,
    pub vel_d: i32,
    pub g_speed: i32,
    pub head_mot: i32,
    pub s_acc: u32,
    pub head_acc: u32,

    pub p_dop: u16,
    pub flags3: u16,

    pub reserved0_a: [u8; 4],
    pub head_veh: i32,

    pub mag_dec: i16,
    pub mag_acc: u16,
}

impl NavPvt {
    pub fn lat_degrees(&self) -> f32 {
        (self.lat as f32) * 10e-8
    }

    pub fn lon_degrees(&self) -> f32 {
        (self.lon as f32) * 10e-8
    }

    pub fn position(&self) -> Position {
        Position {
            lat: self.lat_degrees(),
            lon: self.lon_degrees(),
        }
    }

    pub fn datetime(&self) -> DateTime<Utc> {
        DateTime::from_naive_utc_and_offset(
            NaiveDateTime::new(
                NaiveDate::from_ymd_opt(self.year as i32, self.month as u32, self.day as u32)
                    .unwrap(),
                NaiveTime::from_hms_opt(
                    self.hour as u32,
                    self.min as u32,
                    self.sec as u32,
                )
                .unwrap(),
            ),
            Utc,
        )
    }
}

////////////////////////////////////////////////////////////////////////////////
// Packets which can be generated //////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

pub struct CfgValSet<const N: usize> {
    pub ram: bool,
    pub bbr: bool,
    pub flash: bool,
    pub items: [CfgItem; N],
}

impl<const N: usize> CfgValSet<N> {
    pub fn layers(&self) -> u8 {
        (if self.ram { 1 << 0 } else { 0 })
            | (if self.bbr { 1 << 1 } else { 0 })
            | (if self.flash { 1 << 2 } else { 0 })
    }
}

impl<const N: usize> SendablePacket for CfgValSet<N> {
    type I = impl Iterator<Item = u8>;

    fn class(&self) -> u8 {
        0x06
    }

    fn id(&self) -> u8 {
        0x8a
    }

    fn payload_len(&self) -> usize {
        4 + self.items.iter().map(|i| i.packed_size()).sum::<usize>()
    }

    fn payload_bytes(self) -> Self::I {
        [0x01, self.layers(), 0x00, 0x00]
            .into_iter()
            .chain(self.items.into_iter().flat_map(|i| i.to_bytes()))
    }
}

pub struct CfgCfg {
    pub clear_mask: u32,
    pub save_mask: u32,
    pub load_mask: u32,
    pub dev_bbr: bool,
    pub dev_flash: bool,
    pub dev_eeprom: bool,
    pub dev_spi_flash: bool,
}

impl CfgCfg {
    pub fn device_mask(&self) -> u8 {
        (if self.dev_bbr { 1 << 0 } else { 0 })
            | (if self.dev_flash { 1 << 1 } else { 0 })
            | (if self.dev_eeprom { 1 << 2 } else { 0 })
            | (if self.dev_spi_flash { 1 << 3 } else { 0 })
    }
}

impl SendablePacket for CfgCfg {
    type I = impl Iterator<Item = u8>;

    fn class(&self) -> u8 {
        0x06
    }

    fn id(&self) -> u8 {
        0x09
    }

    fn payload_len(&self) -> usize {
        13
    }

    fn payload_bytes(self) -> Self::I {
        [self.clear_mask, self.save_mask, self.load_mask]
            .into_iter()
            .flat_map(|i| i.to_le_bytes().into_iter())
            .chain(once(self.device_mask()))
    }
}

pub struct CfgRst {
    pub nav_bbr_mask: u16,
    pub reset_mode: u8,
}

impl SendablePacket for CfgRst {
    type I = impl Iterator<Item = u8>;

    fn class(&self) -> u8 {
        0x06
    }

    fn id(&self) -> u8 {
        0x04
    }

    fn payload_len(&self) -> usize {
        4
    }

    fn payload_bytes(self) -> Self::I {
        self.nav_bbr_mask
            .to_le_bytes()
            .into_iter()
            .chain(once(self.reset_mode))
            .chain(once(0))
    }
}
