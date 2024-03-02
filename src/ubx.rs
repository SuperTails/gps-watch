use bytemuck::{Pod, Zeroable};
use tinyvec::ArrayVec;

const UBX_BUF_SIZE: usize = 256;
type UbxBuf = ArrayVec<[u8; UBX_BUF_SIZE]>;

#[derive(Copy, Clone)]
pub struct UbxChecksum(pub u8, pub u8);
impl UbxChecksum {
    pub fn new() -> Self {
        Self(0, 0)
    }

    pub fn next(self, byte: u8) -> Self {
        let Self(a, b) = self;
        Self(a.wrapping_add(byte), b.wrapping_add(a).wrapping_add(byte))
    }
}

impl PartialEq<(u8, u8)> for UbxChecksum {
    fn eq(&self, (other_a, other_b): &(u8, u8)) -> bool {
        let Self(a, b) = self;
        a == other_a && b == other_b
    }
}

#[derive(defmt::Format, Debug, Copy, Clone)]
pub enum UbxError {
    BadChecksum { expect: (u8, u8), saw: (u8, u8) },
    BadStart { expect: u8, saw: u8 },
    BadPayload,
    TooLarge(u16),
}
use UbxError::*;

#[derive(Copy, Clone)]
enum ParserState {
    Start,
    Sync1,
    Sync2,
    Class {
        class: u8,
        checksum: UbxChecksum,
    },
    Id {
        class: u8,
        id: u8,
        checksum: UbxChecksum,
    },
    Len1 {
        class: u8,
        id: u8,
        len1: u8,
        checksum: UbxChecksum,
    },
    Len2 {
        class: u8,
        id: u8,
        len: u16,
        checksum: UbxChecksum,
    },
    Payload {
        class: u8,
        id: u8,
        len: u16,
        checksum: UbxChecksum,
    },
    Checksum1 {
        class: u8,
        id: u8,
        expect: UbxChecksum,
        found: u8,
    },
}
use ParserState::*;

pub struct UbxParser {
    state: ParserState,
    buf: UbxBuf,
}

impl UbxParser {
    pub fn new() -> Self {
        Self {
            state: Start,
            buf: UbxBuf::new(),
        }
    }

    fn feed(&mut self, b: u8) -> Option<Result<(u8, u8), UbxError>> {
        match self.state {
            Start => {
                if b == 0xb5 {
                    self.state = Sync1;
                    None
                } else {
                    self.state = Start;
                    // Some(Err(BadStart {expect: 0xb5, saw: b}))
                    None
                }
            }
            Sync1 => {
                if b == 0x62 {
                    self.state = Sync2;
                    None
                } else if b == 0xb5 {
                    None
                } else {
                    self.state = Start;
                    Some(Err(BadStart {
                        expect: 0x62,
                        saw: b,
                    }))
                }
            }
            Sync2 => {
                self.state = Class {
                    class: b,
                    checksum: UbxChecksum::new().next(b),
                };
                None
            }
            Class { class, checksum } => {
                self.state = Id {
                    class,
                    id: b,
                    checksum: checksum.next(b),
                };
                None
            }
            Id {
                class,
                id,
                checksum,
            } => {
                self.state = Len1 {
                    class,
                    id,
                    len1: b,
                    checksum: checksum.next(b),
                };
                None
            }
            Len1 {
                class,
                id,
                len1,
                checksum,
            } => {
                let len = (b as u16) << 8 | (len1 as u16);
                if len as usize > UBX_BUF_SIZE {
                    self.state = Start;
                    Some(Err(TooLarge(len)))
                } else {
                    self.state = Len2 {
                        class,
                        id,
                        len,
                        checksum: checksum.next(b),
                    };
                    None
                }
            }
            Len2 {
                class,
                id,
                len,
                checksum,
            } => {
                if len > 0 {
                    self.buf.clear();
                    let _ = self.buf.try_push(b);
                    self.state = Payload {
                        class,
                        id,
                        len,
                        checksum: checksum.next(b),
                    };
                    None
                } else {
                    self.buf.clear();
                    self.state = Checksum1 {
                        class,
                        id,
                        expect: checksum,
                        found: b,
                    };
                    None
                }
            }
            Payload {
                class,
                id,
                len,
                checksum,
            } => {
                if self.buf.len() == len as usize {
                    self.state = Checksum1 {
                        class,
                        id,
                        expect: checksum,
                        found: b,
                    };
                    None
                } else {
                    let _ = self.buf.try_push(b);
                    self.state = Payload {
                        class,
                        id,
                        len,
                        checksum: checksum.next(b),
                    };
                    None
                }
            }
            Checksum1 {
                class,
                id,
                expect,
                found,
            } => {
                if expect == (found, b) {
                    self.state = Start;
                    Some(Ok((class, id)))
                } else {
                    self.state = Start;
                    Some(Err(BadChecksum {
                        expect: (expect.0, expect.1),
                        saw: (found, b),
                    }))
                }
            }
        }
    }

    pub fn process_byte(&mut self, b: u8) -> Option<Result<UbxPacket, UbxError>> {
        self.feed(b).map(|r| {
            r.and_then(|(class, id)| match (class, id) {
                (0x05, 0x01) => Ok(UbxPacket::AckAck {
                    class: self.buf[0],
                    id: self.buf[1],
                }),
                (0x05, 0x00) => Ok(UbxPacket::AckNak {
                    class: self.buf[0],
                    id: self.buf[1],
                }),
                (0x01, 0x07) => bytemuck::try_pod_read_unaligned(&self.buf)
                    .map(|x| UbxPacket::NavPvt(x))
                    .map_err(|_| BadPayload),
                _ => Ok(UbxPacket::OtherPacket),
            })
        })
    }
}

#[derive(Debug, Copy, Clone)]
pub enum UbxPacket {
    AckAck { class: u8, id: u8 },
    AckNak { class: u8, id: u8 },
    NavPvt(NavPvt),
    InfNotice(UbxBuf),
    InfError(UbxBuf),

    OtherPacket,
}

// SAFETY: All fields are naturally aligned, so there is no padding.
// Also, this device has the same endianness as UBX (little)
#[repr(C)]
#[derive(Pod, Zeroable, Copy, Clone, Debug)]
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
}
