use super::{packets::NavPvt, UbxBuf, UbxChecksum, UbxError, UBX_BUF_SIZE};

// States are named for the portion of the packet which was *last received*
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
                    None
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
                    Some(Err(UbxError::TooLarge(len)))
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
                    Some(Err(UbxError::BadChecksum {
                        expect: (expect.0, expect.1),
                        saw: (found, b),
                    }))
                }
            }
        }
    }

    pub fn process_byte(&mut self, b: u8) -> Option<Result<ParsedPacket, UbxError>> {
        self.feed(b).map(|r| {
            r.and_then(|(class, id)| match (class, id) {
                (0x05, 0x01) => Ok(ParsedPacket::AckAck {
                    class: self.buf[0],
                    id: self.buf[1],
                }),
                (0x05, 0x00) => Ok(ParsedPacket::AckNak {
                    class: self.buf[0],
                    id: self.buf[1],
                }),
                (0x01, 0x07) => bytemuck::try_pod_read_unaligned(&self.buf)
                    .map(|x| ParsedPacket::NavPvt(x))
                    .map_err(|_| UbxError::BadPayload),
                _ => Ok(ParsedPacket::OtherPacket),
            })
        })
    }
}

#[derive(Debug, Copy, Clone)]
pub enum ParsedPacket {
    AckAck { class: u8, id: u8 },
    AckNak { class: u8, id: u8 },

    NavPvt(NavPvt),

    InfNotice(UbxBuf),
    InfError(UbxBuf),

    OtherPacket,
}
