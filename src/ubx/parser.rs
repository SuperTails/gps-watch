use super::{packets::*, UbxBuf, UbxChecksum, UbxError, UBX_BUFSIZE};

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

impl Default for UbxParser {
    fn default() -> Self {
        Self {
            state: Start,
            buf: UbxBuf::default(),
        }
    }
}

impl UbxParser {
    pub fn new() -> Self {
        Self::default()
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
                if len as usize > UBX_BUFSIZE {
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
                ACK_ACK_ID => {
                    if self.buf.len() == 2 {
                        Ok(ParsedPacket::AckAck {
                            class: self.buf[0],
                            id: self.buf[1],
                        })
                    } else {
                        Err(UbxError::BadPayload)
                    }
                }
                ACK_NAK_ID => {
                    if self.buf.len() == 2 {
                        Ok(ParsedPacket::AckNak {
                            class: self.buf[0],
                            id: self.buf[1],
                        })
                    } else {
                        Err(UbxError::BadPayload)
                    }
                }
                NAV_PVT_ID => bytemuck::try_pod_read_unaligned(&self.buf)
                    .map(ParsedPacket::NavPvt)
                    .map_err(|_| UbxError::BadPayload),
                MON_RF_ID => bytemuck::try_pod_read_unaligned(&self.buf)
                    .map(ParsedPacket::NavPvt)
                    .map_err(|_| UbxError::BadPayload),
                INF_NOTICE_ID => Ok(ParsedPacket::InfNotice(self.buf)),
                INF_WARNING_ID => Ok(ParsedPacket::InfWarning(self.buf)),
                INF_ERROR_ID => Ok(ParsedPacket::InfError(self.buf)),
                _ => Ok(ParsedPacket::OtherPacket),
            })
        })
    }
}

#[derive(defmt::Format, Debug, Copy, Clone)]
pub enum ParsedPacket {
    AckAck { class: u8, id: u8 },
    AckNak { class: u8, id: u8 },

    NavPvt(NavPvt),

    MonRf(MonRf),

    InfNotice(UbxBuf),
    InfWarning(UbxBuf),
    InfError(UbxBuf),

    OtherPacket,
}
