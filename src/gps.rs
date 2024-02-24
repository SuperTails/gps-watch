use core::fmt::{self, Write};

use defmt::{error, info};
use stm32l4xx_hal::hal::serial;
use ublox::{CfgMsgAllPortsBuilder, CfgPrtUartBuilder, NavPvt, PacketRef, Parser, UartMode, UartPortId};
use tinyvec::ArrayVec;

use crate::FmtBuf;



const UBLOX_BUFFER_SIZE: usize = 512;

#[derive(Debug, defmt::Format)]
pub enum State {
    Sync0,
    Sync1,
    Class,
    Id,
    Length0,
    Length1 { lo: u8 },
    Body { remaining: u16 },
    Checksum0,
    Checksum1,
}

pub struct Position {
    latitude: u32,
    longitude: u32,
}

pub struct Gps<SERIAL> {
    pub serial: SERIAL,
    pub parser: Parser<ArrayVec<[u8; UBLOX_BUFFER_SIZE]>>,
    pub state: State,

    pub position: Position,
}

impl<SERIAL> Gps<SERIAL>
    where SERIAL: serial::Read<u8> + serial::Write<u8>,
          <SERIAL as serial::Read<u8>>::Error: core::fmt::Debug,
          <SERIAL as serial::Write<u8>>::Error: core::fmt::Debug,
{
    pub fn new(serial: SERIAL) -> Self {
        let mut s = Self {
            serial,
            parser: Parser::new(ArrayVec::new()),
            state: State::Sync0,
            position: Position { latitude: 0, longitude: 0 }
        };

        s.configure();

        s
    }

    fn configure(&mut self) {
        let packet = CfgPrtUartBuilder {
            portid: UartPortId::Uart1,
            reserved0: 0,
            tx_ready: 0,
            mode: UartMode::new(
                ublox::DataBits::Eight,
                ublox::Parity::None,
                ublox::StopBits::One,
            ),
            baud_rate: 9600,
            in_proto_mask: ublox::InProtoMask::UBLOX,
            out_proto_mask: ublox::OutProtoMask::UBLOX,
            flags: 0,
            reserved5: 0,
        }.into_packet_bytes();

        for b in packet {
            nb::block!(self.serial.write(b)).unwrap();
        }

        let packet = CfgMsgAllPortsBuilder::set_rate_for::<NavPvt>([1, 1, 1, 1, 1, 1]).into_packet_bytes();

        for b in packet {
            nb::block!(self.serial.write(b)).unwrap();
        }
    }

    pub fn handle(&mut self) {
        if let Ok(b) = self.serial.read() {
            //let mut it = self.parser.consume(core::slice::from_ref(&b));
            
            //info!("stuff {}", self.state);

            match self.state {
                State::Sync0 => {
                    if b == 0xB5 {
                        self.parser.consume(core::slice::from_ref(&b));
                        self.state = State::Sync1;
                    }
                }
                State::Sync1 => {
                    if b == 0x62 {
                        self.parser.consume(core::slice::from_ref(&b));
                        self.state = State::Class;
                    } else {
                        self.state = State::Sync0;
                    }
                }
                State::Class => {
                    self.parser.consume(core::slice::from_ref(&b));
                    self.state = State::Id;
                }
                State::Id => {
                    self.parser.consume(core::slice::from_ref(&b));
                    self.state = State::Length0;
                }
                State::Length0 => {
                    self.parser.consume(core::slice::from_ref(&b));
                    self.state = State::Length1 { lo: b }
                }
                State::Length1 { lo } => {
                    self.parser.consume(core::slice::from_ref(&b));
                    let remaining = u16::from_le_bytes([lo, b]);
                    if remaining == 0 {
                        self.state = State::Checksum0;
                    } else {
                        self.state = State::Body { remaining };
                    }
                }
                State::Body { remaining } => {
                    self.parser.consume(core::slice::from_ref(&b));
                    if remaining == 1 {
                        self.state = State::Checksum0;
                    } else {
                        self.state = State::Body { remaining: remaining - 1 }
                    }
                }
                State::Checksum0 => {
                    self.parser.consume(core::slice::from_ref(&b));
                    self.state = State::Checksum1;
                }
                State::Checksum1 => {
                    let mut it = self.parser.consume(core::slice::from_ref(&b));

                    loop {
                        match it.next() {
                            Some(Ok(pkt)) => match pkt {
                                PacketRef::NavPvt(navpvt) => info!("lat: {}, lon: {}", navpvt.lat_degrees(), navpvt.lon_degrees()),
                                _ => info!("other packet")
                            },
                            Some(Err(err)) => {
                                let mut s = FmtBuf(Default::default());
                                write!(&mut s, "{err:?}").unwrap();
                                if let Some(s) = s.as_str() {
                                    error!("ubx error {}", s);
                                } else {
                                    error!("ubx error");
                                }
                            }
                            None => break,
                        }
                    }

                    self.state = State::Sync0;
                }
            }
        }
    }
}