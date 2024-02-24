use core::fmt::{self, Write};

use defmt::{error, info};
use stm32l4xx_hal::hal::serial;
use ublox::{CfgMsgAllPortsBuilder, CfgPrtUartBuilder, NavPvt, PacketRef, Parser, UartMode, UartPortId};
use tinyvec::ArrayVec;

struct WriteableArrayVec(ArrayVec<[u8; 256]>);

impl Write for WriteableArrayVec {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for b in s.bytes() {
            self.0.try_push(b);
        }
        Ok(())
    }
}

impl WriteableArrayVec {
    pub fn as_str(&self) -> Option<&str> {
        core::str::from_utf8(self.0.as_slice()).ok()
    }
}

pub struct Gps<SERIAL> {
    pub serial: SERIAL,
    pub parser: Parser<ArrayVec<[u8; 256]>>
}

impl<SERIAL> Gps<SERIAL>
    where SERIAL: serial::Read<u8> + serial::Write<u8>,
          <SERIAL as serial::Read<u8>>::Error: core::fmt::Debug,
          <SERIAL as serial::Write<u8>>::Error: core::fmt::Debug,
{
    pub fn new(serial: SERIAL) -> Self {
        let mut s = Self {
            serial,
            parser: Parser::new(ArrayVec::new())
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
            let mut it = self.parser.consume(core::slice::from_ref(&b));
            loop {
                match it.next() {
                    Some(Ok(pkt)) => match pkt {
                        PacketRef::NavPvt(navpvt) => info!("lat: {}, lon: {}", navpvt.lat_degrees(), navpvt.lon_degrees()),
                        _ => info!("other packet")
                    },
                    Some(Err(err)) => {
                        let mut s = WriteableArrayVec(Default::default());
                        write!(&mut s, "{err:?}").unwrap();
                        if let Some(s) = s.as_str() {
                            error!("ubx error {}", s);
                        } else {
                            error!("ubx error");
                        }
                    }
                    None => break
                }
            }
        }
    }
}