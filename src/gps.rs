use core::fmt::{self, Write};

use defmt::{error, info, trace};
use stm32l4xx_hal::hal::serial;
use ublox::{CfgMsgAllPortsBuilder, CfgPrtUartBuilder, NavPvt, UartMode, UartPortId};
use tinyvec::ArrayVec;

use crate::{ubx::{UbxError, UbxPacket, UbxParser}, FmtBuf};

#[derive(Debug, Copy, Clone)]
pub struct Position {
    pub latitude: i32,
    pub longitude: i32,
}

pub struct Gps<SERIAL> {
    pub serial: SERIAL,
    pub parser: UbxParser,

    pub position: Position,
    pub last_packet: Result<UbxPacket, UbxError>,
    pub count: usize
}

impl<SERIAL> Gps<SERIAL>
    where SERIAL: serial::Read<u8> + serial::Write<u8>,
          <SERIAL as serial::Read<u8>>::Error: core::fmt::Debug,
          <SERIAL as serial::Write<u8>>::Error: core::fmt::Debug,
{
    pub fn new(mut serial: SERIAL) -> Self {

        // Busy loop waiting for the GPS to be alive
        while let Err(_) = serial.read() {}

        let mut s = Self {
            serial,
            parser: UbxParser::new(),
            position: Position { latitude: 0, longitude: 0 },
            last_packet: Ok(UbxPacket::OtherPacket),
            count: 0
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
            self.count += 1;
            trace!("got {:x} #{}", b, self.count);
            if let Some(r) = self.parser.process_byte(b) {
                if let Ok(UbxPacket::NavPvt(ref navpvt)) = r {
                    self.position.latitude = navpvt.lat;
                    self.position.longitude = navpvt.lon;
                }
                self.last_packet = r;
            }
        }
    }
}