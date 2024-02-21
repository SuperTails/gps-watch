use defmt::{error, info};
use stm32l4xx_hal::hal::serial;
use ublox::{PacketRef, Parser};
use tinyvec::ArrayVec;

pub struct Gps<SERIAL> {
    serial: SERIAL,
    parser: Parser<ArrayVec<[u8; 256]>>
}

impl<SERIAL> Gps<SERIAL>
    where SERIAL: serial::Read<u8>,
          SERIAL::Error: core::fmt::Debug
{
    pub fn new(serial: SERIAL) -> Self {
        Self {
            serial,
            parser: Parser::new(ArrayVec::new())
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
                    Some(Err(_)) => error!("ubx error"),
                    None => break
                }
            }
        }
    }
}