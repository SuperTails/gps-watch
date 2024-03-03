/*use defmt::trace;
use ublox::{CfgMsgAllPortsBuilder, CfgPrtUartBuilder, NavPvt as TxNavPvt, UartMode, UartPortId};

use crate::ubx::{NavPvt, UbxError, UbxPacket, UbxParser};

pub struct Gps {
    pub parser: UbxParser,

    pub last_packet: Option<Result<UbxPacket, UbxError>>,
    pub last_navpvt: Option<NavPvt>,
    pub count: usize,
}

impl Gps {
    pub fn new(mut serial: SERIAL) -> Self {
        // Busy loop waiting for the GPS to be alive
        while let Err(_) = serial.read() {}

        let mut s = Self {
            parser: UbxParser::new(),
            last_packet: None,
            last_navpvt: None,
            count: 0,
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
        }
        .into_packet_bytes();

        for b in packet {
            nb::block!(self.serial.write(b)).unwrap();
        }

        let packet =
            CfgMsgAllPortsBuilder::set_rate_for::<TxNavPvt>([1, 1, 1, 1, 1, 1]).into_packet_bytes();

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
                    self.last_navpvt = Some(*navpvt);
                }
                self.last_packet = Some(r);
            }
        }
    }
}
*/
