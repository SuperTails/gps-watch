
use tinyvec::ArrayVec;

pub mod parser;
pub mod packets;
pub mod generator;
pub mod cfg;

pub use parser::UbxParser;
pub use parser::ParsedPacket;
pub use generator::SendablePacket;

const UBX_BUF_SIZE: usize = 256;
type UbxBuf = ArrayVec<[u8; UBX_BUF_SIZE]>;

#[derive(Debug, Copy, Clone, PartialEq)]
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