use tinyvec::ArrayVec;

pub mod cfg;
pub mod generator;
pub mod packets;
pub mod parser;

pub use generator::SendablePacket;
pub use parser::{ParsedPacket, UbxParser};

const UBX_BUFSIZE: usize = 256;
#[derive(Default, Debug, Copy, Clone)]
pub struct UbxBuf(pub ArrayVec<[u8; UBX_BUFSIZE]>);

impl defmt::Format for UbxBuf {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}", self.0.as_slice())
    }
}

impl core::ops::Deref for UbxBuf {
    type Target = ArrayVec<[u8; UBX_BUFSIZE]>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl core::ops::DerefMut for UbxBuf {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

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
    BadPayload,
    TooLarge(u16),
}
