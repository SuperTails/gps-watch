use core::future::Future;

use crate::rb::Producer;
use super::UbxChecksum;

pub trait SendablePacket: Sized + 'static {
    type I: Iterator<Item = u8> + 'static;

    fn class(&self) -> u8;
    fn id(&self) -> u8;
    fn payload_len(&self) -> usize;
    fn payload_bytes(self) -> Self::I;

    fn to_bytes(self) -> UbxGenerator<Self, Self::I>
    where
        Self: Sized,
    {
        UbxGenerator(Some(GeneratorState::Sync1 { packet: self }))
    }

    fn packet_len(&self) -> usize {
        8 + self.payload_len()
    }

    fn send<const N: usize>(self, tx: &Producer<u8, N>) -> impl Future<Output = usize> {
        tx.async_write_iter(self.to_bytes())
    }
}

// States are named for the portion of the packet which is *about to be sent*
enum GeneratorState<T, I>
where
    T: SendablePacket<I = I>,
{
    Sync1 { packet: T },
    Sync2 { packet: T },
    Class { packet: T },
    Id { packet: T, checksum: UbxChecksum },
    Len1 { packet: T, checksum: UbxChecksum },
    Len2 { packet: T, checksum: UbxChecksum },
    Payload { iter: I, checksum: UbxChecksum },
    Checksum2 { checksum: UbxChecksum },
    Done,
}
use GeneratorState::*;

pub struct UbxGenerator<T, I>(Option<GeneratorState<T, I>>)
where
    T: SendablePacket<I = I>;

impl<T, I> UbxGenerator<T, I>
where
    T: SendablePacket<I = I>,
{
    pub fn done(&self) -> bool {
        matches!(self.0, Some(Done))
    }
}

impl<T, I> Iterator for UbxGenerator<T, I>
where
    T: SendablePacket<I = I>,
    I: Iterator<Item = u8>,
{
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        match core::mem::take(&mut self.0).unwrap() {
            Sync1 { packet } => {
                self.0 = Some(Sync2 { packet });
                Some(0xb5)
            }
            Sync2 { packet } => {
                self.0 = Some(Class { packet });
                Some(0x62)
            }
            Class { packet } => {
                let val = packet.class();
                self.0 = Some(Id {
                    packet,
                    checksum: UbxChecksum::new().next(val),
                });
                Some(val)
            }
            Id { packet, checksum } => {
                let val = packet.id();
                self.0 = Some(Len1 {
                    packet,
                    checksum: checksum.next(val),
                });
                Some(val)
            }
            Len1 { packet, checksum } => {
                let val = (packet.payload_len() as u16).to_le_bytes()[0];
                self.0 = Some(Len2 {
                    packet,
                    checksum: checksum.next(val),
                });
                Some(val)
            }
            Len2 { packet, checksum } => {
                let val = (packet.payload_len() as u16).to_le_bytes()[1];
                self.0 = Some(Payload {
                    iter: packet.payload_bytes(),
                    checksum: checksum.next(val),
                });
                Some(val)
            }
            Payload { mut iter, checksum } => {
                if let Some(val) = iter.next() {
                    self.0 = Some(Payload {
                        iter,
                        checksum: checksum.next(val),
                    });
                    Some(val)
                } else {
                    self.0 = Some(Checksum2 { checksum });
                    Some(checksum.0)
                }
            }
            Checksum2 { checksum } => {
                self.0 = Some(Done);
                Some(checksum.1)
            }
            Done => None,
        }
    }
}
