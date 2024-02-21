// use tinyvec::ArrayVec;
//
//
// pub struct NmeaParser {
//     buf: ArrayVec<[u8; 100]>
// }
//
// impl NmeaParser {
//     pub fn new() -> Self {
//         Self {
//             buf: ArrayVec::new()
//         }
//     }
//
//     pub fn parse_byte(&mut self, byte: u8) -> Option<Result<NmeaSentence, ()>> {
//         if byte == b'\n' {
//             Some(self.parse_buf())
//         }
//         else {
//             None
//         }
//     }
//
//     fn parse_buf(&mut self) -> Result<NmeaSentence, ()> {
//         // self.buf
//         self.buf.clear();
//     }
// }
//
// pub enum NmeaSentence {
//     Gga {
//         utc: f32,
//         lat: f32,
//         lon: f32,
//
//     }
// }
//
// // type Text<const len: usize> = ArrayVec<[u8; len]>;
// // enum ParserState {
// //     StartUnknown, // Default or unknown state
// //     StartSentence, // Start of sentence `$`
// //     TalkerId(Text<2>), // Receiving the Talker ID `GP`
// //     SentenceId(Text<3>), // Reading the Sentence ID `GGA`
// //     // GGA components
// //     GgaUtc(Text<16>),
// //     GgaLat(f32, Text<16>),
// //     GgaLatDir(f32, f32, bool), // true = N
// //     GgaLon(f32, f32, bool, Text<16>),
// //     GgaLonDir(f32, f32, bool, f32, bool), // true = E
// //     GgaQual(f32, f32, bool, f32, bool, u8),
// //     GgaSv(f32, f32, bool, f32, bool, u8, Text<3>),
// //     GgaHdop(f32, f32, bool, f32, bool, u8, u8, Text<16>),
// //     GgaOrthoHeight(f32, f32, bool, f32, u8, u8, f32, Text<16>),
// //     GgaOrthoUnit(f32, f32, bool, f32, u8, u8, f32, Text<16>),
// //
// // }