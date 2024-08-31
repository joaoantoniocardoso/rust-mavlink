use bytes::{Buf, BytesMut};
use tokio_util::codec::Decoder;

use crate::{MAVLinkV2MessageRaw, Message, MAVLINK_SUPPORTED_IFLAGS, MAV_STX_V2};

#[derive(Debug)]
pub enum DecoderState {
    SearchForMagicByte,
    ParseHeader,
    ParsePayload,
    CheckCRC,
}

#[derive(Debug)]
pub struct MAVLinkV2Codec<M: Message> {
    _phantom: std::marker::PhantomData<M>,
    pub state: DecoderState,
    message: MAVLinkV2MessageRaw,
}

impl<M: Message> MAVLinkV2Codec<M> {
    pub fn new() -> Self {
        Self {
            _phantom: std::marker::PhantomData,
            state: DecoderState::SearchForMagicByte,
            message: MAVLinkV2MessageRaw::new(),
        }
    }

    pub fn reset(&mut self) {
        self.state = DecoderState::SearchForMagicByte;
        self.message = MAVLinkV2MessageRaw::new();
    }
}

impl<M: Message> Default for MAVLinkV2Codec<M> {
    fn default() -> Self {
        Self::new()
    }
}

impl<M: Message> Decoder for MAVLinkV2Codec<M> {
    type Item = MAVLinkV2MessageRaw;
    type Error = std::io::Error;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        loop {
            match self.state {
                DecoderState::SearchForMagicByte => {
                    loop {
                        // Search for the magic byte MAV_STX_V2
                        if src.is_empty() {
                            return Ok(None);
                        }

                        let magic = src[0];
                        src.advance(1);

                        if magic == MAV_STX_V2 {
                            break;
                        }
                    }
                    self.message.0[0] = MAV_STX_V2;

                    self.state = DecoderState::ParseHeader;
                    src.reserve(MAVLinkV2MessageRaw::HEADER_SIZE);
                    continue;
                }
                DecoderState::ParseHeader => {
                    // Only continue when we have the full header
                    if src.len() >= MAVLinkV2MessageRaw::HEADER_SIZE {
                        self.message.mut_header()[..MAVLinkV2MessageRaw::HEADER_SIZE]
                            .copy_from_slice(&src[..MAVLinkV2MessageRaw::HEADER_SIZE]);

                        // If there are incompatibility flags set that we do not know, it might actually the wrong start of the message
                        if self.message.incompatibility_flags() & !MAVLINK_SUPPORTED_IFLAGS > 0 {
                            self.reset();
                            continue;
                        }

                        self.state = DecoderState::ParsePayload;
                        let full_payload_len = self.message.payload_length() as usize;
                        let checksum_len: usize = 2;
                        let signature_len: usize = self.message.signature_lenght() as usize;
                        let full_payload_and_checksum_and_sign_len =
                            full_payload_len + checksum_len + signature_len;
                        src.reserve(full_payload_and_checksum_and_sign_len);
                        continue;
                    }
                }
                DecoderState::ParsePayload => {
                    let full_payload_len = self.message.payload_length() as usize;
                    let checksum_len: usize = 2;
                    let signature_len: usize = self.message.signature_lenght() as usize;
                    let full_payload_and_checksum_and_sign_len =
                        full_payload_len + checksum_len + signature_len;
                    let offset = MAVLinkV2MessageRaw::HEADER_SIZE;

                    if src.len() >= full_payload_and_checksum_and_sign_len + offset {
                        self.message.mut_payload_and_checksum_and_sign()
                            [..full_payload_and_checksum_and_sign_len]
                            .copy_from_slice(
                                &src[offset..full_payload_and_checksum_and_sign_len + offset],
                            );

                        self.state = DecoderState::CheckCRC;
                        continue;
                    }
                }
                DecoderState::CheckCRC => {
                    if !self.message.has_valid_crc::<M>() {
                        self.reset();
                        continue;
                    }

                    // src.advance(
                    //     MAVLinkV2MessageRaw::HEADER_SIZE + self.message.payload_length() as usize,
                    // );

                    src.reserve(1 + MAVLinkV2MessageRaw::HEADER_SIZE);
                    return Ok(Some(self.message));
                }
            }

            // Wait for more data
            return Ok(None);
        }
    }
}

mod tests {
    #![allow(unused)]

    use core::ops::Div;
    use std::io::Read;

    // #[allow(unused_imports)]
    use super::*;
    use bytes::{BufMut, BytesMut};

    const COMMAND_LONG_TRUNCATED_V2: &[u8] = &[
        crate::MAV_STX_V2, //magic
        30,                //payload len
        0,                 //incompat flags
        0,                 //compat flags
        0,
        0,
        50, //header
        76,
        0,
        0, //msg ID
        //truncated payload:
        0,
        0,
        230,
        66,
        0,
        64,
        156,
        69,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        255,
        1,
        // crc:
        188,
        195,
    ];

    #[test]
    fn test_v2_tokio_codec_decode() {
        let mut original_raw_message = MAVLinkV2MessageRaw::new();
        original_raw_message.0[..COMMAND_LONG_TRUNCATED_V2.len()]
            .copy_from_slice(COMMAND_LONG_TRUNCATED_V2);

        // Create a buffer containing the raw message data
        let msgs = 3;
        let mut buf = BytesMut::with_capacity((msgs + 1) * 280);

        // Start the buffer at the middle of a message
        buf.put_slice(
            &original_raw_message.0
                [COMMAND_LONG_TRUNCATED_V2.len().div(2)..COMMAND_LONG_TRUNCATED_V2.len()],
        );

        // Add the complete messages we want to parse
        for _ in 0..msgs {
            buf.put_slice(&original_raw_message.0[..COMMAND_LONG_TRUNCATED_V2.len()]);
        }

        // Add some incomplete message at the end
        buf.put_slice(&original_raw_message.0[..COMMAND_LONG_TRUNCATED_V2.len().div(2)]);

        // Create the codec
        let mut codec = MAVLinkV2Codec::<MockMessage>::new();

        for _ in 0..msgs {
            // Attempt to decode the buffer
            match codec.decode(&mut buf) {
                Ok(Some(decoded_message)) => {
                    assert_eq!(decoded_message.0, original_raw_message.0);
                    codec.reset();
                }
                Ok(None) => panic!("Decoder did not produce a message when it should have"),
                Err(error) => panic!("Decoder failed: {:?}", error),
            }
        }
    }

    #[test]
    fn test_v2_tokio_codec_decode_byte_per_byte() {
        let mut original_raw_message = MAVLinkV2MessageRaw::new();
        original_raw_message.0[..COMMAND_LONG_TRUNCATED_V2.len()]
            .copy_from_slice(COMMAND_LONG_TRUNCATED_V2);

        // Create an empty buffer
        let mut buf = BytesMut::with_capacity(280);

        // Create the codec
        let mut codec = MAVLinkV2Codec::<MockMessage>::new();

        let mut bytes_added = 0;
        let mut iterations = 0u8;
        loop {
            // Add byte to the buffer at every second iteration
            if iterations % 2 == 0 {
                assert!(
                    bytes_added <= COMMAND_LONG_TRUNCATED_V2.len() + 1,
                    "Decoder failed after the entire message was added"
                );

                buf.put_u8(original_raw_message.0[bytes_added]);
                bytes_added += 1;

                println!(
                    "buf\t: {:?}\ncodec\t: {:?}\nstate\t: {:?}\n",
                    buf.bytes(),
                    codec.message.0.bytes(),
                    codec.state,
                );
            }
            iterations += 1;

            // Attempt to decode the buffer
            match codec.decode(&mut buf) {
                Ok(Some(decoded_message)) => {
                    assert_eq!(decoded_message.0, original_raw_message.0);
                    codec.reset();
                    break;
                }
                Ok(None) => (),
                Err(error) => panic!("Decoder failed: {:?}. Codec: {:?}", error, codec.message),
            }
        }
    }

    #[test]
    fn test_v2_tokio_codec_decode_resilience_against_rogue_magic_byte() {
        let mut original_raw_message = MAVLinkV2MessageRaw::new();
        original_raw_message.0[..COMMAND_LONG_TRUNCATED_V2.len()]
            .copy_from_slice(COMMAND_LONG_TRUNCATED_V2);

        let msgs = 1;

        // Create an empty buffer
        let mut buf = BytesMut::with_capacity((msgs + 1) * 280);

        // Add some bytes the buffer to contain a rogue magic byte
        buf.put_u8(MAV_STX_V2);
        buf.put_u8(10);
        buf.put_u8(0);

        // Create the codec
        let mut codec = MAVLinkV2Codec::<MockMessage>::new();

        let mut bytes_added = 0;
        let mut iterations = 0u8;
        loop {
            // Add byte to the buffer at every second iteration
            if iterations % 2 == 0 {
                assert!(
                    bytes_added <= COMMAND_LONG_TRUNCATED_V2.len() + 1,
                    "Decoder failed after the entire message was added"
                );

                buf.put_u8(original_raw_message.0[bytes_added]);
                bytes_added += 1;

                println!(
                    "buf\t: {:?}\ncodec\t: {:?}\nstate\t: {:?}\n",
                    buf.bytes(),
                    codec.message.0.bytes(),
                    codec.state,
                );
            }
            iterations += 1;

            // Attempt to decode the buffer
            match codec.decode(&mut buf) {
                Ok(Some(decoded_message)) => {
                    assert_eq!(decoded_message.0, original_raw_message.0);
                    codec.reset();
                    break;
                }
                Ok(None) => (),
                Err(error) => panic!("Decoder failed: {:?}. Codec: {:?}", error, codec.message),
            }
        }
    }

    // Mock `Message` implementation for testing
    #[derive(Debug)]
    struct MockMessage;

    impl Message for MockMessage {
        fn message_id(&self) -> u32 {
            0
        }

        fn message_name(&self) -> &'static str {
            unimplemented!()
        }

        fn ser(&self, version: crate::MavlinkVersion, bytes: &mut [u8]) -> usize {
            unimplemented!()
        }

        fn parse(
            version: crate::MavlinkVersion,
            msgid: u32,
            payload: &[u8],
        ) -> Result<Self, crate::error::ParserError> {
            unimplemented!()
        }

        fn message_id_from_name(name: &str) -> Result<u32, &'static str> {
            unimplemented!()
        }

        fn default_message_from_id(id: u32) -> Result<Self, &'static str> {
            unimplemented!()
        }

        fn extra_crc(id: u32) -> u8 {
            152
        }
    }
}
