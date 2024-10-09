use core::fmt::{Display, Formatter};
#[cfg(feature = "std")]
use std::error::Error;

use crate::MAVLinkMessageRaw;

#[derive(Debug)]
pub enum ParserError {
    InvalidFlag {
        flag_type: &'static str,
        value: u32,
    },
    InvalidEnum {
        enum_type: &'static str,
        value: u32,
    },
    InvalidCRC {
        crc: u16,
        calculated_crc: u16,
        message: Box<MAVLinkMessageRaw>,
    },
    UnknownMessage {
        id: u32,
    },
}

impl Display for ParserError {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidFlag { flag_type, value } => write!(
                f,
                "Invalid flag value for flag type {flag_type:?}, got {value:?}"
            ),
            Self::InvalidEnum { enum_type, value } => write!(
                f,
                "Invalid enum value for enum type {enum_type:?}, got {value:?}"
            ),
            Self::UnknownMessage { id } => write!(f, "Unknown message with ID {id:?}"),
            Self::InvalidCRC {
                crc,
                calculated_crc,
                message: _,
            } => write!(f, "Invalid CRC value {crc:?}, got {calculated_crc:?}",),
        }
    }
}

#[cfg(feature = "std")]
impl Error for ParserError {}

#[derive(Debug)]
pub enum MessageReadError {
    #[cfg(feature = "std")]
    Io(std::io::Error),
    #[cfg(any(feature = "embedded", feature = "embedded-hal-02"))]
    Io,
    Parse(ParserError),
}

impl Display for MessageReadError {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            #[cfg(feature = "std")]
            Self::Io(e) => write!(f, "Failed to read message: {e:#?}"),
            #[cfg(any(feature = "embedded", feature = "embedded-hal-02"))]
            Self::Io => write!(f, "Failed to read message"),
            Self::Parse(e) => write!(f, "Failed to read message: {e:#?}"),
        }
    }
}

#[cfg(feature = "std")]
impl Error for MessageReadError {}

#[cfg(feature = "std")]
impl From<std::io::Error> for MessageReadError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

impl From<ParserError> for MessageReadError {
    fn from(e: ParserError) -> Self {
        Self::Parse(e)
    }
}

#[derive(Debug)]
pub enum MessageWriteError {
    #[cfg(feature = "std")]
    Io(std::io::Error),
    #[cfg(any(feature = "embedded", feature = "embedded-hal-02"))]
    Io,
}

impl Display for MessageWriteError {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            #[cfg(feature = "std")]
            Self::Io(e) => write!(f, "Failed to write message: {e:#?}"),
            #[cfg(any(feature = "embedded", feature = "embedded-hal-02"))]
            Self::Io => write!(f, "Failed to write message"),
        }
    }
}

#[cfg(feature = "std")]
impl Error for MessageWriteError {}

#[cfg(feature = "std")]
impl From<std::io::Error> for MessageWriteError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}
