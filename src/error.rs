//! Defines the error type for the crate.

use std::fmt::Debug;

/// The error type for the crate.
#[derive(Clone)]
pub enum Error {
    ParseFamilyStringError(String),
    CreateImageError { reason: String },
    CreateDetectorError { reason: String },
}
impl Debug for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::ParseFamilyStringError(s) => {
                write!(f, "unable to parse family string '{s}'")
            }
            Error::CreateImageError { reason } => {
                write!(f, "Unable to create an image: {reason}")
            }
            Error::CreateDetectorError { reason } => {
                write!(f, "Unable to create a detector: {reason}")
            }
        }
    }
}
