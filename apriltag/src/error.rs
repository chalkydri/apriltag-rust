//! Defines the error type for the crate.

/// The error type for the crate.
#[derive(Debug, Clone, thiserror::Error)]
pub enum Error {
    #[error("unable to parse family string '{{0}}'")]
    ParseFamilyStringError(String),

    #[error("Unable to create image: {reason}")]
    CreateImageError { reason: String },
}
