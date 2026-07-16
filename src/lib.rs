pub mod controller;
pub mod math;
pub mod navigation;
pub mod robot;
pub mod sensors;
pub mod telemetry;

use std::error::Error;
use std::fmt;
use std::io;
use std::sync::mpsc;

#[derive(Debug)]
pub enum SimulationError {
    Io(io::Error),
    ChannelClosed(&'static str),
    InvalidConfig(&'static str),
}

impl fmt::Display for SimulationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SimulationError::Io(err) => write!(f, "I/O error: {err}"),
            SimulationError::ChannelClosed(name) => write!(f, "channel closed: {name}"),
            SimulationError::InvalidConfig(msg) => write!(f, "invalid configuration: {msg}"),
        }
    }
}

impl Error for SimulationError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            SimulationError::Io(err) => Some(err),
            SimulationError::ChannelClosed(_) | SimulationError::InvalidConfig(_) => None,
        }
    }
}

impl From<io::Error> for SimulationError {
    fn from(value: io::Error) -> Self {
        SimulationError::Io(value)
    }
}

impl<T> From<mpsc::SendError<T>> for SimulationError {
    fn from(_: mpsc::SendError<T>) -> Self {
        SimulationError::ChannelClosed("send")
    }
}
