/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

#[cfg(all(feature = "sync", feature = "async"))]
compile_error!("features `sync` and `async` are mutually exclusive");

#[cfg(not(any(feature = "sync", feature = "async")))]
compile_error!("either `sync` or `async` must be enabled");

#[cfg(any(
    all(feature = "sync", not(feature = "async")),
    all(feature = "async", not(feature = "sync"))
))]
pub mod interface; pub mod wrapper;

/// Errors in this crate
#[derive(Debug)]
pub enum Error<CommE, PinE> {
    /// Sensor communication error
    Comm(CommE),
    /// Pin setting error
    Pin(PinE),

    /// The sensor is not responding
    SensorUnresponsive,
}
