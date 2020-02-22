/*
Copyright (c) 2020 Todd Stellanova
LICENSE: See LICENSE file
*/

#![no_std]

pub mod interface;
pub mod wrapper;

/// Errors in this crate
#[derive(Debug)]
pub enum Error<CommE, PinE>
{
    /// Sensor communication error
    Comm(CommE),
    /// Pin setting error
    Pin(PinE),
}



