pub mod i2c;
pub mod spi;


use embedded_hal::{
    blocking::delay::{DelayUs, DelayMs},
};
use crate::PACKET_HEADER_LENGTH;

/// A method of communicating with the sensor
pub trait SensorInterface {
    /// Interface error type
    type Error;

    /// Send the given packet
    fn send_packet(&mut self, channel: u8, packet: &[u8]) -> Result<(), Self::Error>;

    /// Read enough bytes to fill a packet header
    fn read_packet_header(&mut self, recv_buf: &mut [u8; PACKET_HEADER_LENGTH]) -> Result<(), Self::Error>;

    /// Read a packet of known size
    fn read_sized_packet(&mut self, total_packet_len: usize, packet_recv_buf: &mut [u8] ) -> Result<usize,  Self::Error>;

    // Send a packet and receive the response immediately
    //fn send_and_receive_packet(&mut self, send_buf: &[u8], recv_buf: &mut [u8]) -> Result<usize,  Self::Error>;

}

pub use self::i2c::I2cInterface;
pub use self::spi::SpiInterface;

