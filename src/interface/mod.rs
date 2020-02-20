pub mod i2c;
pub mod spi;

use core::ops::{Shl};

// #[derive(Debug)]
// pub enum CommBusError {
//     /// I2C bus error
//     // I2c(E),
//     // Spi(E),
//
//     /// Invalid sensor product ID was read
//     InvalidChipId(u8),
//
//     /// Unsupported sensor firmware version
//     InvalidFWVersion(u8),
//
//     /// Not enough data available to fulfill the read
//     NoDataAvailable(u8),
// }


/// A method of communicating with the sensor
pub trait SensorInterface {
    /// Interface error type
    type SensorError;

    /// Send the whole packet provided
    fn send_packet(&mut self, packet: &[u8]) -> Result<(), Self::SensorError>;

    /// Read the next packet from the sensor
    /// Returns the size of the packet read (up to the size of the slice provided)
    fn read_packet(&mut self, recv_buf: &mut [u8]) -> Result<usize, Self::SensorError>;

    // Send a packet and receive the response immediately
    //fn send_and_receive_packet(&mut self, send_buf: &[u8], recv_buf: &mut [u8]) -> Result<usize, CommBusError>;
}

pub use self::i2c::I2cInterface;
pub use self::spi::SpiInterface;


pub(crate) const PACKET_HEADER_LENGTH: usize = 4;


struct SensorCommon {

}

impl SensorCommon {
    fn parse_packet_header(packet: &[u8]) -> usize {
        const CONTINUATION_FLAG_CLEAR: u16 = !(0x80);
        if packet.len() < PACKET_HEADER_LENGTH {
            return 0;
        }
        //Bits 14:0 are used to indicate the total number of bytes in the body plus header
        //maximum packet length is ... 32767?
        let raw_pack_len: u16 =
            (packet[0] as u16) + ((packet[1] as u16)
                & CONTINUATION_FLAG_CLEAR).shl(8);
        let packet_len: usize = raw_pack_len as usize;

        //let is_continuation:bool = (packet[1] & 0x80) != 0;
        //let chan_num =  packet[2];
        //let seq_num =  packet[3];

        packet_len
    }
}