pub mod i2c;
pub mod spi;


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

    /// Read enough bytes to fill a packet header
    fn read_packet_header(&mut self, recv_buf: &mut [u8]) -> Result<(), Self::SensorError>;

    /// Read a packet of known size
    fn read_sized_packet(&mut self, total_packet_len: usize, packet_recv_buf: &mut [u8] ) -> Result<usize,  Self::SensorError>;

    // Send a packet and receive the response immediately
    //fn send_and_receive_packet(&mut self, send_buf: &[u8], recv_buf: &mut [u8]) -> Result<usize, CommBusError>;
}

pub use self::i2c::I2cInterface;
pub use self::spi::SpiInterface;


pub(crate) const PACKET_HEADER_LENGTH: usize = 4;

