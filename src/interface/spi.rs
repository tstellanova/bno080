

use embedded_hal;
// use embedded_hal::{
//     digital::v2::OutputPin,
// };

use super::{SensorInterface};
use crate::interface::PACKET_HEADER_LENGTH;


/// This combines the SPI peripheral and a data/command pin
pub struct SpiInterface<SPI> {
    spi: SPI,
}

impl<SPI, CommE> SpiInterface<SPI>
    where
        SPI: embedded_hal::blocking::spi::Write<u8, Error = CommE> +
        embedded_hal::blocking::spi::Transfer<u8, Error = CommE>
{
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }
}

#[derive(Debug)]
pub enum SpiCommError<E> {
    /// Spi bus error
    Spi(E),
}

impl<SPI, CommE> SensorInterface for SpiInterface<SPI>
    where
        SPI: embedded_hal::blocking::spi::Write<u8, Error = CommE> +
        embedded_hal::blocking::spi::Transfer<u8, Error = CommE>
{
    type SensorError = SpiCommError<CommE>;

    fn send_packet(&mut self, packet: &[u8]) -> Result<(), Self::SensorError> {
        self.spi.write(&packet).map_err(SpiCommError::Spi)?;
        Ok(())
    }

    fn read_packet_header(&mut self, recv_buf: &mut [u8]) -> Result<(), Self::SensorError> {
        //ensure that buffer is zeroed
        for i in recv_buf.iter_mut() {
            *i = 0;
        }
        //TODO not clear if transfer modifies the buffer sent?
        self.spi.transfer(&mut recv_buf[..PACKET_HEADER_LENGTH]).map_err(piCommError::Spi)?;
        Ok(())
    }

    fn read_sized_packet(&mut self, total_packet_len: usize, recv_buf: &mut [u8]) -> Result<usize, Self::SensorError> {
        //ensure that buffer is zeroed
        for i in recv_buf.iter_mut() {
            *i = 0;
        }
        self.spi.transfer( &mut recv_buf).map_err(SpiCommError::Spi)?;
        unimplemented!()
    }
}
