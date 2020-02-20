

use embedded_hal;
// use embedded_hal::{
//     digital::v2::OutputPin,
// };

use super::{SensorInterface};
use crate::interface::{PACKET_HEADER_LENGTH, SensorCommon};


/// This combines the SPI peripheral and a data/command pin
pub struct SpiInterface<SPI> {
    spi: SPI,
    received_packet_count: usize,
}

impl<SPI, CommE> SpiInterface<SPI>
    where
        SPI: embedded_hal::blocking::spi::Write<u8, Error = CommE> +
        embedded_hal::blocking::spi::Transfer<u8, Error = CommE>
{
    pub fn new(spi: SPI) -> Self {
        Self { spi, received_packet_count: 0 }
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

    fn read_packet(&mut self, recv_buf: &mut [u8]) -> Result<usize, Self::SensorError> {
        //ensure that buffer is zeroed since we're not sending any data
        for i in recv_buf.iter_mut() {
            *i = 0;
        }
        //TODO might need to look at INTN pin to detect whether a packet is available
        // get just the header
        self.spi.transfer(&mut recv_buf[..PACKET_HEADER_LENGTH]).map_err(SpiCommError::Spi)?;
        let packet_len = SensorCommon::parse_packet_header(&recv_buf[..PACKET_HEADER_LENGTH]);
        if packet_len > PACKET_HEADER_LENGTH {
            self.spi.transfer( &mut recv_buf[PACKET_HEADER_LENGTH..packet_len]).map_err(SpiCommError::Spi)?;
        }

        if  packet_len > 0 {
            self.received_packet_count += 1;
        }

        Ok(packet_len)
    }
}
