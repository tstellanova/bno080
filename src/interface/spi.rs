

use embedded_hal;
use embedded_hal::{
    blocking::delay::{DelayUs, DelayMs},
    digital::v2::OutputPin,
};

use super::SensorInterface;
use crate::Error;


/// This combines the SPI peripheral and a data/command pin
pub struct SpiInterface<SPI> {
    spi: SPI,
}

impl<SPI, CommE> SpiInterface<SPI>
    where
        SPI: embedded_hal::blocking::spi::Write<u8, Error = CommE>
{
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }
}

impl<SPI, CommE> SensorInterface for SpiInterface<SPI>
    where
        SPI: embedded_hal::blocking::spi::Write<u8, Error = CommE>
{
    type Error = Error<CommE>;


    fn send_packet(&mut self, channel: u8, packet: &[u8]) -> Result<(), Self::Error> {
        //        self.spi.write(&cmds).map_err(Error::Comm)?;
        unimplemented!()
    }

    fn read_packet_header(&mut self, recv_buf: &mut [u8; 4]) -> Result<(), Self::Error> {
        unimplemented!()
    }

    fn read_sized_packet(&mut self, total_packet_len: usize, packet_recv_buf: &mut [u8]) -> Result<usize, Self::Error> {
        unimplemented!()
    }
}
