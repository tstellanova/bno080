

use embedded_hal;
// use embedded_hal::{
//     digital::v2::OutputPin,
// };

use super::{SensorInterface};
use crate::interface::{PACKET_HEADER_LENGTH, SensorCommon};
use embedded_hal::digital::v2::{OutputPin, InputPin};
use embedded_hal::blocking::delay::DelayMs;

use crate::Error;

/// This combines the SPI peripheral and a data/command pin
pub struct SpiInterface<SPI, CS, IN, WN, RS> {
    spi: SPI,
    cs: CS,
    hintn: IN,
    waken: WN,
    reset: RS,
    received_packet_count: usize,
}

impl<SPI, CS, IN, WN, RS, CommE, PinE> SpiInterface<SPI, CS, IN, WN, RS>
    where
        SPI: embedded_hal::blocking::spi::Write<u8, Error = CommE> +
        embedded_hal::blocking::spi::Transfer<u8, Error = CommE>,
        CS: OutputPin<Error = PinE>,
        IN: InputPin<Error = PinE>,
        WN: OutputPin<Error = PinE>,
        RS: OutputPin<Error = PinE>,
{
    pub fn new(spi: SPI, cs: CS, hintn: IN, waken: WN, reset: RS) -> Self {
        Self {
            spi,
            cs,
            hintn,
            waken,
            reset,
            received_packet_count: 0
        }
    }


    fn sensor_ready(&self) ->  bool  {
        self.hintn.is_low().unwrap_or(false)
    }

    /// return true when the sensor is ready
    fn wait_for_ready(&mut self, delay_source: &mut impl DelayMs<u8>) -> bool {
        self.wait_for_data_available(200, delay_source)
    }
}

impl<SPI, CS, IN, WN, RS, CommE, PinE> SensorInterface for SpiInterface<SPI, CS, IN, WN, RS>
    where
        SPI: embedded_hal::blocking::spi::Write<u8, Error = CommE> +
        embedded_hal::blocking::spi::Transfer<u8, Error = CommE>,
        CS: OutputPin<Error = PinE>,
        IN: InputPin<Error = PinE>,
        WN: OutputPin<Error = PinE>,
        RS: OutputPin<Error = PinE>,
{
    type SensorError = Error<CommE, PinE>;

    fn setup(&mut self, delay_source: &mut impl DelayMs<u8>) -> Result<(), Self::SensorError> {
        self.waken.set_high().map_err(Error::Pin)?;
        self.reset.set_low().map_err(Error::Pin)?;
        delay_source.delay_ms(2);

        self.reset.set_high().map_err(Error::Pin)?;

        if self.wait_for_ready(delay_source) {
            self.waken.set_low().map_err(Error::Pin)?;
            return Ok(())
        }

        //TODO error condition
        Ok(())
    }

    fn wait_for_data_available(&mut self, max_ms: u8, delay_source: &mut impl DelayMs<u8>) -> bool {
        for _i in 0..max_ms {
            delay_source.delay_ms(1);
            if self.sensor_ready() {
                return true;
            }
        }

        false
    }

    fn send_packet(&mut self, packet: &[u8]) -> Result<(), Self::SensorError> {
        self.waken.set_low().map_err(Error::Pin)?;
        self.cs.set_low().map_err(Error::Pin)?;

        self.spi.write(&packet).map_err(Error::Comm)?;
        self.cs.set_high().map_err(Error::Pin)?;
        Ok(())
    }

    fn read_packet(&mut self, recv_buf: &mut [u8]) -> Result<usize, Self::SensorError> {
        self.waken.set_low().map_err(Error::Pin)?;

        if !self.sensor_ready() {
            return Ok(0)
        }

        self.cs.set_low().map_err(Error::Pin)?;

        //ensure that buffer is zeroed since we're not sending any data
        for i in recv_buf.iter_mut() {
            *i = 0;
        }
        //TODO might need to look at INTN pin to detect whether a packet is available
        // get just the header
        self.spi.transfer(&mut recv_buf[..PACKET_HEADER_LENGTH]).map_err(Error::Comm)?;
        let mut packet_len = SensorCommon::parse_packet_header(&recv_buf[..PACKET_HEADER_LENGTH]);
        if packet_len > PACKET_HEADER_LENGTH {
            if packet_len < recv_buf.len() {
                self.spi.transfer( &mut recv_buf[PACKET_HEADER_LENGTH..packet_len]).map_err(Error::Comm)?;
            }
            else {
                packet_len = 0;
            }
        }

        if  packet_len > 0 {
            self.received_packet_count += 1;
        }

        self.cs.set_high().map_err(Error::Pin)?;
        Ok(packet_len)
    }
}


