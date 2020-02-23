

use embedded_hal;

use super::{SensorInterface};
use crate::interface::{PACKET_HEADER_LENGTH, SensorCommon};
use embedded_hal::digital::v2::{OutputPin, InputPin};
use embedded_hal::blocking::delay::DelayMs;

use crate::Error;

/// This combines the SPI peripheral and
/// associated control pins such as:
/// - CSN : Chip Select (aka SS or Slave Select)
/// - HINTN: Hardware Interrupt. Sensor uses this to indicate it had data available for read
/// - WAK: Wake pin.  Master asserts this to choose SPI mode, then deasserts to wake up the sensor.
pub struct SpiInterface<SPI, CSN, IN, WAK, RSTN> {
    spi: SPI,
    csn: CSN,
    hintn: IN,
    waken: WAK,
    reset: RSTN,
    received_packet_count: usize,
}

impl<SPI, CSN, IN, WAK, RSTN, CommE, PinE> SpiInterface<SPI, CSN, IN, WAK, RSTN>
    where
        SPI: embedded_hal::blocking::spi::Write<u8, Error = CommE> +
        embedded_hal::blocking::spi::Transfer<u8, Error = CommE>,
        CSN: OutputPin<Error = PinE>,
        IN: InputPin<Error = PinE>,
        WAK: OutputPin<Error = PinE>,
        RSTN: OutputPin<Error = PinE>,
{
    pub fn new(spi: SPI, csn: CSN, hintn: IN, waken: WAK, reset: RSTN) -> Self {
        Self {
            spi,
            csn,
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
        self.wait_for_data_available(250, delay_source)
    }
}

impl<SPI, CSN, IN, WAK, RS, CommE, PinE> SensorInterface for SpiInterface<SPI, CSN, IN, WAK, RS>
    where
        SPI: embedded_hal::blocking::spi::Write<u8, Error = CommE> +
        embedded_hal::blocking::spi::Transfer<u8, Error = CommE>,
        CSN: OutputPin<Error = PinE>,
        IN: InputPin<Error = PinE>,
        WAK: OutputPin<Error = PinE>,
        RS: OutputPin<Error = PinE>,
{
    type SensorError = Error<CommE, PinE>;

    fn setup(&mut self, delay_source: &mut impl DelayMs<u8>) -> Result<(), Self::SensorError> {
        self.csn.set_high().map_err(Error::Pin)?;
        self.waken.set_high().map_err(Error::Pin)?;
        // reset cycle
        self.reset.set_low().map_err(Error::Pin)?;
        delay_source.delay_ms(5);
        self.reset.set_high().map_err(Error::Pin)?;

        if self.wait_for_ready(delay_source) {
            // TODO do we need to ever drop this low? or assume the sensor stays awake?
            //self.waken.set_low().map_err(Error::Pin)?;
            return Ok(())
        }

        Err(Error::SensorUnresponsive)
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
        self.csn.set_low().map_err(Error::Pin)?;
        let rc = self.spi.write(&packet).map_err(Error::Comm);
        self.csn.set_high().map_err(Error::Pin)?;

        if rc.is_err() {
            return Err(rc.unwrap_err());
        }


        Ok(())
    }

    fn read_packet(&mut self, recv_buf: &mut [u8]) -> Result<usize, Self::SensorError> {
        //detect whether the sensor is ready to send data
        if !self.sensor_ready() {
            return Ok(0)
        }

        // grab this sensor
        self.csn.set_low().map_err(Error::Pin)?;

        //ensure that the first header bytes are zeroed since we're not sending any data
        for i in recv_buf[..PACKET_HEADER_LENGTH].iter_mut() {
            *i = 0;
        }

        // get just the header
        self.spi.transfer(&mut recv_buf[..PACKET_HEADER_LENGTH]).map_err(Error::Comm)?;
        let mut packet_len = SensorCommon::parse_packet_header(&recv_buf[..PACKET_HEADER_LENGTH]);
        // now get the body
        if packet_len > PACKET_HEADER_LENGTH {
            if packet_len < recv_buf.len() {
                //exchnage 0xFF bytes for whatever the sensor is sending
                for w in recv_buf[PACKET_HEADER_LENGTH..packet_len].iter_mut() {
                    *w = 0xFF;
                }
                self.spi.transfer( &mut recv_buf[PACKET_HEADER_LENGTH..packet_len]).map_err(Error::Comm)?;
            }
            else {
                packet_len = 0;
            }
        }

        if  packet_len > 0 {
            self.received_packet_count += 1;
        }

        // release the sensor
        self.csn.set_high().map_err(Error::Pin)?;
        Ok(packet_len)
    }
}


