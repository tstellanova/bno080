

use embedded_hal;

use super::{SensorInterface};
use crate::interface::{PACKET_HEADER_LENGTH, SensorCommon, DebugFunc};
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
    debug_func: Option<DebugFunc>,
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
            received_packet_count: 0,
            debug_func: None,
        }
    }

    fn sensor_ready(&self) ->  bool  {
        self.hintn.is_low().unwrap_or(false)
    }

    /// Wait for sensor to be ready.
    /// After reset this can take around 120 ms
    fn wait_for_sensor_ready(&mut self, delay_source: &mut impl DelayMs<u8>) -> bool {
        self.wait_for_data_available(250, delay_source)
    }

/// read the body ("cargo") of a packet,
/// return the total packet length read
    fn read_packet_cargo(&mut self, recv_buf: &mut [u8]) -> usize  {
        let mut packet_len = SensorCommon::parse_packet_header(&recv_buf[..PACKET_HEADER_LENGTH]);
        // now get the body
        if (packet_len > PACKET_HEADER_LENGTH) && (packet_len < recv_buf.len()) {
            //exchange 0xFF bytes for whatever the sensor is sending
            for w in recv_buf[PACKET_HEADER_LENGTH..packet_len].iter_mut() {
                *w = 0xFF;
            }
            let rc = self.spi.transfer( &mut recv_buf[PACKET_HEADER_LENGTH..packet_len]);
            if rc.is_err() {
                packet_len = 0;
            }
        }
        else {
            packet_len = 0;
        }

        packet_len
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
        // Deselect sensor
        self.csn.set_high().map_err(Error::Pin)?;
        // Set WAK / PS0 to high before we reset, in order to select SPI (vs UART) mode
        self.waken.set_high().map_err(Error::Pin)?;
        // begin reset cycle
        self.reset.set_low().map_err(Error::Pin)?;
        delay_source.delay_ms(5);
        self.reset.set_high().map_err(Error::Pin)?;

        self.wait_for_sensor_ready(delay_source);

        Ok(())
        //Err(Error::SensorUnresponsive)
    }

    /// return true if the sensor is ready to provide data
    fn wait_for_data_available(&mut self, max_ms: u8, delay_source: &mut impl DelayMs<u8>) -> bool {
        for _i in 0..max_ms {
            if self.sensor_ready() {
                return true;
            }
            delay_source.delay_ms(1);
        }
        false
    }

    fn send_and_receive_packet(&mut self, send_buf: &[u8], recv_buf: &mut [u8], delay_source: &mut impl DelayMs<u8>)
        -> Result<usize,  Self::SensorError> {

        //ensure that the first header bytes are zeroed since we're not sending any data
        for i in recv_buf[..PACKET_HEADER_LENGTH].iter_mut() {
            *i = 0;
        }

        //grab the sensor
        self.csn.set_low().map_err(Error::Pin)?;
        let rc = self.spi.write(&send_buf).map_err(Error::Comm);

        if rc.is_err() {
            //release the sensor
            self.csn.set_high().map_err(Error::Pin)?;
            return Err(rc.unwrap_err());
        }

        
        if !self.wait_for_data_available(50, delay_source) {
            self.csn.set_high().map_err(Error::Pin)?;
            return Ok(0)
        }

        // get just the header
        let rc = self.spi.transfer(&mut recv_buf[..PACKET_HEADER_LENGTH]).map_err(Error::Comm);
        if rc.is_err() {
            //release the sensor
            self.csn.set_high().map_err(Error::Pin)?;
            return Err(rc.unwrap_err());
        }

        let packet_len = self.read_packet_cargo(recv_buf);

        //release the sensor
        self.csn.set_high().map_err(Error::Pin)?;

        if  packet_len > 0 {
            self.received_packet_count += 1;
        }

        Ok(packet_len)

    }


    fn enable_debugging(&mut self, dbf: DebugFunc) {
        self.debug_func = Some(dbf);
    }


    fn write_packet(&mut self, packet: &[u8]) -> Result<(), Self::SensorError> {
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

        //ensure that the first header bytes are zeroed since we're not sending any data
        for i in recv_buf[..PACKET_HEADER_LENGTH].iter_mut() {
            *i = 0;
        }

        // grab this sensor
        self.csn.set_low().map_err(Error::Pin)?;
        // get just the header
        let rc = self.spi.transfer(&mut recv_buf[..PACKET_HEADER_LENGTH]).map_err(Error::Comm);


        if rc.is_err() {
            //release the sensor
            self.csn.set_high().map_err(Error::Pin)?;
            return Err(rc.unwrap_err());
        }

        let packet_len = self.read_packet_cargo(recv_buf);

        //release the sensor
        self.csn.set_high().map_err(Error::Pin)?;

        if  packet_len > 0 {
            self.received_packet_count += 1;
        }

        Ok(packet_len)
    }
}


