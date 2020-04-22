use embedded_hal;

use super::SensorInterface;
use crate::interface::{SensorCommon, PACKET_HEADER_LENGTH};
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::{InputPin, OutputPin};

use crate::Error;
use crate::Error::SensorUnresponsive;

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

/// Encapsulates all the lines required to operate this sensor
/// - SCK: clock line from master
/// - MISO: Data input from the sensor to the master
/// - MOSI: Output from the master to the sensor
/// - CSN: chip select line that selects the device on the shared SPI bus
/// - HINTN: Hardware Interrupt. Sensor uses this to indicate it had data available for read
/// - WAK: Wake pin.  Master asserts this to choose SPI mode, then deasserts to wake up the sensor.
/// - RSTN: Reset the device
pub struct SpiControlLines<SPI, CSN, IN, WAK, RSTN> {
    pub spi: SPI,
    pub csn: CSN,
    pub hintn: IN,
    pub waken: WAK,
    pub reset: RSTN,
}

/// This combines the SPI peripheral and associated control pins
///
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
    SPI: embedded_hal::blocking::spi::Write<u8, Error = CommE>
        + embedded_hal::blocking::spi::Transfer<u8, Error = CommE>,
    CSN: OutputPin<Error = PinE>,
    IN: InputPin<Error = PinE>,
    WAK: OutputPin<Error = PinE>,
    RSTN: OutputPin<Error = PinE>,
    CommE: core::fmt::Debug,
    PinE: core::fmt::Debug,
{
    pub fn new(lines: SpiControlLines<SPI, CSN, IN, WAK, RSTN>) -> Self {
        //TODO allow some lines to be optional, such as WAK
        Self {
            spi: lines.spi,
            csn: lines.csn,
            hintn: lines.hintn,
            waken: lines.waken,
            reset: lines.reset,
            received_packet_count: 0,
        }
    }

    /// Is the sensor indicating it has data available
    /// "In SPI and I2C mode the HOST_INTN signal is used by the BNO080 to
    /// indicate to the application processor that the BNO080 needs attention."
    fn hintn_signaled(&self) -> bool {
        self.hintn.is_low().unwrap_or(false)
    }

    /// Wait for sensor to be ready.
    /// After reset this can take around 120 ms
    /// Return true if the sensor is awake, false if it doesn't wake up
    /// `max_ms` maximum milliseconds to await for HINTN change
    fn wait_for_sensor_awake(
        &mut self,
        delay_source: &mut impl DelayMs<u8>,
        max_ms: u8,
    ) -> bool {
        for _ in 0..max_ms {
            if self.hintn_signaled() {
                return true;
            }
            delay_source.delay_ms(1);
        }

        false
    }

    /// block on HINTN for n cycles
    fn block_on_hintn(&mut self, max_cycles: usize) -> bool {
        for _ in 0..max_cycles {
            if self.hintn_signaled() {
                return true;
            }
        }
        #[cfg(feature = "rttdebug")]
        rprintln!("no hintn??");

        false
    }

    /// read the body ("cargo" or "payload") of a packet,
    /// return the total packet length read
    fn read_packet_cargo(&mut self, recv_buf: &mut [u8]) -> usize {
        let mut packet_len = SensorCommon::parse_packet_header(
            &recv_buf[..PACKET_HEADER_LENGTH],
        );
        // now get the body
        if (packet_len > PACKET_HEADER_LENGTH) && (packet_len < recv_buf.len())
        {
            //exchange 0xFF bytes for whatever the sensor is sending
            for w in recv_buf[PACKET_HEADER_LENGTH..packet_len].iter_mut() {
                *w = 0xFF;
            }
            let rc = self
                .spi
                .transfer(&mut recv_buf[PACKET_HEADER_LENGTH..packet_len]);
            if rc.is_err() {
                packet_len = 0;
            }
        } else {
            packet_len = 0;
        }

        packet_len
    }
}

impl<SPI, CSN, IN, WAK, RS, CommE, PinE> SensorInterface
    for SpiInterface<SPI, CSN, IN, WAK, RS>
where
    SPI: embedded_hal::blocking::spi::Write<u8, Error = CommE>
        + embedded_hal::blocking::spi::Transfer<u8, Error = CommE>,
    CSN: OutputPin<Error = PinE>,
    IN: InputPin<Error = PinE>,
    WAK: OutputPin<Error = PinE>,
    RS: OutputPin<Error = PinE>,
    CommE: core::fmt::Debug,
    PinE: core::fmt::Debug,
{
    type SensorError = Error<CommE, PinE>;

    fn requires_soft_reset(&self) -> bool {
        false
    }

    fn setup(
        &mut self,
        delay_source: &mut impl DelayMs<u8>,
    ) -> Result<(), Self::SensorError> {
        // Deselect sensor
        self.csn.set_high().map_err(Error::Pin)?;
        // Set WAK / PS0 to high before we reset, in order to select SPI (vs UART) mode
        self.waken.set_high().map_err(Error::Pin)?;
        // should already be high by default, but just in case...
        self.reset.set_high().map_err(Error::Pin)?;

        // #[cfg(feature = "rttdebug")]
        // rprintln!("reset cycle... ");
        // reset cycle

        self.reset.set_low().map_err(Error::Pin)?;
        delay_source.delay_ms(2);
        self.reset.set_high().map_err(Error::Pin)?;

        // wait for sensor to set hintn pin after reset
        let ready = self.wait_for_sensor_awake(delay_source, 200u8);
        if !ready {
            #[cfg(feature = "rttdebug")]
            rprintln!("sensor not ready");
            return Err(SensorUnresponsive);
        }

        Ok(())
    }

    fn send_and_receive_packet(
        &mut self,
        send_buf: &[u8],
        recv_buf: &mut [u8],
    ) -> Result<usize, Self::SensorError> {
        // select the sensor
        self.csn.set_low().map_err(Error::Pin)?;
        let rc = self.spi.write(&send_buf).map_err(Error::Comm);
        self.csn.set_high().map_err(Error::Pin)?;
        if rc.is_err() {
            //release the sensor
            return Err(rc.unwrap_err());
        }
        #[cfg(feature = "rttdebug")]
        rprintln!("sent {}", send_buf.len());

        //zero the receive buffer
        for i in recv_buf[..PACKET_HEADER_LENGTH].iter_mut() {
            *i = 0;
        }

        if !self.block_on_hintn(1000) {
            //no packet to be read
            #[cfg(feature = "rttdebug")]
            rprintln!("no packet to read?");
            return Ok(0);
        }

        self.csn.set_low().map_err(Error::Pin)?;
        // get just the header
        let rc = self
            .spi
            .transfer(&mut recv_buf[..PACKET_HEADER_LENGTH])
            .map_err(Error::Comm);
        if rc.is_err() {
            //release the sensor
            #[cfg(feature = "rttdebug")]
            rprintln!("transfer err: {:?}", rc);
            self.csn.set_high().map_err(Error::Pin)?;
            return Err(rc.unwrap_err());
        }

        let packet_len = self.read_packet_cargo(recv_buf);

        //release the sensor
        self.csn.set_high().map_err(Error::Pin)?;

        if packet_len > 0 {
            self.received_packet_count += 1;
        }

        Ok(packet_len)
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

    /// Read a complete packet from the sensor
    fn read_packet(
        &mut self,
        recv_buf: &mut [u8],
    ) -> Result<usize, Self::SensorError> {
        // Note: HINTN cannot always be used to detect data ready.
        // As soon as host selects CSN, HINTN resets

        //Zero the header bytes are zeroed since we're not sending any data
        for i in recv_buf[..PACKET_HEADER_LENGTH].iter_mut() {
            *i = 0;
        }

        // grab this sensor
        self.csn.set_low().map_err(Error::Pin)?;
        // get just the header
        let rc = self
            .spi
            .transfer(&mut recv_buf[..PACKET_HEADER_LENGTH])
            .map_err(Error::Comm);

        if rc.is_err() {
            //release the sensor
            self.csn.set_high().map_err(Error::Pin)?;
            return Err(rc.unwrap_err());
        }

        let packet_len = self.read_packet_cargo(recv_buf);

        //release the sensor
        self.csn.set_high().map_err(Error::Pin)?;

        if packet_len > 0 {
            self.received_packet_count += 1;
        }

        Ok(packet_len)
    }

    fn read_with_timeout(
        &mut self,
        recv_buf: &mut [u8],
        delay_source: &mut impl DelayMs<u8>,
        max_ms: u8,
    ) -> Result<usize, Self::SensorError> {
        if self.wait_for_sensor_awake(delay_source, max_ms) {
            return self.read_packet(recv_buf);
        }
        Ok(0)
    }
}
