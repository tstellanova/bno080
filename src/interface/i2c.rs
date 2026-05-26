use super::i2c_common::I2cCommon;
use super::{SensorInterface, PACKET_HEADER_LENGTH};
use crate::Error;

#[cfg(feature = "async")]
use {
    embedded_hal_async::delay::DelayNs,
    embedded_hal_async::i2c::I2c,
};

#[cfg(not(feature = "async"))]
use {
    embedded_hal::delay::DelayNs,
    embedded_hal::i2c::I2c,
};

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

pub use super::i2c_common::{ALTERNATE_ADDRESS, DEFAULT_ADDRESS};

pub struct I2cInterface<I2C> {
    /// i2c port
    i2c_port: I2C,
    common: I2cCommon,
}

#[maybe_async::maybe_async]
impl<I2C, CommE> I2cInterface<I2C>
where
    I2C: I2c<Error = CommE>,
{
    pub fn default(i2c: I2C) -> Self {
        Self::new(i2c, DEFAULT_ADDRESS)
    }

    pub fn alternate(i2c: I2C) -> Self {
        Self::new(i2c, ALTERNATE_ADDRESS)
    }

    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c_port: i2c,
            common: I2cCommon::new(addr),
        }
    }

    pub fn free(self) -> I2C {
        self.i2c_port
    }

    async fn read_packet_header(&mut self) -> Result<(), Error<CommE, ()>> {
        self.common.zero_recv_packet_header();
        let address = self.common.address();
        self.i2c_port
            .read(
                address,
                &mut self.common.seg_recv_buf_mut()[..PACKET_HEADER_LENGTH],
            )
            .await
            .map_err(Error::Comm)?;

        Ok(())
    }

    /// Read the remainder of the packet after the packet header, if any
    async fn read_sized_packet(
        &mut self,
        total_packet_len: usize,
        packet_recv_buf: &mut [u8],
    ) -> Result<usize, Error<CommE, ()>> {
        let mut sized_read =
            I2cCommon::sized_read(total_packet_len, packet_recv_buf);

        // #[cfg(feature = "rttdebug")]
        // rprintln!("r.t {}", total_packet_len);

        if let Some(read_len) = sized_read.direct_read_len() {
            let address = self.common.address();
            self.i2c_port
                .read(address, &mut packet_recv_buf[..read_len])
                .await
                .map_err(Error::Comm)?;
            return Ok(sized_read.finish_direct_read(read_len));
        }

        while sized_read.has_remaining_segments() {
            let segment_read_len = sized_read.next_segment_read_len();
            // #[cfg(feature = "rttdebug")]
            // rprintln!("r.s {:x} {}", self.common.address(), segment_read_len);

            self.common.zero_recv_packet_header();
            let address = self.common.address();
            self.i2c_port
                .read(
                    address,
                    &mut self.common.seg_recv_buf_mut()[..segment_read_len],
                )
                .await
                .map_err(Error::Comm)?;

            let promised_packet_len = self.common.packet_len_from_header();
            if promised_packet_len <= PACKET_HEADER_LENGTH {
                #[cfg(feature = "rttdebug")]
                rprintln!("WTFFF {}", promised_packet_len);
                return Ok(0);
            }

            sized_read.transcribe_segment(
                segment_read_len,
                self.common.seg_recv_buf(),
                packet_recv_buf,
            );
        }

        Ok(sized_read.already_read_len())
    }
}

#[maybe_async::maybe_async(AFIT)]
impl<I2C, CommE> SensorInterface for I2cInterface<I2C>
where
    I2C: I2c<Error = CommE>,
{
    type SensorError = Error<CommE, ()>;

    fn requires_soft_reset(&self) -> bool {
        true
    }

    async fn setup(
        &mut self,
        delay_source: &mut impl DelayNs,
    ) -> Result<(), Self::SensorError> {
        // #[cfg(feature = "rttdebug")]
        // rprintln!("i2c setup");
        delay_source.delay_ms(5).await;
        Ok(())
    }

    async fn write_packet(
        &mut self,
        packet: &[u8],
    ) -> Result<(), Self::SensorError> {
        #[cfg(feature = "rttdebug")]
        rprintln!("w {:x} {}", self.common.address(), packet.len());
        let address = self.common.address();
        self.i2c_port
            .write(address, packet)
            .await
            .map_err(Error::Comm)?;
        Ok(())
    }

    async fn read_with_timeout(
        &mut self,
        recv_buf: &mut [u8],
        delay_source: &mut impl DelayNs,
        max_ms: u8,
    ) -> Result<usize, Self::SensorError> {
        let mut total_delay: u8 = 0;
        while total_delay < max_ms {
            match self.read_packet(recv_buf).await {
                Ok(read_size) => {
                    if 0 == read_size {
                        // no data available yet...wait a while longer
                        delay_source.delay_ms(1).await;
                        total_delay += 1;
                    } else {
                        return Ok(read_size);
                    }
                }
                Err(e) => return Err(e),
            }
        }

        Ok(0)
    }

    /// Read one packet into the receive buffer
    async fn read_packet(
        &mut self,
        recv_buf: &mut [u8],
    ) -> Result<usize, Self::SensorError> {
        // #[cfg(feature = "rttdebug")]
        // rprintln!("rpkt");

        self.read_packet_header().await?;
        let packet_len = self.common.packet_len_from_header();

        // if packet_len == 0 {
        //     #[cfg(feature = "rttdebug")]
        //     rprintln!("eh {:x?}", &self.common.seg_recv_buf()[..PACKET_HEADER_LENGTH]);
        // }

        let received_len = if packet_len > PACKET_HEADER_LENGTH {
            self.read_sized_packet(packet_len, recv_buf).await?
        } else {
            packet_len
        };

        self.common.record_received_packet(packet_len);

        Ok(received_len)
    }

    async fn send_and_receive_packet(
        &mut self,
        send_buf: &[u8],
        recv_buf: &mut [u8],
    ) -> Result<usize, Self::SensorError> {
        // Cannot use write_read with bno080,
        // because it does not support repeated start with i2c.

        let address = self.common.address();
        self.i2c_port
            .write(address, send_buf)
            .await
            .map_err(Error::Comm)?;

        self.common.zero_recv_packet_header();
        //stall before attempted read?
        I2cCommon::zero_buffer(recv_buf);

        self.i2c_port
            .read(
                address,
                &mut self.common.seg_recv_buf_mut()[..PACKET_HEADER_LENGTH],
            )
            .await
            .map_err(Error::Comm)?;

        let packet_len = self.common.packet_len_from_header();

        let received_len = if packet_len > PACKET_HEADER_LENGTH {
            self.read_sized_packet(packet_len, recv_buf).await?
        } else {
            packet_len
        };
        self.common.record_received_packet(packet_len);

        Ok(received_len)
    }
}

#[cfg(test)]
mod tests {
    // use crate::interface::i2c::DEFAULT_ADDRESS;
    // use crate::interface::mock_i2c_port::FakeI2cPort;
    // use crate::interface::I2cInterface;
    // use crate::wrapper::BNO080;

    // #[test]
    // fn test_multi_segment_receive_packet() {
    //     let mut mock_i2c_port = FakeI2cPort::new();

    //     let packet = ADVERTISING_PACKET_FULL;
    //     mock_i2c_port.add_available_packet(&packet);

    //     let mut shub = BNO080::new_with_interface(I2cInterface::new(
    //         mock_i2c_port,
    //         DEFAULT_ADDRESS,
    //     ));
    //     let rc = shub.receive_packet();

    //     assert!(rc.is_ok());
    //     let next_packet_size = rc.unwrap_or(0);
    //     assert_eq!(next_packet_size, packet.len(), "wrong length");
    // }

    //TODO test failing due to bug in mock_i2c_port
    // #[test]
    // fn test_receive_under() {
    //     let mut mock_i2c_port = FakeI2cPort::new();
    //
    //     let packet: [u8; 3] = [0; 3];
    //     mock_i2c_port.add_available_packet(&packet);
    //
    //     let mut shub = BNO080::new_with_interface(
    //         I2cInterface::new(mock_i2c_port, DEFAULT_ADDRESS));
    //     let rc = shub.receive_packet();
    //
    //     assert!(rc.is_err());
    // }

    // Actual advertising packet received from sensor:
    // pub const ADVERTISING_PACKET_FULL: [u8; 276] = [
    //     0x14, 0x81, 0x00, 0x01, 0x00, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x80,
    //     0x06, 0x31, 0x2e, 0x30, 0x2e, 0x30, 0x00, 0x02, 0x02, 0x00, 0x01, 0x03,
    //     0x02, 0xff, 0x7f, 0x04, 0x02, 0x00, 0x01, 0x05, 0x02, 0xff, 0x7f, 0x08,
    //     0x05, 0x53, 0x48, 0x54, 0x50, 0x00, 0x06, 0x01, 0x00, 0x09, 0x08, 0x63,
    //     0x6f, 0x6e, 0x74, 0x72, 0x6f, 0x6c, 0x00, 0x01, 0x04, 0x01, 0x00, 0x00,
    //     0x00, 0x08, 0x0b, 0x65, 0x78, 0x65, 0x63, 0x75, 0x74, 0x61, 0x62, 0x6c,
    //     0x65, 0x00, 0x06, 0x01, 0x01, 0x09, 0x07, 0x64, 0x65, 0x76, 0x69, 0x63,
    //     0x65, 0x00, 0x01, 0x04, 0x02, 0x00, 0x00, 0x00, 0x08, 0x0a, 0x73, 0x65,
    //     0x6e, 0x73, 0x6f, 0x72, 0x68, 0x75, 0x62, 0x00, 0x06, 0x01, 0x02, 0x09,
    //     0x08, 0x63, 0x6f, 0x6e, 0x74, 0x72, 0x6f, 0x6c, 0x00, 0x06, 0x01, 0x03,
    //     0x09, 0x0c, 0x69, 0x6e, 0x70, 0x75, 0x74, 0x4e, 0x6f, 0x72, 0x6d, 0x61,
    //     0x6c, 0x00, 0x07, 0x01, 0x04, 0x09, 0x0a, 0x69, 0x6e, 0x70, 0x75, 0x74,
    //     0x57, 0x61, 0x6b, 0x65, 0x00, 0x06, 0x01, 0x05, 0x09, 0x0c, 0x69, 0x6e,
    //     0x70, 0x75, 0x74, 0x47, 0x79, 0x72, 0x6f, 0x52, 0x76, 0x00, 0x80, 0x06,
    //     0x31, 0x2e, 0x31, 0x2e, 0x30, 0x00, 0x81, 0x64, 0xf8, 0x10, 0xf5, 0x04,
    //     0xf3, 0x10, 0xf1, 0x10, 0xfb, 0x05, 0xfa, 0x05, 0xfc, 0x11, 0xef, 0x02,
    //     0x01, 0x0a, 0x02, 0x0a, 0x03, 0x0a, 0x04, 0x0a, 0x05, 0x0e, 0x06, 0x0a,
    //     0x07, 0x10, 0x08, 0x0c, 0x09, 0x0e, 0x0a, 0x08, 0x0b, 0x08, 0x0c, 0x06,
    //     0x0d, 0x06, 0x0e, 0x06, 0x0f, 0x10, 0x10, 0x05, 0x11, 0x0c, 0x12, 0x06,
    //     0x13, 0x06, 0x14, 0x10, 0x15, 0x10, 0x16, 0x10, 0x17, 0x00, 0x18, 0x08,
    //     0x19, 0x06, 0x1a, 0x00, 0x1b, 0x00, 0x1c, 0x06, 0x1d, 0x00, 0x1e, 0x10,
    //     0x1f, 0x00, 0x20, 0x00, 0x21, 0x00, 0x22, 0x00, 0x23, 0x00, 0x24, 0x00,
    //     0x25, 0x00, 0x26, 0x00, 0x27, 0x00, 0x28, 0x0e, 0x29, 0x0c, 0x2a, 0x0e,
    // ];
}
