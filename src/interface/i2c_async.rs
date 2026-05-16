use super::i2c_common::I2cCommon;
use super::{SensorInterfaceAsync, PACKET_HEADER_LENGTH};
use crate::Error;

use embedded_hal_async::delay::DelayNs;

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

pub use super::i2c_common::{ALTERNATE_ADDRESS, DEFAULT_ADDRESS};

pub struct I2cInterfaceAsync<I2C> {
    /// i2c port
    i2c_port: I2C,
    common: I2cCommon,
}

impl<I2C, CommE> I2cInterfaceAsync<I2C>
where
    I2C: embedded_hal_async::i2c::I2c<Error = CommE>,
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

impl<I2C, CommE> SensorInterfaceAsync for I2cInterfaceAsync<I2C>
where
    I2C: embedded_hal_async::i2c::I2c<Error = CommE>,
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
