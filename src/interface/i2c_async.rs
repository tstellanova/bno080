use super::{SensorCommon, SensorInterfaceAsync, PACKET_HEADER_LENGTH};
use crate::Error;

use embedded_hal_async::delay::DelayNs;

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

/// the i2c address normally used by BNO080
pub const DEFAULT_ADDRESS: u8 = 0x4A;
/// alternate i2c address for BNO080
pub const ALTERNATE_ADDRESS: u8 = 0x4B;

/// Length of our receive buffer:
/// Note that this likely needs to be < 256 to accommodate underlying HAL
const SEG_RECV_BUF_LEN: usize = 240;
const MAX_SEGMENT_READ: usize = SEG_RECV_BUF_LEN;

pub struct I2cInterfaceAsync<I2C> {
    /// i2c port
    i2c_port: I2C,
    /// address for i2c communications with the sensor hub
    address: u8,
    /// buffer for receiving segments of packets from the sensor hub
    seg_recv_buf: [u8; SEG_RECV_BUF_LEN],

    /// number of packets received
    received_packet_count: usize,
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
            address: addr,
            seg_recv_buf: [0; SEG_RECV_BUF_LEN],
            received_packet_count: 0,
        }
    }

    pub fn free(self) -> I2C {
        self.i2c_port
    }

    async fn read_packet_header(&mut self) -> Result<(), Error<CommE, ()>> {
        self.zero_recv_packet_header();
        self.i2c_port
            .read(self.address, &mut self.seg_recv_buf[..PACKET_HEADER_LENGTH])
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
        let mut remaining_body_len: usize =
            total_packet_len - PACKET_HEADER_LENGTH;
        let mut already_read_len: usize = 0;

        // zero packet header receive buffer
        for byte in &mut packet_recv_buf[..PACKET_HEADER_LENGTH] {
            *byte = 0;
        }

        // #[cfg(feature = "rttdebug")]
        // rprintln!("r.t {}", total_packet_len);

        if total_packet_len < MAX_SEGMENT_READ {
            //read directly into the provided receive buffer
            if total_packet_len > 0 {
                self.i2c_port
                    .read(
                        self.address,
                        &mut packet_recv_buf[..total_packet_len],
                    )
                    .await
                    .map_err(Error::Comm)?;
                already_read_len = total_packet_len;
            }
        } else {
            while remaining_body_len > 0 {
                let whole_segment_length =
                    remaining_body_len + PACKET_HEADER_LENGTH;
                let segment_read_len =
                    if whole_segment_length > MAX_SEGMENT_READ {
                        MAX_SEGMENT_READ
                    } else {
                        whole_segment_length
                    };
                // #[cfg(feature = "rttdebug")]
                // rprintln!("r.s {:x} {}", self.address, segment_read_len);

                self.zero_recv_packet_header();
                self.i2c_port
                    .read(
                        self.address,
                        &mut self.seg_recv_buf[..segment_read_len],
                    )
                    .await
                    .map_err(Error::Comm)?;

                let promised_packet_len = SensorCommon::parse_packet_header(
                    &self.seg_recv_buf[..PACKET_HEADER_LENGTH],
                );
                if promised_packet_len <= PACKET_HEADER_LENGTH {
                    #[cfg(feature = "rttdebug")]
                    rprintln!("WTFFF {}", promised_packet_len);
                    return Ok(0);
                }

                //if we've never read any segments, transcribe the first packet header;
                //otherwise, just transcribe the segment body (no header)
                let transcribe_start_idx = if already_read_len > 0 {
                    PACKET_HEADER_LENGTH
                } else {
                    0
                };
                let transcribe_len = if already_read_len > 0 {
                    segment_read_len - PACKET_HEADER_LENGTH
                } else {
                    segment_read_len
                };
                packet_recv_buf
                    [already_read_len..already_read_len + transcribe_len]
                    .copy_from_slice(
                        &self.seg_recv_buf[transcribe_start_idx
                            ..transcribe_start_idx + transcribe_len],
                    );
                already_read_len += transcribe_len;

                let body_read_len = segment_read_len - PACKET_HEADER_LENGTH;
                remaining_body_len -= body_read_len;
            }
        }

        Ok(already_read_len)
    }

    fn zero_recv_packet_header(&mut self) {
        Self::zero_buffer(&mut self.seg_recv_buf[..PACKET_HEADER_LENGTH]);
    }

    fn zero_buffer(buf: &mut [u8]) {
        for byte in buf {
            *byte = 0;
        }
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
        rprintln!("w {:x} {}", self.address, packet.len());
        self.i2c_port
            .write(self.address, packet)
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
        let packet_len = SensorCommon::parse_packet_header(
            &self.seg_recv_buf[..PACKET_HEADER_LENGTH],
        );

        // if packet_len == 0 {
        //     #[cfg(feature = "rttdebug")]
        //     rprintln!("eh {:x?}", &self.seg_recv_buf[..PACKET_HEADER_LENGTH]);
        // }

        let received_len = if packet_len > PACKET_HEADER_LENGTH {
            self.read_sized_packet(packet_len, recv_buf).await?
        } else {
            packet_len
        };

        if packet_len > 0 {
            self.received_packet_count += 1;
            //let _ = SensorCommon::parse_packet_header(&recv_buf[..packet_len]);
        }

        Ok(received_len)
    }

    async fn send_and_receive_packet(
        &mut self,
        send_buf: &[u8],
        recv_buf: &mut [u8],
    ) -> Result<usize, Self::SensorError> {
        // Cannot use write_read with bno080,
        // because it does not support repeated start with i2c.

        self.i2c_port
            .write(self.address, send_buf)
            .await
            .map_err(Error::Comm)?;

        self.zero_recv_packet_header();
        //stall before attempted read?
        Self::zero_buffer(recv_buf);

        self.i2c_port
            .read(self.address, &mut self.seg_recv_buf[..PACKET_HEADER_LENGTH])
            .await
            .map_err(Error::Comm)?;

        let packet_len = SensorCommon::parse_packet_header(
            &self.seg_recv_buf[..PACKET_HEADER_LENGTH],
        );

        let received_len = if packet_len > PACKET_HEADER_LENGTH {
            self.read_sized_packet(packet_len, recv_buf).await?
        } else {
            packet_len
        };
        if packet_len > 0 {
            self.received_packet_count += 1;
        }

        Ok(received_len)
    }
}
