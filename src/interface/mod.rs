pub mod i2c;
pub mod spi;

#[cfg(test)]
pub mod mock_i2c_port;

use core::ops::Shl;

use embedded_hal::blocking::delay::DelayMs;

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

/// A method of communicating with the sensor
pub trait SensorInterface {
    /// Interface error type
    type SensorError;

    /// give the sensor interface a chance to set up
    fn setup(
        &mut self,
        delay_source: &mut impl DelayMs<u8>,
    ) -> Result<(), Self::SensorError>;

    /// Write the whole packet provided
    fn write_packet(&mut self, packet: &[u8]) -> Result<(), Self::SensorError>;

    /// Read the next packet from the sensor
    /// Returns the size of the packet read (up to the size of the slice provided)
    fn read_packet(
        &mut self,
        recv_buf: &mut [u8],
    ) -> Result<usize, Self::SensorError>;

    /// Wait for sensor to indicate it has data available before reading
    /// - `max_ms` maximum number of milliseconds to wait for data
    fn read_with_timeout(
        &mut self,
        recv_buf: &mut [u8],
        delay_source: &mut impl DelayMs<u8>,
        max_ms: u8,
    ) -> Result<usize, Self::SensorError>;

    /// Send a packet and receive the response immediately
    fn send_and_receive_packet(
        &mut self,
        send_buf: &[u8],
        recv_buf: &mut [u8],
    ) -> Result<usize, Self::SensorError>;

    /// Does this interface require a soft reset after init?
    fn requires_soft_reset(&self) -> bool;
}

pub use self::i2c::I2cInterface;
pub use self::spi::SpiInterface;

pub(crate) const PACKET_HEADER_LENGTH: usize = 4;
pub(crate) const MAX_CARGO_DATA_LENGTH: usize = 32766 - PACKET_HEADER_LENGTH;

struct SensorCommon {}

impl SensorCommon {
    fn parse_packet_header(packet: &[u8]) -> usize {
        const CONTINUATION_FLAG_MASK: u16 = 0x80;
        const CONTINUATION_FLAG_CLEAR: u16 = !(CONTINUATION_FLAG_MASK);
        if packet.len() < PACKET_HEADER_LENGTH {
            return 0;
        }
        //Bits 14:0 are used to indicate the total number of bytes in the body plus header
        //maximum packet length is ... PACKET_HEADER_LENGTH
        let raw_pack_len: u16 = (packet[0] as u16)
            + ((packet[1] as u16) & CONTINUATION_FLAG_CLEAR).shl(8);

        let mut packet_len: usize = raw_pack_len as usize;
        if packet_len > MAX_CARGO_DATA_LENGTH {
            // we sometimes get garbage packets of [0xFF, 0xFF, 0xFF, 0xFF]
            packet_len = 0; //PACKET_HEADER_LENGTH;
        }

        if 0 == packet_len && 0 != raw_pack_len {
            #[cfg(feature = "rttdebug")]
            rprintln!(
                "pph: {:?} {} -> {}",
                &packet[..PACKET_HEADER_LENGTH],
                raw_pack_len,
                packet_len
            );
        } else {
            // hprintln!("pph: {:?} {} ", &packet[..PACKET_HEADER_LENGTH], packet_len).unwrap();
        }

        packet_len
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::ops::Shr;

    #[test]
    fn test_parse_packet_header() {
        let short_packet: [u8; 2] = [13, 15];
        let size = SensorCommon::parse_packet_header(&short_packet);
        assert_eq!(0, size, "truncated packet header should have length zero");

        let long_packet_len: usize = 1024;
        let mut raw_packet: [u8; PACKET_HEADER_LENGTH] = [
            (long_packet_len & 0xFF) as u8,
            long_packet_len.shr(8) as u8,
            0,
            0,
        ];
        let size = SensorCommon::parse_packet_header(&raw_packet);
        assert_eq!(size, long_packet_len, "verify > 255 packet length");

        //now set the continuation flag
        raw_packet[1] = 0x80 | raw_packet[1];
        let size = SensorCommon::parse_packet_header(&raw_packet);
        assert_eq!(size, long_packet_len, "verify continuation packet");

        let short_packet_len: usize = 36;
        raw_packet = [
            (short_packet_len & 0xFF) as u8,
            short_packet_len.shr(8) as u8,
            0,
            0,
        ];
        let size = SensorCommon::parse_packet_header(&raw_packet);
        assert_eq!(size, short_packet_len, "verify short packet");

        raw_packet[1] = 0x80 | raw_packet[1];
        let size = SensorCommon::parse_packet_header(&raw_packet);
        assert_eq!(size, short_packet_len, "verify short packet continuation");

        // first (uncontinued) packet
        raw_packet = [20 as u8, 1 as u8, 0, 0];
        let size = SensorCommon::parse_packet_header(&raw_packet);
        assert_eq!(size, 276, "verify > 255 packet length");

        //from actual received packet
        raw_packet = [19 as u8, 129 as u8, 0, 1];
        let size = SensorCommon::parse_packet_header(&raw_packet);
        assert_eq!(size, 275, "verify > 255 packet length");
    }
}
