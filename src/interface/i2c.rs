
use embedded_hal::{
    blocking::delay::{DelayUs, DelayMs},
};

use super::SensorInterface;
use crate::{Error, PACKET_HEADER_LENGTH};
use core::ops::{Shl, Shr};

/// the i2c address normally used by BNO080
pub const DEFAULT_ADDRESS: u8 =  0x4A;
/// alternate i2c address for BNO080
pub const ALTERNATE_ADDRESS: u8 =  0x4B;

const PACKET_SEND_BUF_LEN: usize = 256;
const SEG_RECV_BUF_LEN: usize = 32;
const PACKET_RECV_BUF_LEN: usize = 512;
const MAX_SEGMENT_READ: usize = SEG_RECV_BUF_LEN ;

const NUM_CHANNELS: usize = 6;

pub struct I2cInterface<I2C> {
    /// i2c port
    i2c_port:  I2C,

    /// address for i2c communications with the sensor hub
    address: u8,

    /// each communication channel with the device has its own sequence number
    sequence_numbers: [u8; NUM_CHANNELS],
    /// buffer for building and sending packet to the sensor hub
    packet_send_buf: [u8; PACKET_SEND_BUF_LEN],
    /// buffer for receiving segments of packets from the sensor hub
    seg_recv_buf: [u8; SEG_RECV_BUF_LEN],
    /// buffer for building packets received from the sensor hub
    packet_recv_buf: [u8; PACKET_RECV_BUF_LEN],


    /// has the device been succesfully reset
    device_reset: bool,
    prod_id_verified: bool,

    received_packet_count: u32,
}

impl<I2C, CommE> I2cInterface<I2C>
    where
        I2C: embedded_hal::blocking::i2c::Write<Error = CommE>,
{
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c_port: i2c,
            address: addr,
            sequence_numbers: [0; NUM_CHANNELS],
            packet_send_buf: [0; PACKET_SEND_BUF_LEN],
            seg_recv_buf: [0; SEG_RECV_BUF_LEN],
            packet_recv_buf: [0; PACKET_RECV_BUF_LEN],
            device_reset: false,
            prod_id_verified: false,
            received_packet_count: 0
        }
    }
}

impl<I2C, CommE> SensorInterface for I2cInterface<I2C>
    where
        I2C: embedded_hal::blocking::i2c::Write<Error = CommE>,
{
    type Error = Error<CommE>;


    /// Send a standard packet header followed by the body data provided
    fn send_packet(&mut self, channel: u8, body_data: &[u8]) -> Result<usize, Self::Error> {
        let body_len = body_data.len();

        self.sequence_numbers[channel as usize] += 1;
        let packet_length = body_len + PACKET_HEADER_LENGTH;
        let packet_header = [
            (packet_length & 0xFF) as u8, //LSB
            packet_length.shr(8) as u8, //MSB
            channel,
            self.sequence_numbers[channel as usize]
        ];

        self.packet_send_buf[..PACKET_HEADER_LENGTH].copy_from_slice(packet_header.as_ref());
        self.packet_send_buf[PACKET_HEADER_LENGTH..packet_length].copy_from_slice(body_data);
        self.i2c_port.write(self.address, &self.packet_send_buf[..packet_length]).map_err(Error::I2c)?;
        Ok(packet_length)
    }


    fn read_packet_header(&mut self, recv_buf: &mut [u8; PACKET_HEADER_LENGTH]) -> Result<(), Self::Error> {
        self.seg_recv_buf[0] = 0;
        self.seg_recv_buf[1] = 0;
        self.i2c_port.read(self.address, &mut recv_buf[..PACKET_HEADER_LENGTH]).map_err(Error::I2c)?;
        Ok(())
    }

    /// Read the remainder of the packet after the packet header, if any
    fn read_sized_packet(&mut self, total_packet_len: usize) -> Result<usize, Self::Error> {
        //iprintln!("sized: {}", total_packet_len).unwrap();
        let mut remaining_body_len: usize = total_packet_len - PACKET_HEADER_LENGTH;
        let mut already_read_len: usize = 0;

        if total_packet_len < MAX_SEGMENT_READ {
            if total_packet_len > 0 {
                self.packet_recv_buf[0] = 0;
                self.packet_recv_buf[1] = 0;
                self.i2c_port.read(self.address, &mut self.packet_recv_buf[..total_packet_len]).map_err(Error::I2c)?;
                //let packet_declared_len = Self::parse_packet_header(&self.packet_recv_buf[..PACKET_HEADER_LENGTH]);
                already_read_len = total_packet_len;
            }
        }
        else {
            while remaining_body_len > 0 {
                let whole_segment_length = remaining_body_len + PACKET_HEADER_LENGTH;
                let segment_read_len =
                    if whole_segment_length > MAX_SEGMENT_READ { MAX_SEGMENT_READ }
                    else { whole_segment_length };

                self.seg_recv_buf[0] = 0;
                self.seg_recv_buf[1] = 0;
                self.i2c_port.read(self.address, &mut self.seg_recv_buf[..segment_read_len]).map_err(Error::I2c)?;
                //let packet_declared_len = Self::parse_packet_header(&self.seg_recv_buf[..PACKET_HEADER_LENGTH]);

                //if we've never read any segments, transcribe the first packet header;
                //otherwise, just transcribe the segment body (no header)
                let transcribe_start_idx = if already_read_len > 0 { PACKET_HEADER_LENGTH } else { 0 };
                let transcribe_len = if already_read_len > 0 { segment_read_len - PACKET_HEADER_LENGTH } else { segment_read_len };
                self.packet_recv_buf[already_read_len..already_read_len+transcribe_len].
                    copy_from_slice(&self.seg_recv_buf[transcribe_start_idx..transcribe_start_idx+transcribe_len]);
                already_read_len += transcribe_len;

                let body_read_len = segment_read_len - PACKET_HEADER_LENGTH;
                remaining_body_len -= body_read_len;

            }
        }

        Ok(already_read_len)
    }

}
