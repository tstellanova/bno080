/*
Copyright (c) 2019 Todd Stellanova
LICENSE: See LICENSE file
*/

#![no_std]

use embedded_hal::{
    blocking::i2c::{Read, Write, WriteRead},
};

/// the i2c address normally used by BNO080
pub const DEFAULT_ADDRESS: u8 =  0x4B;
/// alternate i2c address for BNO080
pub const ALTERNATE_ADDRESS: u8 =  0x4A;


/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),

    /// Invalid chip ID was read
    InvalidChipId(u8),
}

pub struct BNO080<I>  {
    // each communication channel with the device has its own sequence number
    sequence_numbers: [u8; 6],
    send_buf: [u8; 64],
    recv_buf: [u8; 256],
    address: u8,
    port:  I,
}


impl<I, E> BNO080<I>
    where
        I: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
{

    pub fn new(port: I) -> Self {
        BNO080 {
            sequence_numbers: [0; 6],
            send_buf: [0; 64],
            recv_buf: [0; 256],
            address: DEFAULT_ADDRESS,
            port: port,
        }
    }

    pub fn with_alternative_address(mut self) -> Self {
        self.address = ALTERNATE_ADDRESS;
        self
    }

    pub fn init(&mut self) -> Result<(), Error<E>> {
        self.soft_reset()?;

        //TODO check for invalid product/chip ID

        Ok(())
    }


    /// Send a packet and receive the response packet
    fn send_and_receive_packet(&mut self, channel: usize, body_data: &[u8]) -> Result<(), Error<E>> {
        let packet_length = body_data.len() + PACKET_HEADER_LENGTH;
        let packet_header = [
            (packet_length & 0xFF) as u8, //LSB
            (packet_length >> 8) as u8, //MSB
            channel as u8,
            self.sequence_numbers[channel]
        ];

        let body_len = body_data.len();
        self.send_buf[..PACKET_HEADER_LENGTH].copy_from_slice(packet_header.as_ref());
        self.send_buf[PACKET_HEADER_LENGTH..PACKET_HEADER_LENGTH + body_len].copy_from_slice(body_data);
        self.sequence_numbers[channel] += 1;

        let response_packet_len = self.write_read_buffer(0, body_len+PACKET_HEADER_LENGTH,
            0, PACKET_HEADER_LENGTH)?;
        self.read_packet_continuation( response_packet_len)?;

        Ok(())
    }

    /// Send a standard packet header followed by the body data provided
    fn send_packet(&mut self, channel: usize, body_data: &[u8]) -> Result<(), Error<E>> {
        let packet_length = body_data.len() + PACKET_HEADER_LENGTH;
        let packet_header = [
            (packet_length & 0xFF) as u8, //LSB
            (packet_length >> 8) as u8, //MSB
            channel as u8,
            self.sequence_numbers[channel]
        ];

        let body_len = body_data.len();
        self.send_buf[..PACKET_HEADER_LENGTH].copy_from_slice(packet_header.as_ref());
        self.send_buf[PACKET_HEADER_LENGTH..PACKET_HEADER_LENGTH+body_len].copy_from_slice(body_data);
        self.sequence_numbers[channel] += 1;

        self.port.write(self.address, self.send_buf.as_ref()).map_err(Error::I2c)

    }

    /// Read one packet into the receive buffer
    fn receive_packet(&mut self) -> Result<(), Error<E>> {
        let packet_len:usize = self.read_packet_header()?;
        self.read_packet_continuation(packet_len)?;

       Ok(())
    }

    /// Send a soft reset command to the sensor
    pub fn soft_reset(&mut self) -> Result<(), Error<E>> {
        let data:[u8; 1] = [1]; //reset execute

        // send command packet and ignore received packets
        self.send_and_receive_packet(CHANNEL_EXECUTABLE, data.as_ref())?;

        //we may or may not receive a second garbage packet
        let _res = self.receive_packet();

        Ok(())
    }

    /// Read just the first header bytes of a packet
    /// Return the total size of the packet.
    fn read_packet_header(&mut self) -> Result<usize, Error<E>> {
        self.recv_buf[0] = 0;
        self.recv_buf[1] = 0;
        self.port.read(self.address, &mut self.recv_buf[..PACKET_HEADER_LENGTH]).map_err(Error::I2c)?;

        let packet_len_lsb = self.recv_buf[0];
        let packet_len_msb =  self.recv_buf[1];
        //let _chan_num =  header_data[2];
        //let _seq_num =  header_data[3];

        let mut packet_len:usize = (packet_len_msb << 8 | packet_len_lsb) as usize;
        packet_len = packet_len & (!32768); //clear continuation bit (MS)

        Ok(packet_len)
    }

    /// Read the remainder of the packet after the packet header, if any
    fn read_packet_continuation(&mut self, total_packet_len: usize) -> Result<(), Error<E>> {
        if total_packet_len > PACKET_HEADER_LENGTH {
            self.port.read(self.address, &mut self.recv_buf[PACKET_HEADER_LENGTH..total_packet_len]).map_err(Error::I2c)?;
        }

        Ok(())
    }

//    fn read_bytes(&mut self,  buffer: &mut [u8]) -> Result<(), Error<E>> {
//        //Clear the "packet length" header for responses
//        buffer[0] = 0;
//        buffer[1] = 0;
//        self.port.read(self.address, buffer).map_err(Error::I2c)
//    }
    
//    fn read_register(&mut self, reg: u8) -> Result<u8, E> {
//        let mut byte: [u8; 1] = [0; 1];
//
//        match self.port.write_read(self.address, &[reg], &mut byte) {
//            Ok(_) => Ok(byte[0]),
//            Err(e) => Err(e),
//        }
//    }

//    /// Write bytes to and read bytes from target in a single transaction
//    fn write_read_bytes(&mut self,  bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error<E>> {
//        //Clear the "packet length" header for responses
//        buffer[0] = 0;
//        buffer[1] = 0;
//        self.port.write_read(self.address, bytes, buffer).map_err(Error::I2c)
//    }

    /// Write from the send buffer and receive into the receive buffer in a single transaction
    fn write_read_buffer(&mut self, write_start: usize, write_stop: usize, read_start: usize, read_stop: usize) -> Result<usize, Error<E>>  {
        let sendo = &mut self.send_buf[write_start..write_stop];
        let recvo = &mut self.recv_buf[read_start..read_stop];
        recvo[0] = 0;
        recvo[1] = 0;
        self.port.write_read(self.address, sendo, recvo).map_err(Error::I2c)?;

        let packet_len_lsb = self.recv_buf[0];
        let packet_len_msb =  self.recv_buf[1];
        //let _chan_num =  header_data[2];
        //let _seq_num =  header_data[3];

        let mut packet_len:usize = (packet_len_msb << 8 | packet_len_lsb) as usize;
        packet_len = packet_len & (!32768); //clear continuation bit (MS)

        Ok(packet_len)
    }

}


/// length of packet headers
const PACKET_HEADER_LENGTH: usize = 4;

/// communication channels provided by BNO080
const  CHANNEL_COMMAND: usize = 0;
const  CHANNEL_EXECUTABLE: usize = 1;
const  CHANNEL_CONTROL: usize = 2;
const  CHANNEL_REPORTS: usize = 3;
const  CHANNEL_WAKE_REPORTS: usize = 4;
const  CHANNEL_GYRO: usize = 5;


#[cfg(test)]
mod tests {
    use crate::BNO080;
    use embedded_hal::blocking::i2c::{Read, WriteRead, Write};

    struct FakeI2cPort {

    }

    impl FakeI2cPort {
        fn new() -> Self {
            FakeI2cPort {

            }
        }

        pub fn set_available_packet(&mut self) {

        }
    }

    impl Read for FakeI2cPort {
        type Error = ();

        fn read(&mut self, _address: u8, _buffer: &mut [u8]) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    impl Write for FakeI2cPort {
        type Error = ();

        fn write(&mut self, _addr: u8, _bytes: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    impl WriteRead for FakeI2cPort {
        type Error = ();

        fn write_read(&mut self, _addr: u8, _bytes: &[u8], _buffer: &mut [u8]) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    fn test_setup() {
        let mock_i2c_port = FakeI2cPort::new();
        let mut foo = BNO080::new(mock_i2c_port);
        assert!(foo.init().is_ok(), "init failed");
    }

}
