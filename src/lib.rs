/*
Copyright (c) 2019 Todd Stellanova
LICENSE: See LICENSE file
*/

#![no_std]

use embedded_hal::{
    blocking::i2c::{Read, Write},
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
        I: Read<Error = E> + Write<Error = E>,
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

//        let id = self.id()?;
//        if id != BNO055_ID {
//            return Err(Error::InvalidChipId(id));
//        }

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

        // TODO proper error handling (Eg on no ACK from sensor)
        let res = self.port.write(self.address, self.send_buf.as_ref());
        res.map_err(Error::I2c)
    }

    /// Read one packet into the receive buffer
    fn receive_packet(&mut self) -> Result<(), Error<E>> {
        let mut header_data:[u8; PACKET_HEADER_LENGTH] = [0,0,0,0];
        //read packet header
        let mut result = self.port.read(self.address, &mut header_data);
        if result.is_ok() {
            //got a packet
            let packet_len_lsb = header_data[0];
            let packet_len_msb =  header_data[1];
            let _chan_num =  header_data[2]; //TODO always CHANNEL_REPORTS ?
            let  _seq_num =  header_data[3];

            let mut packet_length:usize = (packet_len_msb << 8 | packet_len_lsb) as usize;
            if packet_length > 0 {
                //continuation bit, MS, is 1<<15 = 32768
                packet_length = packet_length & (!32768); //clear continuation bit (MS)
                packet_length -= 4; //remove header length
                result = self.port.read(self.address, &mut self.recv_buf[..packet_length] );
            }
            //TODO else...result is bad
        }

        result.map_err(Error::I2c)
    }

    pub fn soft_reset(&mut self) -> Result<(), Error<E>> {
        let data:[u8; 1] = [1];
        self.send_packet(CHANNEL_EXECUTABLE, data.as_ref())
        //TODO read any garbage data, check for errors
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
    use embedded_hal::blocking::i2c::{Read, Write};

    struct FakeI2cPort {

    }

    impl FakeI2cPort {
        fn new() -> Self {
            FakeI2cPort {

            }
        }
    }
    impl Read for FakeI2cPort {
        type Error = ();

        fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    impl Write for FakeI2cPort {
        type Error = ();

        fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    fn test_setup() {
        let mut mock_i2c_port = FakeI2cPort::new();
        let mut foo = BNO080::new(mock_i2c_port);
        assert!(foo.init().is_ok(), "init failed");
    }

}
