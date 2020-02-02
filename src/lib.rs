/*
Copyright (c) 2019 Todd Stellanova
LICENSE: See LICENSE file
*/

#![no_std]

use embedded_hal::{
    blocking::delay::DelayMs,
    blocking::i2c::{Read, Write, WriteRead},
};
use core::ops::Shl;

use cortex_m_semihosting::{ hprintln};

/// the i2c address normally used by BNO080
pub const DEFAULT_ADDRESS: u8 =  0x4A;
/// alternate i2c address for BNO080
pub const ALTERNATE_ADDRESS: u8 =  0x4B;

const SEND_BUF_LEN: usize = 256;
const RECV_BUF_LEN: usize = 1024;

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
    send_buf: [u8; SEND_BUF_LEN],
    recv_buf: [u8; RECV_BUF_LEN],
    address: u8,
    port:  I,

}

/// The BNO080 uses Hillcrest’s SHTP (Sensor Hub Transport Protocol)

impl<I, E> BNO080<I>
    where
        I: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
{

    pub fn new(port: I) -> Self {
        BNO080 {
            sequence_numbers: [0; 6],
            send_buf: [0; SEND_BUF_LEN],
            recv_buf: [0; RECV_BUF_LEN],
            address: DEFAULT_ADDRESS,
            port: port,
        }
    }

    pub fn with_alternative_address(mut self) -> Self {
        self.address = ALTERNATE_ADDRESS;
        self
    }



    fn receive_advertisement(&mut self) -> Result<(), Error<E>> {

        let mut res = self.receive_packet();
        for i in 0..10 {
            if res.is_ok() {
                res = self.receive_packet();
            }
            else {
                break;
            }
        }


        if res.is_err() {
            hprintln!("final recv err").unwrap();
        }

        Ok(())
        //TODO look at contents of advertisement?
    }
    /// The BNO080 starts up with all sensors disabled,
    /// waiting for the application to configure it.
    pub fn init(&mut self, delay: &mut dyn DelayMs<u8>) -> Result<(), Error<E>> {
        //Section 5.1.1.1 :
        // On system startup, the SHTP control application will send
        // its full advertisement response, unsolicited, to the host.
        self.receive_advertisement()?;

        self.soft_reset(delay)?;


//        // check for a valid product ID response from sensor
//        let cmd_body: [u8; 2] = [
//            SHTP_SENSORHUB_PROD_ID_REQ, //request product ID
//            0, //reserved
//            ];
//
//        self.send_and_receive_packet(CHANNEL_CONTROL,&cmd_body)?;

//        let res = self.receive_packet();
//        if res.is_ok() {
//            //verify the response
//            let product_id = self.recv_buf[PACKET_HEADER_LENGTH + 0];
//            if SHTP_SENSORHUB_PROD_ID_RESP != product_id {
//                return Err(Error::InvalidChipId(product_id));
//            }
//        }

 //       self.enable_rotation_vector(100)?;
        Ok(())
    }

    pub fn enable_rotation_vector(&mut self, millis_between_reports: u16)  -> Result<(), Error<E>> {
        self.enable_report(SENSOR_REPORTID_ROTATION_VECTOR, millis_between_reports)
    }

    pub fn enable_report(&mut self, report_id: u8, millis_between_reports: u16)  -> Result<(), Error<E>> {
        let micros_between_reports: u32 = (millis_between_reports * 1000) as u32;
        let cmd_body: [u8; 17] = [
            SHTP_REPORT_SET_FEATURE_COMMAND,
            report_id,
            0, //feature flags
            0, //LSB change sensitivity
            0, //MSB change sensitivity
            (micros_between_reports & 0xFF) as u8, // LSB report interval, microseconds
            ((micros_between_reports >> 8)  & 0xFF) as u8,
            ((micros_between_reports >> 16) & 0xFF) as u8,
            ((micros_between_reports >> 24) & 0xFF) as u8, // MSB report interval
            0, // LSB Batch Interval
            0,
            0,
            0, // MSB Batch interval
            0, // LSB sensor-specific config
            0,
            0,
            0, // MSB sensor-specific config
        ];

        self.send_packet(CHANNEL_CONTROL, &cmd_body)

    }

    /// Send a packet and receive the response packet
    fn send_and_receive_packet(&mut self, channel: usize, body_data: &[u8]) -> Result<(), Error<E>> {
        self.sequence_numbers[channel] += 1;
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

//    pub fn send_reinitialize_command(&mut self) -> Result<(), Error<E>> {
//        let data:[u8; 12] = [
//            SHTP_SENSORHUB_COMMAND_REQ, // report ID
//            0, // TODO sequence
//            SH2_CMD_INITIALIZE,
//            SH2_INIT_SYSTEM,
//            0, 0, 0, 0,
//            0, 0, 0, 0,
//        ];
//
//        self.send_and_receive_packet(CHANNEL_CONTROL, data.as_ref())?;
//
//
//    }


    /// Send a soft reset command to the sensor
    pub fn soft_reset(&mut self, delay: &mut dyn DelayMs<u8>) -> Result<(), Error<E>> {
        let data:[u8; 1] = [EXECUTABLE_DEVICE_CMD_RESET]; //reset execute

        // send command packet and ignore received packets
        //self.send_and_receive_packet(CHANNEL_EXECUTABLE, data.as_ref())?;
        self.send_packet(CHANNEL_EXECUTABLE, data.as_ref())?;

        //give the device time to reset
        delay.delay_ms(50);

        //we may or may not receive a second garbage packet
 //       let _res = self.receive_packet(); //TODO seems to timeout

//        let mut res = self.receive_packet();
//        while res.is_ok() {
//            res = self.receive_packet();
//        }

        Ok(())
    }

    /// Read just the first header bytes of a packet
    /// Return the total size of the packet.
    fn read_packet_header(&mut self) -> Result<usize, Error<E>> {
        self.recv_buf[0] = 0;
        self.recv_buf[1] = 0;
//        hprintln!("start header read").unwrap();

        self.port.read(self.address, &mut self.recv_buf[..PACKET_HEADER_LENGTH]).map_err(Error::I2c)?;

        //Bits 14:0 are used to indicate the total number of bytes in the body plus header
        let raw_pack_len: u16 =
            (self.recv_buf[0] as u16) + (self.recv_buf[1] as u16).shl(8);

        let mut packet_len: usize =  (raw_pack_len & (!0x8000 as u16) ) as usize;
        let chan_num =  self.recv_buf[2];
        let seq_num =  self.recv_buf[3];


        hprintln!("packet_len: {} raw: {} ch {} seq {}", packet_len, raw_pack_len, chan_num, seq_num).unwrap();
        if packet_len > 256 {
            packet_len = 256; //TODO wtf advertising packet
        }
        Ok(packet_len)
    }

    /// Read the remainder of the packet after the packet header, if any
    fn read_packet_continuation(&mut self, total_packet_len: usize) -> Result<(), Error<E>> {
        hprintln!("read continuation: {}", total_packet_len).unwrap();
        if total_packet_len > RECV_BUF_LEN {
            hprintln!("overflow {} > {}", total_packet_len, RECV_BUF_LEN).unwrap();
        }
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
        hprintln!("send {} to {}, recv {} to {}", write_start, write_stop, read_start, read_stop).unwrap();

        self.port.write_read(self.address, sendo, recvo).map_err(Error::I2c)?;

        let packet_len_lsb: u16 = self.recv_buf[0] as u16;
        let packet_len_msb: u16 =  self.recv_buf[1] as u16;
        let chan_num =  self.recv_buf[2];
        let seq_num =  self.recv_buf[3];

        let mut packet_len:usize = (packet_len_msb.shl(8) | packet_len_lsb) as usize;
        packet_len = packet_len & (!32768); //clear continuation bit (MS)
        hprintln!("packet_len: {} ch {} seq {}", packet_len, chan_num, seq_num).unwrap();

        Ok(packet_len)
    }

}


/// length of packet headers
const PACKET_HEADER_LENGTH: usize = 4;

/// communication channels provided by BNO080
const  SHTP_CHAN_COMMAND: usize = 0;
const  CHANNEL_EXECUTABLE: usize = 1;
const  CHANNEL_CONTROL: usize = 2;
//const  CHANNEL_REPORTS: usize = 3;
//const  CHANNEL_WAKE_REPORTS: usize = 4;
//const  CHANNEL_GYRO: usize = 5;


/// SHTP constants
const SHTP_SENSORHUB_PROD_ID_REQ: u8 = 0xF9;
const SHTP_SENSORHUB_PROD_ID_RESP: u8 =  0xF8;


const SHTP_REPORT_SET_FEATURE_COMMAND: u8 = 0xFD;

const SENSOR_REPORTID_ROTATION_VECTOR: u8 = 0x05;

/// requests
const SHTP_SENSORHUB_COMMAND_REQ:u8 =      0xF2;

/// executable/device channel responses
/// Figure 1-27: SHTP executable commands and response
const EXECUTABLE_DEVICE_CMD_RESET: u8 =  1;
//const EXECUTABLE_DEVICE_CMD_ON: u8 =   2;
//const EXECUTABLE_DEVICE_CMD_SLEEP =  3;

/// Response to CMD_RESET
const EXECUTABLE_DEVICE_RESP_RESET_COMPLETE: u8 = 1;

const SH2_OK: i8 = 0; /// Success
const SH2_ERR: i8 = -1; ///  General Error

//#define SH2_ERR_BAD_PARAM      (-2) /**< Bad parameter to an API call */
//#define SH2_ERR_OP_IN_PROGRESS (-3) /**< Operation in progress */
//#define SH2_ERR_IO             (-4) /**< Error communicating with hub */
//#define SH2_ERR_HUB            (-5) /**< Error reported by hub */
//#define SH2_ERR_TIMEOUT        (-6) /**< Operation timed out */


/// Commands and subcommands
//const SH2_CMD_INITIALIZE: u8 =            4;
//const SH2_INIT_SYSTEM: u8 =               1;

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

//        pub fn set_available_packet(&mut self) {
//
//        }
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
        assert!(true, "oke");
//        let mock_i2c_port = FakeI2cPort::new();
//        let mut foo = BNO080::new(mock_i2c_port);
//        assert!(foo.init().is_ok(), "init failed");
    }

}
