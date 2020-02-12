/*
Copyright (c) 2020 Todd Stellanova
LICENSE: See LICENSE file
*/

#![no_std]

use embedded_hal::{
    blocking::delay::DelayMs,
    blocking::i2c::{Read, Write, WriteRead},
};
use core::ops::{Shl, Shr};



/// the i2c address normally used by BNO080
pub const DEFAULT_ADDRESS: u8 =  0x4A;
/// alternate i2c address for BNO080
pub const ALTERNATE_ADDRESS: u8 =  0x4B;

const PACKET_SEND_BUF_LEN: usize = 256;
const SEG_RECV_BUF_LEN: usize = 256;
const PACKET_RECV_BUF_LEN: usize = 1024;

/// the maximum number of bytes we can read from the device at one time
const MAX_TRANSFER_READ: usize = 255;///TODO i2c max transfer seems to be this, verify dynamically

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),

    /// Invalid chip ID was read
    InvalidChipId(u8),
    /// Unsupported sensor firmware version
    InvalidFWVersion(u8)
}

pub struct BNO080<I>  {
    /// each communication channel with the device has its own sequence number
    sequence_numbers: [u8; 6],
    /// buffer for building and sending packet to the sensor hub
    packet_send_buf: [u8; PACKET_SEND_BUF_LEN],
    /// buffer for receiving segments of packets from the sensor hub
    seg_recv_buf: [u8; SEG_RECV_BUF_LEN],
    /// buffer for building packets received from the sensor hub
    packet_recv_buf: [u8; PACKET_RECV_BUF_LEN],

    /// address for i2c communications with the sensor hub
    address: u8,

    /// i2c port
    port:  I,

    /// has the device been succesfully reset
    device_reset: bool,
}



/// The BNO080 uses Hillcrest’s SHTP (Sensor Hub Transport Protocol)

impl<I, E> BNO080<I>
    where
        I: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(port: I) -> Self {
        BNO080 {
            sequence_numbers: [0; 6],
            packet_send_buf: [0; PACKET_SEND_BUF_LEN],
            seg_recv_buf: [0; SEG_RECV_BUF_LEN],
            packet_recv_buf: [0; PACKET_RECV_BUF_LEN],
            address: DEFAULT_ADDRESS,
            port: port,
            device_reset: false
        }
    }

    pub fn with_alternative_address(mut self) -> Self {
        self.address = ALTERNATE_ADDRESS;
        self
    }

    /// Consume all available messages on the port without processing them
    pub fn eat_all_messages(&mut self) {
        loop {
            let res = self.receive_packet();
            let received_len = res.unwrap_or(0);
            if received_len == 0 {
                break;
            }
        }
    }

    fn parse_packet_header( packet: &[u8]) -> usize {
        const CONTINUATION_FLAG_CLEAR: u16 = !(0x80);
        if packet.len() < PACKET_HEADER_LENGTH {
            return 0;
        }
        //Bits 14:0 are used to indicate the total number of bytes in the body plus header
        //maximum packet length is ... 32767?
        let raw_pack_len: u16 =  (packet[0] as u16) + ((packet[1] as u16) & CONTINUATION_FLAG_CLEAR).shl(8);
        let packet_len: usize =  raw_pack_len as usize; //(raw_pack_len & (!0x8000 as u16) ) as usize;

        //let is_continuation:bool = (packet[1] & 0x80) != 0;
        //let chan_num =  packet[2];
        //let seq_num =  packet[3];

        packet_len
    }

    pub fn handle_one_message(&mut self, received_len: usize) {
        let msg = &self.packet_recv_buf[..received_len];
        let chan_num =  msg[2];
        //let _seq_num =  msg[3];
        let report_id: u8 = msg[4];

        match chan_num {
            CHANNEL_SENSOR_REPORTS => {
                self.handle_input_report(received_len);
            },
            SHTP_CHAN_COMMAND => {
                //iprintln!('command report_id: 0x{:01x}', report_id);
            },
            CHANNEL_EXECUTABLE => {
                match report_id {
                    EXECUTABLE_DEVICE_RESP_RESET_COMPLETE => {
                        self.device_reset = true;
                    },
                    _ => {
                        //iprintln!("executable: 0x{:01x}", report_id);
                    }
                }
            },
            CHANNEL_HUB_CONTROL => {
                match report_id {
                    SENSORHUB_COMMAND_RESP => {
                        let cmd_resp = msg[6];
                        if cmd_resp == SH2_STARTUP_INIT_UNSOLICITED {
                           //  iprintln!("SH2_STARTUP_INIT_UNSOLICIT");
                        }
                        else {
                           // iprintln!("SENSORHUB_COMMAND_RESP" {}",cmd_resp);
                        }
                    },
                    SENSORHUB_PROD_ID_REQ => {
                        //iprintln!("SHTP_SENSORHUB_PROD_ID_REQ");
                    },
                    SENSORHUB_PROD_ID_RESP => {
                        //iprintln!("SENSORHUB_PROD_ID_RESP");
                    },
                    _ =>  {
                        //iprintln!("control: 0x{:01x}", report_id).unwrap();
                         }
                }
            },
            _ => {
                //iprintln!("unhandled chan_num: {}", chan_num).unwrap();
                 }
        }

    }

    /// read and parse all available messages from sensorhub queue
    pub fn handle_all_messages(&mut self) -> u32 {
        let mut msg_count = 0;
        loop {
            let res = self.receive_packet();
            let received_len = res.unwrap_or(0);
            if received_len == 0 {
                break;
            }
            else {
                msg_count += 1;
                self.handle_one_message(received_len);
            }
        }

        msg_count
    }

//    fn receive_advertisement(&mut self) -> Result<(), Error<E>> {
//
//        //let mut received_len = self.receive_packet()?;
//
//        loop {
//            let received_len = self.receive_packet()?;
//            if received_len == 0 {
//                break;
//            }
//        }
//
//        iprintln!("recv adv done ").unwrap();
//        Ok(())
//        //TODO look at contents of advertisement?
//    }

    /// The BNO080 starts up with all sensors disabled,
    /// waiting for the application to configure it.
    pub fn init(&mut self, delay: &mut dyn DelayMs<u8>) -> Result<(), Error<E>> {
        //Section 5.1.1.1 :
        // On system startup, the SHTP control application will send
        // its full advertisement response, unsolicited, to the host.
        //self.eat_all_messages();
        self.handle_all_messages();
        delay.delay_ms(1);
        if !self.device_reset {
            self.send_reinitialize_command()?;
            //self.soft_reset()?;
            //delay.delay_ms(50);
        }
        //self.verify_product_id()?;

        self.enable_rotation_vector(500)?;
        Ok(())
    }

    pub fn enable_rotation_vector(&mut self, millis_between_reports: u16)  -> Result<(), Error<E>> {
        self.enable_report(SENSOR_REPORTID_ROTATION_VECTOR, millis_between_reports)
    }

    pub fn enable_report(&mut self, report_id: u8, millis_between_reports: u16)  -> Result<(), Error<E>> {
//        iprintln!("enable_report: {}", report_id).unwrap();
        let micros_between_reports: u32 = (millis_between_reports as u32) * 1000;
        let cmd_body: [u8; 17] = [
            SHTP_REPORT_SET_FEATURE_COMMAND,
            report_id,
            0, //feature flags
            0, //LSB change sensitivity
            0, //MSB change sensitivity
            (micros_between_reports & 0xFFu32) as u8, // LSB report interval, microseconds
            (micros_between_reports.shr(8)   & 0xFFu32 ) as u8,
            (micros_between_reports.shr( 16) & 0xFFu32 ) as u8,
            (micros_between_reports.shr(24) & 0xFFu32 ) as u8, // MSB report interval
            0, // LSB Batch Interval
            0,
            0,
            0, // MSB Batch interval
            0, // LSB sensor-specific config
            0,
            0,
            0, // MSB sensor-specific config
        ];

        //iprintln!("cmd_body: {:?}", cmd_body).unwrap();

        self.send_packet(CHANNEL_HUB_CONTROL, &cmd_body)?;
        Ok(())
    }


    // Sensor input reports have the form:
    // [u8; 5]  timestamp in microseconds
    // u8 report ID
    // u8 sequence number of report
    // ?? follows: about 5 * 2 bytes for eg rotation vec
    fn handle_input_report(&mut self, received_len: usize) {
        let msg = &self.packet_recv_buf[..received_len];
        let mut cursor = PACKET_HEADER_LENGTH; //skip header
        cursor += 5; // skip timestamp
        let feature_report_id = msg[cursor];
        //cursor += 1;

        match feature_report_id {
            SENSOR_REPORTID_ROTATION_VECTOR => {
                //iprintln!("SENSOR_REPORTID_ROTATION_VECTOR").unwrap();
            },
            _ => {
                //iprintln!("handle_input_report[{}]: 0x{:01x} ", received_len, feature_report_id).unwrap();
            }
        }
    }

    /// Send a packet and receive the response packet
    /// return the length of the response packet received
    fn send_and_receive_packet(&mut self, channel: u8, body_data: &[u8]) -> Result<usize, Error<E>> {
        let packet_length = body_data.len() + PACKET_HEADER_LENGTH;
        let packet_header = [
            (packet_length & 0xFF) as u8, //LSB
            packet_length.shr(8) as u8, //MSB
            channel,
            self.sequence_numbers[channel as usize]
        ];
        self.sequence_numbers[channel as usize] += 1;

        let body_len = body_data.len();
        let total_send_len = PACKET_HEADER_LENGTH + body_len;
        self.packet_send_buf[..PACKET_HEADER_LENGTH].copy_from_slice(packet_header.as_ref());
        self.packet_send_buf[PACKET_HEADER_LENGTH..total_send_len].copy_from_slice(body_data);

        self.seg_recv_buf[0] = 0;
        self.seg_recv_buf[1] = 0;

        //write the full packet and receive back the response packet header
        self.port.write_read(self.address,
                             &self.packet_send_buf[0..total_send_len],
                             &mut self.seg_recv_buf[..PACKET_HEADER_LENGTH]).
            map_err(Error::I2c)?;
        let response_packet_len = Self::parse_packet_header(&self.seg_recv_buf[..PACKET_HEADER_LENGTH]);

        //now read the full (size known) packet
        let received_len = self.read_sized_packet( response_packet_len)?;

        Ok(received_len)
    }

    /// Send a standard packet header followed by the body data provided
    fn send_packet(&mut self, channel: u8, body_data: &[u8]) -> Result<(), Error<E>> {
        let packet_length = body_data.len() + PACKET_HEADER_LENGTH;
        let packet_header = [
            (packet_length & 0xFF) as u8, //LSB
            (packet_length >> 8) as u8, //MSB
            channel,
            self.sequence_numbers[channel as usize]
        ];
        self.sequence_numbers[channel as usize] += 1;

        let body_len = body_data.len();
        let total_send_len = PACKET_HEADER_LENGTH + body_len;
        self.packet_send_buf[..PACKET_HEADER_LENGTH].copy_from_slice(packet_header.as_ref());
        self.packet_send_buf[PACKET_HEADER_LENGTH..total_send_len].copy_from_slice(body_data);

        self.port.write(self.address, &self.packet_send_buf[0..total_send_len]).map_err(Error::I2c)
    }

    /// Read one packet into the receive buffer
    fn receive_packet(&mut self) -> Result<usize, Error<E>> {
        let packet_len:usize = self.read_unsized_packet()?;
        let received_len = self.read_sized_packet(packet_len)?;

       Ok(received_len)
    }

    /// check for a valid product ID response from sensor
    fn verify_product_id(&mut self) -> Result<(), Error<E>> {
        let cmd_body: [u8; 2] = [
            SENSORHUB_PROD_ID_REQ, //request product ID
            0, //reserved
            ];

        self.send_packet(CHANNEL_HUB_CONTROL, &cmd_body)?;
        let recv_len = self.receive_packet()?;

        //verify the response
        if recv_len > PACKET_HEADER_LENGTH {
            //iprintln!("resp: {:?}", &self.msg_buf[..recv_len]).unwrap();
            //TODO this sometimes doesn't match because another response interjects
            let report_id = self.packet_recv_buf[PACKET_HEADER_LENGTH + 0];
            if SENSORHUB_PROD_ID_RESP != report_id {
//                iprintln!("prod_id report_id: {} ??", report_id).unwrap();
                return Err(Error::InvalidChipId(0));
            }

//            let sw_ver_major = self.msg_buf[2];
//            let sw_ver_minor = self.msg_buf[3];
//            iprintln!("FW version: {}.{} ", sw_ver_major, sw_ver_minor).unwrap();
            //TODO detect invalid sw version
        }

        Ok(())
    }

    fn send_reinitialize_command(&mut self) -> Result<(), Error<E>> {
        let data:[u8; 12] = [
            SENSORHUB_COMMAND_REQ, // report ID
            self.sequence_numbers[CHANNEL_HUB_CONTROL as usize],
            SH2_CMD_INITIALIZE, //command
            SH2_INIT_SYSTEM, //p9
            0, 0, 0, 0,
            0, 0, 0, 0,
        ];
        self.sequence_numbers[CHANNEL_HUB_CONTROL as usize] += 1;

        let resp_pack_len = self.send_and_receive_packet(CHANNEL_HUB_CONTROL, data.as_ref())?;
        if resp_pack_len > 0 {
            let report_id = data[PACKET_HEADER_LENGTH + 0];
            let cmd = data[PACKET_HEADER_LENGTH + 1];
            let _cmd_seq = data[PACKET_HEADER_LENGTH + 2];
            let _resp_seq = data[PACKET_HEADER_LENGTH + 3];
            let _resp_rc = data[PACKET_HEADER_LENGTH + 4];

            if report_id != SENSORHUB_COMMAND_RESP {
                //iprintln!(Peripherals::take().unwrap().ITM.stim[0].borrow_mut(),
                // "bogus report id: {}", report_id);
                //TODO error out?
            }
            if cmd != SH2_CMD_INITIALIZE {
                //iprintln!(Peripherals::take().unwrap().ITM.stim[0].borrow_mut(),
                //"cmd: {}", cmd);
                //TODO error out?
            }

        }

        Ok(())
    }

//    /// Send a soft reset command to the sensor
//    pub fn soft_reset(&mut self, delay: &mut dyn DelayMs<u8>) -> Result<(), Error<E>> {
//        let data:[u8; 1] = [EXECUTABLE_DEVICE_CMD_RESET]; //reset execute
//
//        // send command packet and ignore received packets
////        let received_len = self.send_and_receive_packet(CHANNEL_EXECUTABLE, data.as_ref())?;
//        let _rc = self.send_packet(CHANNEL_EXECUTABLE, data.as_ref());
//
//        delay.delay_ms(50);
//
//        self.eat_all_messages();
//
////        iprintln!("received_len: {}",received_len).unwrap();
////        //give the device time to reset
////        delay.delay_ms(50);
//
//        //we may or may not receive a second garbage packet
// //       let _res = self.receive_packet(); //TODO seems to timeout
//
////        let mut res = self.receive_packet();
////        while res.is_ok() {
////            res = self.receive_packet();
////        }
//
//        Ok(())
//    }

    /// Read just the first header bytes of a packet
    /// Return the total size of the packet that follows
    fn read_unsized_packet(&mut self) -> Result<usize, Error<E>> {
        self.seg_recv_buf[0] = 0;
        self.seg_recv_buf[1] = 0;
        self.port.read(self.address, &mut self.seg_recv_buf[..PACKET_HEADER_LENGTH]).map_err(Error::I2c)?;
        let packet_len = Self::parse_packet_header(&self.seg_recv_buf[..PACKET_HEADER_LENGTH]);
        Ok(packet_len)
    }



    /// Read the remainder of the packet after the packet header, if any
    fn read_sized_packet(&mut self, total_packet_len: usize) -> Result<usize, Error<E>> {
        //iprintln!("sized: {}", total_packet_len).unwrap();
        let mut remaining_len: usize = total_packet_len;
        let mut already_read_len: usize = 0;

        if total_packet_len < MAX_TRANSFER_READ {
            if total_packet_len > 0 {
                //iprintln!("simple read: {}",total_packet_len).unwrap();
                self.packet_recv_buf[0] = 0;
                self.packet_recv_buf[1] = 0;
                self.port.read(self.address, &mut self.packet_recv_buf[..total_packet_len]).map_err(Error::I2c)?;
                let packet_declared_len = Self::parse_packet_header(&self.packet_recv_buf[..PACKET_HEADER_LENGTH]);
                already_read_len = packet_declared_len;
            }
        }
        else {
            while remaining_len > 0 {
                //TODO simplify and test this directly
                let mut cur_read_len = remaining_len;
                if cur_read_len > MAX_TRANSFER_READ { cur_read_len = MAX_TRANSFER_READ; }

                self.seg_recv_buf[0] = 0;
                self.seg_recv_buf[1] = 0;
//                iprintln!("partial {} / {}", cur_read_len, remaining_len).unwrap();
                self.port.read(self.address, &mut self.seg_recv_buf[..cur_read_len]).map_err(Error::I2c)?;
                let packet_declared_len = Self::parse_packet_header(&self.seg_recv_buf[..PACKET_HEADER_LENGTH]);

                //if we've never read any segments, transcribe the first packet header;
                //otherwise, just transcribe the segment body (no header)
                let transcribe_start_idx = if already_read_len > 0 { PACKET_HEADER_LENGTH } else { 0 };
                let transcribe_len = if already_read_len > 0 { cur_read_len - PACKET_HEADER_LENGTH } else { cur_read_len };

                self.packet_recv_buf[already_read_len..already_read_len+transcribe_len].
                    copy_from_slice(&self.seg_recv_buf[transcribe_start_idx..cur_read_len]);

                remaining_len = packet_declared_len - cur_read_len;
                if remaining_len > 0 { remaining_len += PACKET_HEADER_LENGTH};
                already_read_len += cur_read_len;
//                iprintln!("already {} remaining {}", already_read_len, remaining_len).unwrap();
            }
        }

        Ok(already_read_len)
    }

}


/// length of packet headers
const PACKET_HEADER_LENGTH: usize = 4;

// The BNO080 supports six communication channels:
const  SHTP_CHAN_COMMAND: u8 = 0; /// the SHTP command channel
const  CHANNEL_EXECUTABLE: u8 = 1; /// executable channel
const  CHANNEL_HUB_CONTROL: u8 = 2; /// sensor hub control channel
const  CHANNEL_SENSOR_REPORTS: u8 = 3; /// input sensor reports (non-wake, not gyroRV)
//const  CHANNEL_WAKE_REPORTS: usize = 4; /// wake input sensor reports (for sensors configured as wake up sensors)
//const  CHANNEL_GYRO_ROTATION: usize = 5; ///  gyro rotation vector (gyroRV)



/// SHTP constants
const SENSORHUB_PROD_ID_REQ: u8 = 0xF9;
const SENSORHUB_PROD_ID_RESP: u8 =  0xF8;


const SHTP_REPORT_SET_FEATURE_COMMAND: u8 = 0xFD;

const SENSOR_REPORTID_ROTATION_VECTOR: u8 = 0x05;

/// requests
const SENSORHUB_COMMAND_REQ:u8 =      0xF2;
const SENSORHUB_COMMAND_RESP:u8 =       0xF1;


/// executable/device channel responses
/// Figure 1-27: SHTP executable commands and response
//const EXECUTABLE_DEVICE_CMD_RESET: u8 =  1;
//const EXECUTABLE_DEVICE_CMD_ON: u8 =   2;
//const EXECUTABLE_DEVICE_CMD_SLEEP =  3;

/// Response to CMD_RESET
const EXECUTABLE_DEVICE_RESP_RESET_COMPLETE: u8 = 1;

//const SH2_OK: i8 = 0; /// Success
//const SH2_ERR: i8 = -1; ///  General Error

//#define SH2_ERR_BAD_PARAM      (-2) /**< Bad parameter to an API call */
//#define SH2_ERR_OP_IN_PROGRESS (-3) /**< Operation in progress */
//#define SH2_ERR_IO             (-4) /**< Error communicating with hub */
//#define SH2_ERR_HUB            (-5) /**< Error reported by hub */
//#define SH2_ERR_TIMEOUT        (-6) /**< Operation timed out */


/// Commands and subcommands
const SH2_INIT_UNSOLICITED: u8 = 0x80;
const SH2_CMD_INITIALIZE: u8 = 4;
const SH2_INIT_SYSTEM: u8 = 1;
const SH2_STARTUP_INIT_UNSOLICITED:u8 = SH2_CMD_INITIALIZE | SH2_INIT_UNSOLICITED;



#[cfg(test)]
mod tests {
    use crate::{BNO080, PACKET_HEADER_LENGTH};
    use embedded_hal::blocking::{
        delay::DelayMs,
        i2c::{Read, WriteRead, Write}
    };
    use core::ops::Shr;

    struct FakeDelay {}

    impl DelayMs<u8> for FakeDelay {
        fn delay_ms(&mut self, _ms: u8) {
            // no-op
        }
    }

    const RECV_BUF_LEN: usize = 256;
    const SEND_BUF_LEN: usize = 256;

    struct FakeI2cPort {
        pub packet_available_buf: [u8; SEND_BUF_LEN],
        pub packet_received_buf: [u8; RECV_BUF_LEN],
    }

    impl FakeI2cPort {
        fn new() -> Self {
            FakeI2cPort {
                packet_available_buf: [0; SEND_BUF_LEN ],
                packet_received_buf: [0; RECV_BUF_LEN]
            }
        }

        pub fn set_available_packet(&mut self, bytes: &[u8]) {
            self.packet_available_buf.copy_from_slice(bytes);
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
        assert!(true, "ok");
        let mock_i2c_port = FakeI2cPort::new();
        let mut shub = BNO080::new(mock_i2c_port);
        let rc = shub.init(&mut FakeDelay {} );
        assert!(rc.is_ok(), "init failed");
    }

    #[test]
    fn test_parse_packet_header() {

        let short_packet: [u8; 2] = [ 13, 15];
        let size = BNO080::<FakeI2cPort>::parse_packet_header(&short_packet);
        assert_eq!(0, size, "truncated packet header should have length zero");

        let long_packet_len: usize = 1024;
        let mut raw_packet: [u8; PACKET_HEADER_LENGTH] = [
            (long_packet_len & 0xFF) as u8 ,
            long_packet_len.shr(8)  as u8,
            0,
            0
        ];
        let size = BNO080::<FakeI2cPort>::parse_packet_header(&raw_packet);
        assert_eq!(size, long_packet_len, "verify > 255 packet length");

        //now set the continuation flag
        raw_packet[1] = 0x80 | raw_packet[1];
        let size = BNO080::<FakeI2cPort>::parse_packet_header(&raw_packet);
        assert_eq!(size, long_packet_len, "verify continuation packet");

        let short_packet_len: usize = 36;
        raw_packet = [
            (short_packet_len & 0xFF) as u8 ,
            short_packet_len.shr(8)  as u8,
            0,
            0
        ];
        let size = BNO080::<FakeI2cPort>::parse_packet_header(&raw_packet);
        assert_eq!(size, short_packet_len, "verify short packet");

        raw_packet[1] = 0x80 | raw_packet[1];
        let size = BNO080::<FakeI2cPort>::parse_packet_header(&raw_packet);
        assert_eq!(size, short_packet_len, "verify short packet continuation");

        // first (uncontinued) packet
        raw_packet = [
            20 as u8 ,
            1  as u8,
            0,
            0
        ];
        let size = BNO080::<FakeI2cPort>::parse_packet_header(&raw_packet);
        assert_eq!(size, 276, "verify > 255 packet length");

        //from actual received packet
        raw_packet = [
            19 as u8 ,
            129  as u8,
            0,
            1
        ];
        let size = BNO080::<FakeI2cPort>::parse_packet_header(&raw_packet);
        assert_eq!(size, 275, "verify > 255 packet length");

    }

}
