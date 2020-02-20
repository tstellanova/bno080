/*
Copyright (c) 2020 Todd Stellanova
LICENSE: See LICENSE file
*/

#![no_std]

pub mod interface;
pub mod wrapper;

// use embedded_hal::{
//     blocking::delay::DelayMs,
//     //blocking::i2c::{Read, Write, WriteRead},
// };
// use core::ops::{Shl, Shr};


// /// the i2c address normally used by BNO080
// pub const DEFAULT_ADDRESS: u8 =  0x4A;
// /// alternate i2c address for BNO080
// pub const ALTERNATE_ADDRESS: u8 =  0x4B;

// const PACKET_SEND_BUF_LEN: usize = 256;
// const SEG_RECV_BUF_LEN: usize = 32;
// const PACKET_RECV_BUF_LEN: usize = 512;
//
// /// the maximum number of bytes we can read from the device at one time
// const MAX_SEGMENT_READ: usize = SEG_RECV_BUF_LEN ;

// #[derive(Debug)]
// pub enum Error<E> {
//     /// I2C bus error
//     I2c(E),
//
//     /// Invalid chip ID was read
//     InvalidChipId(u8),
//     /// Unsupported sensor firmware version
//     InvalidFWVersion(u8),
//
//     /// Not enough data available to fulfill the read
//     NoDataAvailable(u8),
// }

// pub struct BNO080<I>  {
//     /// each communication channel with the device has its own sequence number
//     sequence_numbers: [u8; 6],
//     /// buffer for building and sending packet to the sensor hub
//     packet_send_buf: [u8; PACKET_SEND_BUF_LEN],
//     /// buffer for receiving segments of packets from the sensor hub
//     seg_recv_buf: [u8; SEG_RECV_BUF_LEN],
//     /// buffer for building packets received from the sensor hub
//     packet_recv_buf: [u8; PACKET_RECV_BUF_LEN],
//
//     /// address for i2c communications with the sensor hub
//     address: u8,
//
//     /// i2c port
//     i2c_port:  I,
//
//     /// has the device been succesfully reset
//     device_reset: bool,
//     prod_id_verified: bool,
//
//     received_packet_count: u32,
// }


// The BNO080 uses Hillcrest’s SHTP (Sensor Hub Transport Protocol)

// impl<I, E> BNO080<I>
//     where
//         I: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,  E: core::fmt::Debug
// {
//     pub fn new(port: I) -> Self {
//         BNO080 {
//             sequence_numbers: [0; 6],
//             packet_send_buf: [0; PACKET_SEND_BUF_LEN],
//             seg_recv_buf: [0; SEG_RECV_BUF_LEN],
//             packet_recv_buf: [0; PACKET_RECV_BUF_LEN],
//             address: DEFAULT_ADDRESS,
//             i2c_port: port,
//             device_reset: false,
//             prod_id_verified: false,
//             received_packet_count: 0,
//         }
//     }

    // pub fn with_alternative_address(mut self) -> Self {
    //     self.address = ALTERNATE_ADDRESS;
    //     self
    // }

    // /// Receive and ignore one message
    // pub fn eat_one_message(&mut self) -> usize {
    //     let res = self.receive_packet();
    //     res.unwrap_or(0)
    // }

    // /// Consume all available messages on the port without processing them
    // pub fn eat_all_messages(&mut self,  delay: &mut dyn DelayMs<u8>) {
    //     loop {
    //         let received_len = self.eat_one_message();
    //         if received_len == 0 {
    //             break;
    //         }
    //         else {
    //             //give some time to other parts of the system
    //             delay.delay_ms(1);
    //         }
    //     }
    // }

    // pub fn get_received_packet_count(&self) -> u32 {
    //     self.received_packet_count
    // }

    // fn parse_packet_header( packet: &[u8]) -> usize {
    //     const CONTINUATION_FLAG_CLEAR: u16 = !(0x80);
    //     if packet.len() < PACKET_HEADER_LENGTH {
    //         return 0;
    //     }
    //     //Bits 14:0 are used to indicate the total number of bytes in the body plus header
    //     //maximum packet length is ... 32767?
    //     let raw_pack_len: u16 =  (packet[0] as u16) + ((packet[1] as u16) & CONTINUATION_FLAG_CLEAR).shl(8);
    //     let packet_len: usize =  raw_pack_len as usize;
    //
    //     //let is_continuation:bool = (packet[1] & 0x80) != 0;
    //     //let chan_num =  packet[2];
    //     //let seq_num =  packet[3];
    //
    //     packet_len
    // }

    // pub fn handle_received_packet(&mut self, received_len: usize) {
    //     let msg = &self.packet_recv_buf[..received_len];
    //     let chan_num =  msg[2];
    //     //let _seq_num =  msg[3];
    //     let report_id: u8 = msg[4];
    //
    //     match chan_num {
    //         CHANNEL_SENSOR_REPORTS => {
    //             self.handle_input_report(received_len);
    //         },
    //         SHTP_CHAN_COMMAND => {
    //             match report_id {
    //                 0 => { //RESP_ADVERTISE
    //                     self.handle_advertise_response(received_len);
    //                 },
    //                 _ => {
    //
    //                 }
    //             }
    //         },
    //         CHANNEL_EXECUTABLE => {
    //             match report_id {
    //                 EXECUTABLE_DEVICE_RESP_RESET_COMPLETE => {
    //                     self.device_reset = true;
    //                 },
    //                 _ => {
    //
    //                 }
    //             }
    //         },
    //         CHANNEL_HUB_CONTROL => {
    //             match report_id {
    //                 SENSORHUB_COMMAND_RESP => {
    //                     let cmd_resp = msg[6];
    //                     if cmd_resp == SH2_STARTUP_INIT_UNSOLICITED {
    //
    //                     }
    //                     else {
    //                     }
    //                 },
    //                 SENSORHUB_PROD_ID_RESP => {
    //                     self.prod_id_verified = true;
    //                 },
    //                 _ =>  {
    //
    //                 }
    //             }
    //         },
    //         _ => {
    //
    //              }
    //     }
    //
    // }

    // // return the number of messages handled
    // pub fn handle_one_message(&mut self) -> u32 {
    //     let mut msg_count = 0;
    //
    //     let res = self.receive_packet();
    //     if res.is_ok()  {
    //         let received_len = res.unwrap_or(0);
    //         if received_len > 0 {
    //             msg_count += 1;
    //             self.handle_received_packet(received_len);
    //         }
    //     }
    //
    //     msg_count
    // }

    // /// read and parse all available messages from sensorhub queue
    // pub fn handle_all_messages(&mut self) -> u32 {
    //     let mut msg_count = 0;
    //
    //     loop  {
    //         let iter_count = self.handle_one_message();
    //         if iter_count < 1 {
    //             break;
    //         }
    //         msg_count += iter_count;
    //     }
    //
    //     msg_count
    // }


    // /// The BNO080 starts up with all sensors disabled,
    // /// waiting for the application to configure it.
    // pub fn init(&mut self, delay: &mut dyn DelayMs<u8>) -> Result<(), Error<E>> {
    //     //Section 5.1.1.1 :
    //     // On system startup, the SHTP control application will send
    //     // its full advertisement response, unsolicited, to the host.
    //
    //     self.soft_reset()?;
    //     delay.delay_ms(50);
    //     self.eat_one_message();
    //     delay.delay_ms(50);
    //     self.eat_all_messages(delay);
    //     // delay.delay_ms(50);
    //     // self.eat_all_messages(delay);
    //
    //     self.verify_product_id()?;
    //
    //     Ok(())
    // }

    // Tell the sensor to start reporting the fused rotation vector
    // on a regular cadence. Note that the maximum valid update rate
    // is 1 kHz, based on the max update rate of the sensor's gyros.
    // pub fn enable_rotation_vector(&mut self, millis_between_reports: u16)  -> Result<(), Error<E>> {
    //     self.enable_report(SENSOR_REPORTID_ROTATION_VECTOR, millis_between_reports)
    // }

    // Enable a particular report
    // fn enable_report(&mut self, report_id: u8, millis_between_reports: u16)  -> Result<(), Error<E>> {
    //     let micros_between_reports: u32 = (millis_between_reports as u32) * 1000;
    //     let cmd_body: [u8; 17] = [
    //         SHTP_REPORT_SET_FEATURE_COMMAND,
    //         report_id,
    //         0, //feature flags
    //         0, //LSB change sensitivity
    //         0, //MSB change sensitivity
    //         (micros_between_reports & 0xFFu32) as u8, // LSB report interval, microseconds
    //         (micros_between_reports.shr(8)   & 0xFFu32 ) as u8,
    //         (micros_between_reports.shr( 16) & 0xFFu32 ) as u8,
    //         (micros_between_reports.shr(24) & 0xFFu32 ) as u8, // MSB report interval
    //         0, // LSB Batch Interval
    //         0,
    //         0,
    //         0, // MSB Batch interval
    //         0, // LSB sensor-specific config
    //         0,
    //         0,
    //         0, // MSB sensor-specific config
    //     ];
    //
    //     self.send_packet(CHANNEL_HUB_CONTROL, &cmd_body)?;
    //     Ok(())
    // }


    // Sensor input reports have the form:
    // [u8; 5]  timestamp in microseconds
    // u8 report ID
    // u8 sequence number of report
    // ?? follows: about 5 * 2 bytes for eg rotation vec
    // fn handle_input_report(&mut self, received_len: usize) {
    //     let msg = &self.packet_recv_buf[..received_len];
    //     let mut cursor = PACKET_HEADER_LENGTH; //skip header
    //     cursor += 5; // skip timestamp
    //     let feature_report_id = msg[cursor];
    //     //cursor += 1;
    //
    //     match feature_report_id {
    //         SENSOR_REPORTID_ROTATION_VECTOR => {
    //             //iprintln!("SENSOR_REPORTID_ROTATION_VECTOR").unwrap();
    //         },
    //         _ => {
    //             //iprintln!("handle_input_report[{}]: 0x{:01x} ", received_len, feature_report_id).unwrap();
    //         }
    //     }
    // }

    // fn handle_advertise_response(&mut self, received_len: usize) {
    //     let payload_len = received_len - PACKET_HEADER_LENGTH;
    //     let payload = &self.packet_recv_buf[PACKET_HEADER_LENGTH..received_len];
    //     let mut cursor:usize = 1; //skip response type
    //
    //     while cursor < payload_len {
    //         let _tag: u8 = payload[cursor]; cursor += 1;
    //         let len: u8 = payload[cursor]; cursor +=1;
    //         //let val: u8 = payload + cursor;
    //         cursor += len as usize;
    //     }
    //
    // }

    // /// Send a standard packet header followed by the body data provided
    // fn send_packet(&mut self, channel: u8, body_data: &[u8]) -> Result<usize, Error<E>> {
    //     let body_len = body_data.len();
    //
    //     self.sequence_numbers[channel as usize] += 1;
    //     let packet_length = body_len + PACKET_HEADER_LENGTH;
    //     let packet_header = [
    //         (packet_length & 0xFF) as u8, //LSB
    //         packet_length.shr(8) as u8, //MSB
    //         channel,
    //         self.sequence_numbers[channel as usize]
    //     ];
    //
    //     self.packet_send_buf[..PACKET_HEADER_LENGTH].copy_from_slice(packet_header.as_ref());
    //     self.packet_send_buf[PACKET_HEADER_LENGTH..packet_length].copy_from_slice(body_data);
    //     self.i2c_port.write(self.address, &self.packet_send_buf[..packet_length]).map_err(Error::I2c)?;
    //     Ok(packet_length)
    // }

    // Read one packet into the receive buffer
    // fn receive_packet(&mut self) -> Result<usize, Error<E>> {
    //     let packet_len:usize = self.read_unsized_packet()?;
    //
    //     if  packet_len > 0 {
    //         self.received_packet_count += 1;
    //     }
    //
    //     let received_len =
    //         if packet_len > PACKET_HEADER_LENGTH {
    //             self.read_sized_packet(packet_len)?
    //         }
    //         else {
    //             packet_len
    //         };
    //
    //     Ok(received_len)
    // }

    // Send a packet and receive the response
    // fn send_and_receive_packet(&mut self, channel: u8, body_data: &[u8]) ->  Result<usize, Error<E>> {
    //     //TODO reimplement with WriteRead once that interface is stable
    //     self.send_packet(channel, body_data)?;
    //     self.receive_packet()
    // }

    // check for a valid product ID response from sensor
    // fn verify_product_id(&mut self) -> Result<(), Error<E>> {
    //     let cmd_body: [u8; 2] = [
    //         SENSORHUB_PROD_ID_REQ, //request product ID
    //         0, //reserved
    //         ];
    //
    //     let recv_len = self.send_and_receive_packet(CHANNEL_HUB_CONTROL, cmd_body.as_ref())?;
    //
    //     //verify the response
    //     if recv_len > PACKET_HEADER_LENGTH {
    //         //iprintln!("resp: {:?}", &self.msg_buf[..recv_len]).unwrap();
    //         let report_id = self.packet_recv_buf[PACKET_HEADER_LENGTH + 0];
    //         if SENSORHUB_PROD_ID_RESP == report_id {
    //             self.prod_id_verified = true;
    //             return Ok(())
    //         }
    //     }
    //
    //     return Err(Error::InvalidChipId(0));
    // }


   // /// Send a soft reset command to the sensor
   // pub fn soft_reset(&mut self) -> Result<(), Error<E>> {
   //     let data:[u8; 1] = [EXECUTABLE_DEVICE_CMD_RESET]; //reset execute
   //     // send command packet and ignore received packets
   //     self.send_packet(CHANNEL_EXECUTABLE, data.as_ref())?;
   //     //ignore the response because the size is wacky
   //     self.read_unsized_packet()?;
   //     Ok(())
   // }

    // /// Read just the first header bytes of a packet
    // /// Return the total size of the packet that follows
    // fn read_unsized_packet(&mut self) -> Result<usize, Error<E>> {
    //     self.seg_recv_buf[0] = 0;
    //     self.seg_recv_buf[1] = 0;
    //     self.i2c_port.read(self.address, &mut self.seg_recv_buf[..PACKET_HEADER_LENGTH]).map_err(Error::I2c)?;
    //     let packet_len = Self::parse_packet_header(&self.seg_recv_buf[..PACKET_HEADER_LENGTH]);
    //     Ok(packet_len)
    // }


    // /// Read the remainder of the packet after the packet header, if any
    // fn read_sized_packet(&mut self, total_packet_len: usize) -> Result<usize, Error<E>> {
    //     //iprintln!("sized: {}", total_packet_len).unwrap();
    //     let mut remaining_body_len: usize = total_packet_len - PACKET_HEADER_LENGTH;
    //     let mut already_read_len: usize = 0;
    //
    //     if total_packet_len < MAX_SEGMENT_READ {
    //         if total_packet_len > 0 {
    //             self.packet_recv_buf[0] = 0;
    //             self.packet_recv_buf[1] = 0;
    //             self.i2c_port.read(self.address, &mut self.packet_recv_buf[..total_packet_len]).map_err(Error::I2c)?;
    //             //let packet_declared_len = Self::parse_packet_header(&self.packet_recv_buf[..PACKET_HEADER_LENGTH]);
    //             already_read_len = total_packet_len;
    //         }
    //     }
    //     else {
    //         while remaining_body_len > 0 {
    //             let whole_segment_length = remaining_body_len + PACKET_HEADER_LENGTH;
    //             let segment_read_len =
    //                 if whole_segment_length > MAX_SEGMENT_READ { MAX_SEGMENT_READ }
    //                 else { whole_segment_length };
    //
    //             self.seg_recv_buf[0] = 0;
    //             self.seg_recv_buf[1] = 0;
    //             self.i2c_port.read(self.address, &mut self.seg_recv_buf[..segment_read_len]).map_err(Error::I2c)?;
    //             //let packet_declared_len = Self::parse_packet_header(&self.seg_recv_buf[..PACKET_HEADER_LENGTH]);
    //
    //             //if we've never read any segments, transcribe the first packet header;
    //             //otherwise, just transcribe the segment body (no header)
    //             let transcribe_start_idx = if already_read_len > 0 { PACKET_HEADER_LENGTH } else { 0 };
    //             let transcribe_len = if already_read_len > 0 { segment_read_len - PACKET_HEADER_LENGTH } else { segment_read_len };
    //             self.packet_recv_buf[already_read_len..already_read_len+transcribe_len].
    //                 copy_from_slice(&self.seg_recv_buf[transcribe_start_idx..transcribe_start_idx+transcribe_len]);
    //             already_read_len += transcribe_len;
    //
    //             let body_read_len = segment_read_len - PACKET_HEADER_LENGTH;
    //             remaining_body_len -= body_read_len;
    //
    //         }
    //     }
    //
    //     Ok(already_read_len)
    // }

// }


// /// length of packet headers
// const PACKET_HEADER_LENGTH: usize = 4;
//
// // The BNO080 supports six communication channels:
// const  SHTP_CHAN_COMMAND: u8 = 0; /// the SHTP command channel
// const  CHANNEL_EXECUTABLE: u8 = 1; /// executable channel
// const  CHANNEL_HUB_CONTROL: u8 = 2; /// sensor hub control channel
// const  CHANNEL_SENSOR_REPORTS: u8 = 3; /// input sensor reports (non-wake, not gyroRV)
// //const  CHANNEL_WAKE_REPORTS: usize = 4; /// wake input sensor reports (for sensors configured as wake up sensors)
// //const  CHANNEL_GYRO_ROTATION: usize = 5; ///  gyro rotation vector (gyroRV)



// /// SHTP constants
// const SENSORHUB_PROD_ID_REQ: u8 = 0xF9;
// const SENSORHUB_PROD_ID_RESP: u8 =  0xF8;
//
//
// const SHTP_REPORT_SET_FEATURE_COMMAND: u8 = 0xFD;
//
// const SENSOR_REPORTID_ROTATION_VECTOR: u8 = 0x05;
//
// /// requests
// //const SENSORHUB_COMMAND_REQ:u8 =  0xF2;
// const SENSORHUB_COMMAND_RESP:u8 = 0xF1;


// executable/device channel responses
// Figure 1-27: SHTP executable commands and response
// const EXECUTABLE_DEVICE_CMD_UNKNOWN: u8 =  0;
// const EXECUTABLE_DEVICE_CMD_RESET: u8 =  1;
//const EXECUTABLE_DEVICE_CMD_ON: u8 =   2;
//const EXECUTABLE_DEVICE_CMD_SLEEP =  3;

// /// Response to CMD_RESET
// const EXECUTABLE_DEVICE_RESP_RESET_COMPLETE: u8 = 1;

//const SH2_OK: i8 = 0; /// Success
//const SH2_ERR: i8 = -1; ///  General Error

//#define SH2_ERR_BAD_PARAM      (-2) /**< Bad parameter to an API call */
//#define SH2_ERR_OP_IN_PROGRESS (-3) /**< Operation in progress */
//#define SH2_ERR_IO             (-4) /**< Error communicating with hub */
//#define SH2_ERR_HUB            (-5) /**< Error reported by hub */
//#define SH2_ERR_TIMEOUT        (-6) /**< Operation timed out */


// /// Commands and subcommands
// const SH2_INIT_UNSOLICITED: u8 = 0x80;
// const SH2_CMD_INITIALIZE: u8 = 4;
// //const SH2_INIT_SYSTEM: u8 = 1;
// const SH2_STARTUP_INIT_UNSOLICITED:u8 = SH2_CMD_INITIALIZE | SH2_INIT_UNSOLICITED;



