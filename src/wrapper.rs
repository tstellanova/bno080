/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

use crate::interface::{SensorInterface, PACKET_HEADER_LENGTH};
use embedded_hal::blocking::delay::DelayMs;

use core::ops::Shr;

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

const PACKET_SEND_BUF_LEN: usize = 256;
const PACKET_RECV_BUF_LEN: usize = 1024;

const NUM_CHANNELS: usize = 6;

#[derive(Debug)]
pub enum WrapperError<E> {
    ///Communications error
    CommError(E),
    /// Invalid chip ID was read
    InvalidChipId(u8),
    /// Unsupported sensor firmware version
    InvalidFWVersion(u8),
    /// We expected some data but didn't receive any
    NoDataAvailable,
}

pub struct BNO080<SI> {
    pub(crate) sensor_interface: SI,
    /// each communication channel with the device has its own sequence number
    sequence_numbers: [u8; NUM_CHANNELS],
    /// buffer for building and sending packet to the sensor hub
    packet_send_buf: [u8; PACKET_SEND_BUF_LEN],
    /// buffer for building packets received from the sensor hub
    packet_recv_buf: [u8; PACKET_RECV_BUF_LEN],

    last_packet_len_received: usize,
    /// has the device been succesfully reset
    device_reset: bool,
    /// has the product ID been verified
    prod_id_verified: bool,

    init_received: bool,

    /// have we received the full advertisement
    advert_received: bool,

    /// have we received an error list
    error_list_received: bool,
    last_error_received: u8,

    last_chan_received: u8,
    last_exec_chan_rid: u8,
    last_command_chan_rid: u8,

    /// Rotation vector as unit quaternion
    rotation_quaternion: [f32; 4],
    /// Heading accuracy of rotation vector (radians)
    rot_quaternion_acc: f32,
}

impl<SI> BNO080<SI> {
    pub fn new_with_interface(sensor_interface: SI) -> Self {
        Self {
            sensor_interface,
            sequence_numbers: [0; NUM_CHANNELS],
            packet_send_buf: [0; PACKET_SEND_BUF_LEN],
            packet_recv_buf: [0; PACKET_RECV_BUF_LEN],
            last_packet_len_received: 0,
            device_reset: false,
            prod_id_verified: false,
            init_received: false,
            advert_received: false,
            error_list_received: false,
            last_error_received: 0,
            last_chan_received: 0,
            last_exec_chan_rid: 0,
            last_command_chan_rid: 0,
            rotation_quaternion: [0.0; 4],
            rot_quaternion_acc: 0.0,
        }
    }
}

impl<SI, SE> BNO080<SI>
where
    SI: SensorInterface<SensorError = SE>,
    SE: core::fmt::Debug,
{
    /// Consume all available messages on the port without processing them
    pub fn eat_all_messages(&mut self, delay: &mut impl DelayMs<u8>) {
        loop {
            let msg_count = self.eat_one_message();
            if msg_count == 0 {
                break;
            } else {
                //give some time to other parts of the system
                delay.delay_ms(1);
            }
        }
    }

    pub fn handle_all_messages(&mut self, delay: &mut dyn DelayMs<u8>) -> u32 {
        let mut total_handled: u32 = 0;
        loop {
            let handled_count = self.handle_one_message();
            if handled_count == 0 {
                break;
            } else {
                total_handled += handled_count;
                //give some time to other parts of the system
                delay.delay_ms(1);
            }
        }
        total_handled
    }

    /// return the number of messages handled
    pub fn handle_one_message(&mut self) -> u32 {
        let mut msg_count = 0;

        let res = self.receive_packet();
        if res.is_ok() {
            let received_len = res.unwrap_or(0);
            if received_len > 0 {
                msg_count += 1;
                self.handle_received_packet(received_len);
            }
        } else {
            #[cfg(feature = "rttdebug")]
            rprintln!("handle1 err {:?}", res);
        }

        msg_count
    }

    /// Receive and ignore one message,
    /// returning the size of the packet received or zero
    /// if there was no packet to read.
    pub fn eat_one_message(&mut self) -> usize {
        let res = self.receive_packet();
        return if let Ok(received_len) = res {
            #[cfg(feature = "rttdebug")]
            rprintln!("e1 {}", received_len);
            received_len
        } else {
            #[cfg(feature = "rttdebug")]
            rprintln!("eat1 err {:?}", res);
            0
        };
    }

    fn handle_advertise_response(&mut self, received_len: usize) {
        let payload_len = received_len - PACKET_HEADER_LENGTH;
        let payload = &self.packet_recv_buf[PACKET_HEADER_LENGTH..received_len];
        let mut cursor: usize = 1; //skip response type

        #[cfg(feature = "rttdebug")]
        rprintln!("AdvRsp: {}", payload_len);

        while cursor < payload_len {
            let _tag: u8 = payload[cursor];
            cursor += 1;
            let len: u8 = payload[cursor];
            cursor += 1;
            //let val: u8 = payload + cursor;
            cursor += len as usize;
        }

        self.advert_received = true;
    }

    fn read_u8_at_cursor(msg: &[u8], cursor: &mut usize) -> u8 {
        let val = msg[*cursor];
        *cursor += 1;
        val
    }

    fn read_i16_at_cursor(msg: &[u8], cursor: &mut usize) -> i16 {
        let val = (msg[*cursor] as i16) | ((msg[*cursor + 1] as i16) << 8);
        *cursor += 2;
        val
    }

    /// Read data values from a single input report
    fn handle_one_input_report(
        outer_cursor: usize,
        msg: &[u8],
    ) -> (usize, u8, i16, i16, i16, i16, i16) {
        let mut cursor = outer_cursor;
        let remaining = msg.len() - cursor;

        let feature_report_id = Self::read_u8_at_cursor(msg, &mut cursor);
        let _rep_seq_num = Self::read_u8_at_cursor(msg, &mut cursor);
        let _rep_status = Self::read_u8_at_cursor(msg, &mut cursor);
        let _delay = Self::read_u8_at_cursor(msg, &mut cursor);

        let data1: i16 = Self::read_i16_at_cursor(msg, &mut cursor);
        let data2: i16 = Self::read_i16_at_cursor(msg, &mut cursor);
        let data3: i16 = Self::read_i16_at_cursor(msg, &mut cursor);
        let data4: i16 = if remaining > 14 {
            Self::read_i16_at_cursor(msg, &mut cursor)
        } else {
            0
        };
        let data5: i16 = if remaining > 16 {
            Self::read_i16_at_cursor(msg, &mut cursor)
        } else {
            0
        };

        (cursor, feature_report_id, data1, data2, data3, data4, data5)
    }

    /// Handle parsing of an input report packet,
    /// which may include multiple input reports
    fn handle_sensor_reports(&mut self, received_len: usize) {
        // Sensor input packets have the form:
        // [u8; 5]  timestamp in microseconds for the packet?
        // a sequence of n reports, each with four byte header
        // u8 report ID
        // u8 sequence number of report

        // let mut report_count = 0;
        let mut outer_cursor: usize = PACKET_HEADER_LENGTH + 5; //skip header, timestamp
                                                                //TODO need to skip more above for a payload-level timestamp??
        if received_len < outer_cursor {
            #[cfg(feature = "rttdebug")]
            rprintln!("bad lens: {} < {}", received_len, outer_cursor);
            return;
        }

        let payload_len = received_len - outer_cursor;
        if payload_len < 14 {
            #[cfg(feature = "rttdebug")]
            rprintln!(
                "bad report: {:?}",
                &self.packet_recv_buf[..PACKET_HEADER_LENGTH]
            );

            return;
        }

        // there may be multiple reports per payload
        while outer_cursor < payload_len {
            //let start_cursor = outer_cursor;
            let (inner_cursor, report_id, data1, data2, data3, data4, data5) =
                Self::handle_one_input_report(
                    outer_cursor,
                    &self.packet_recv_buf[..received_len],
                );
            outer_cursor = inner_cursor;
            // report_count += 1;
            match report_id {
                SENSOR_REPORTID_ROTATION_VECTOR => {
                    self.update_rotation_quaternion(
                        data1, data2, data3, data4, data5,
                    );
                }
                _ => {
                    // debug_println!("uhr: {:X}", report_id);
                    // debug_println!("uhr: 0x{:X} {:?}  ", report_id, &self.packet_recv_buf[start_cursor..start_cursor+5]);
                }
            }
        }

        //debug_println!("report_count: {}",report_count);
    }

    /// Given a set of quaternion values in the Q-fixed-point format,
    /// calculate and update the corresponding float values
    fn update_rotation_quaternion(
        &mut self,
        q_i: i16,
        q_j: i16,
        q_k: i16,
        q_r: i16,
        q_a: i16,
    ) {
        //debug_println!("rquat {} {} {} {} {}", q_i, q_j, q_k, q_r, q_a);
        self.rotation_quaternion = [
            q14_to_f32(q_i),
            q14_to_f32(q_j),
            q14_to_f32(q_k),
            q14_to_f32(q_r),
        ];
        self.rot_quaternion_acc = q12_to_f32(q_a);
    }

    /// Handle one or more errors sent in response to a command
    fn handle_cmd_resp_error_list(&mut self, received_len: usize) {
        let payload_len = received_len - PACKET_HEADER_LENGTH;
        let payload = &self.packet_recv_buf[PACKET_HEADER_LENGTH..received_len];

        self.error_list_received = true;
        for cursor in 1..payload_len {
            let err: u8 = payload[cursor];
            self.last_error_received = err;
            #[cfg(feature = "rttdebug")]
            rprintln!("lerr: {:x}", err);
        }
    }

    pub fn handle_received_packet(&mut self, received_len: usize) {
        let msg = &self.packet_recv_buf[..received_len];
        let chan_num = msg[2];
        //let _seq_num =  msg[3];
        let report_id: u8 = if received_len > PACKET_HEADER_LENGTH {
            msg[4]
        } else {
            0
        };

        self.last_chan_received = chan_num;
        match chan_num {
            CHANNEL_COMMAND => match report_id {
                CMD_RESP_ADVERTISEMENT => {
                    self.handle_advertise_response(received_len);
                }
                CMD_RESP_ERROR_LIST => {
                    self.handle_cmd_resp_error_list(received_len);
                }
                _ => {
                    self.last_command_chan_rid = report_id;
                    #[cfg(feature = "rttdebug")]
                    rprintln!("unh cmd: {}", report_id);
                }
            },
            CHANNEL_EXECUTABLE => match report_id {
                EXECUTABLE_DEVICE_RESP_RESET_COMPLETE => {
                    self.device_reset = true;
                    #[cfg(feature = "rttdebug")]
                    rprintln!("resp_reset {}", 1);
                }
                _ => {
                    self.last_exec_chan_rid = report_id;
                    #[cfg(feature = "rttdebug")]
                    rprintln!("unh exe: {:x}", report_id);
                }
            },
            CHANNEL_HUB_CONTROL => {
                match report_id {
                    SHUB_COMMAND_RESP => {
                        // 0xF1 / 241
                        let cmd_resp = msg[6];
                        if cmd_resp == SH2_STARTUP_INIT_UNSOLICITED {
                            self.init_received = true;
                        } else if cmd_resp == SH2_INIT_SYSTEM {
                            self.init_received = true;
                        }
                        #[cfg(feature = "rttdebug")]
                        rprintln!("CMD_RESP: 0x{:X}", cmd_resp);
                    }
                    SHUB_PROD_ID_RESP => {
                        #[cfg(feature = "rttdebug")]
                        rprintln!("PID_RESP {}", 1);
                        self.prod_id_verified = true;
                    }
                    SHUB_GET_FEATURE_RESP => {
                        // 0xFC
                        #[cfg(feature = "rttdebug")]
                        rprintln!("feat resp: {}", msg[5]);
                    }
                    _ => {
                        #[cfg(feature = "rttdebug")]
                        rprintln!(
                            "unh hbc: 0x{:X} {:?}",
                            report_id,
                            &msg[..PACKET_HEADER_LENGTH]
                        );
                    }
                }
            }
            CHANNEL_SENSOR_REPORTS => {
                self.handle_sensor_reports(received_len);
            }
            _ => {
                self.last_chan_received = chan_num;
                #[cfg(feature = "rttdebug")]
                rprintln!("unh chan 0x{:X}", chan_num);
            }
        }
    }

    /// The BNO080 starts up with all sensors disabled,
    /// waiting for the application to configure it.
    pub fn init(
        &mut self,
        delay_source: &mut impl DelayMs<u8>,
    ) -> Result<(), WrapperError<SE>> {
        #[cfg(feature = "rttdebug")]
        rprintln!("wrapper init");
        //Section 5.1.1.1 : On system startup, the SHTP control application will send
        // its full advertisement response, unsolicited, to the host.
        self.sensor_interface
            .setup(delay_source)
            .map_err(WrapperError::CommError)?;
        delay_source.delay_ms(1u8);
        if self.sensor_interface.requires_soft_reset() {
            self.soft_reset()?;
        }

        delay_source.delay_ms(50u8);
        self.eat_all_messages(delay_source);
        delay_source.delay_ms(100u8);
        self.eat_all_messages(delay_source);

        self.verify_product_id()?;
        self.eat_all_messages(delay_source);

        Ok(())
    }

    /// Tell the sensor to start reporting the fused rotation vector
    /// on a regular cadence. Note that the maximum valid update rate
    /// is 1 kHz, based on the max update rate of the sensor's gyros.
    pub fn enable_rotation_vector(
        &mut self,
        millis_between_reports: u16,
    ) -> Result<(), WrapperError<SE>> {
        self.enable_report(
            SENSOR_REPORTID_ROTATION_VECTOR,
            millis_between_reports,
        )
    }

    /// Enable a particular report
    fn enable_report(
        &mut self,
        report_id: u8,
        millis_between_reports: u16,
    ) -> Result<(), WrapperError<SE>> {
        #[cfg(feature = "rttdebug")]
        rprintln!("enable_report 0x{:X}", report_id);

        let micros_between_reports: u32 =
            (millis_between_reports as u32) * 1000;
        let cmd_body: [u8; 17] = [
            SHUB_REPORT_SET_FEATURE_CMD,
            report_id,
            0,                                        //feature flags
            0,                                        //LSB change sensitivity
            0,                                        //MSB change sensitivity
            (micros_between_reports & 0xFFu32) as u8, // LSB report interval, microseconds
            (micros_between_reports.shr(8) & 0xFFu32) as u8,
            (micros_between_reports.shr(16) & 0xFFu32) as u8,
            (micros_between_reports.shr(24) & 0xFFu32) as u8, // MSB report interval
            0, // LSB Batch Interval
            0,
            0,
            0, // MSB Batch interval
            0, // LSB sensor-specific config
            0,
            0,
            0, // MSB sensor-specific config
        ];

        //we simply blast out this configuration command and assume it'll succeed
        self.send_packet(CHANNEL_HUB_CONTROL, &cmd_body)?;
        // any error or success in configuration will arrive some time later

        Ok(())
    }

    /// Prepare a packet for sending, in our send buffer
    fn prep_send_packet(&mut self, channel: u8, body_data: &[u8]) -> usize {
        let body_len = body_data.len();

        let packet_length = body_len + PACKET_HEADER_LENGTH;
        let packet_header = [
            (packet_length & 0xFF) as u8, //LSB
            packet_length.shr(8) as u8,   //MSB
            channel,
            self.sequence_numbers[channel as usize],
        ];
        self.sequence_numbers[channel as usize] += 1;

        self.packet_send_buf[..PACKET_HEADER_LENGTH]
            .copy_from_slice(packet_header.as_ref());
        self.packet_send_buf[PACKET_HEADER_LENGTH..packet_length]
            .copy_from_slice(body_data);

        packet_length
    }

    /// Send packet from our packet send buf
    fn send_packet(
        &mut self,
        channel: u8,
        body_data: &[u8],
    ) -> Result<usize, WrapperError<SE>> {
        let packet_length = self.prep_send_packet(channel, body_data);
        self.sensor_interface
            .write_packet(&self.packet_send_buf[..packet_length])
            .map_err(WrapperError::CommError)?;
        Ok(packet_length)
    }

    /// Read one packet into the receive buffer
    pub(crate) fn receive_packet(&mut self) -> Result<usize, WrapperError<SE>> {
        self.packet_recv_buf[0] = 0;
        self.packet_recv_buf[1] = 0;
        let packet_len = self
            .sensor_interface
            .read_packet(&mut self.packet_recv_buf)
            .map_err(WrapperError::CommError)?;

        self.last_packet_len_received = packet_len;
        #[cfg(feature = "rttdebug")]
        rprintln!("recv {}", packet_len);

        Ok(packet_len)
    }

    /// Verify that the sensor returns an expected chip ID
    pub fn verify_product_id(&mut self) -> Result<(), WrapperError<SE>> {
        let cmd_body: [u8; 2] = [
            SHUB_PROD_ID_REQ, //request product ID
            0,                //reserved
        ];

        let recv_len = self
            .send_and_receive_packet(CHANNEL_HUB_CONTROL, cmd_body.as_ref())?;
        if recv_len > PACKET_HEADER_LENGTH {
            self.handle_received_packet(recv_len);
        }

        if !self.prod_id_verified {
            return Err(WrapperError::InvalidChipId(0));
        }
        Ok(())
    }

    /// Read normalized quaternion:
    /// QX normalized quaternion – X, or Heading | range: 0.0 – 1.0 ( ±π )
    /// QY normalized quaternion – Y, or Pitch   | range: 0.0 – 1.0 ( ±π/2 )
    /// QZ normalized quaternion – Z, or Roll    | range: 0.0 – 1.0 ( ±π )
    /// QW normalized quaternion – W, or 0.0     | range: 0.0 – 1.0
    pub fn rotation_quaternion(&self) -> Result<[f32; 4], WrapperError<SE>> {
        Ok(self.rotation_quaternion)
    }

    pub fn heading_accuracy(&self) -> f32 {
        self.rot_quaternion_acc
    }

    /// Tell the sensor to reset.
    /// Normally applications should not need to call this directly,
    /// as it is called during `init`.
    pub fn soft_reset(&mut self) -> Result<(), WrapperError<SE>> {
        #[cfg(feature = "rttdebug")]
        rprintln!("soft_reset");
        let data: [u8; 1] = [EXECUTABLE_DEVICE_CMD_RESET];
        // send command packet and ignore received packets
        self.send_and_receive_packet(CHANNEL_EXECUTABLE, data.as_ref())?;
        Ok(())
    }

    /// Send a packet and receive the response
    fn send_and_receive_packet(
        &mut self,
        channel: u8,
        body_data: &[u8],
    ) -> Result<usize, WrapperError<SE>> {
        let send_packet_length = self.prep_send_packet(channel, body_data);
        #[cfg(feature = "rttdebug")]
        rprintln!("srcv {} ...", send_packet_length);

        let recv_packet_length = self
            .sensor_interface
            .send_and_receive_packet(
                &self.packet_send_buf[..send_packet_length].as_ref(),
                &mut self.packet_recv_buf,
            )
            .map_err(WrapperError::CommError)?;

        #[cfg(feature = "rttdebug")]
        rprintln!("srcv {} {}", send_packet_length, recv_packet_length);

        Ok(recv_packet_length)
    }
}

const Q12_SCALE: f32 = 1.0 / ((1 << 12) as f32);
const Q14_SCALE: f32 = 1.0 / ((1 << 14) as f32);

fn q14_to_f32(q_val: i16) -> f32 {
    // let qq_val =  fpa::I2F14(q_val).unwrap();
    // return f32(qq_val)
    (q_val as f32) * Q14_SCALE
}

fn q12_to_f32(q_val: i16) -> f32 {
    (q_val as f32) * Q12_SCALE
}

// The BNO080 supports six communication channels:
const CHANNEL_COMMAND: u8 = 0;
/// the SHTP command channel
const CHANNEL_EXECUTABLE: u8 = 1;
/// executable channel
const CHANNEL_HUB_CONTROL: u8 = 2;
/// sensor hub control channel
const CHANNEL_SENSOR_REPORTS: u8 = 3;
/// input sensor reports (non-wake, not gyroRV)
//const  CHANNEL_WAKE_REPORTS: usize = 4; /// wake input sensor reports (for sensors configured as wake up sensors)
//const  CHANNEL_GYRO_ROTATION: usize = 5; ///  gyro rotation vector (gyroRV)

/// Command Channel requests / responses

// Commands
//const CMD_GET_ADVERTISEMENT: u8 = 0;
//const CMD_SEND_ERROR_LIST: u8 = 1;

/// Responses
const CMD_RESP_ADVERTISEMENT: u8 = 0;
const CMD_RESP_ERROR_LIST: u8 = 1;

/// SHTP constants

/// Report ID for Product ID request
const SHUB_PROD_ID_REQ: u8 = 0xF9;
/// Report ID for Product ID response
const SHUB_PROD_ID_RESP: u8 = 0xF8;
const SHUB_GET_FEATURE_RESP: u8 = 0xFC;
const SHUB_REPORT_SET_FEATURE_CMD: u8 = 0xFD;
// const SHUB_GET_FEATURE_REQ: u8 = 0xFE;
// const SHUB_FORCE_SENSOR_FLUSH: u8 = 0xF0;
const SHUB_COMMAND_RESP: u8 = 0xF1;
//const SHUB_COMMAND_REQ:u8 =  0xF2;

/// Report IDs from SH2 Reference Manual:
// 0x01 accelerometer (m/s^2 including gravity): Q point 8
// 0x02 gyroscope calibrated (rad/s): Q point 9
// 0x03 mag field calibrated (uTesla): Q point 4
// 0x04 linear acceleration (m/s^2 minus gravity): Q point 8
/// Unit quaternion rotation vector, Q point 12, with heading accuracy estimate (radians)
const SENSOR_REPORTID_ROTATION_VECTOR: u8 = 0x05;
// const SENSOR_REPORTID_GRAVITY: u8 = 0x06; // Q point 8
// 0x08 game rotation vector : Q point 14
// 0x09 geomagnetic rotation vector: Q point 14 for quaternion, Q point 12 for heading accuracy
// 0x0A pressure (hectopascals) from external baro: Q point 20
// 0x0B ambient light (lux) from external sensor: Q point 8
// 0x0C humidity (percent) from external sensor: Q point 8
// 0x0D proximity (centimeters) from external sensor: Q point 4
// 0x0E temperature (degrees C) from external sensor: Q point 7

/// executable/device channel responses
/// Figure 1-27: SHTP executable commands and response
// const EXECUTABLE_DEVICE_CMD_UNKNOWN: u8 =  0;
const EXECUTABLE_DEVICE_CMD_RESET: u8 = 1;
//const EXECUTABLE_DEVICE_CMD_ON: u8 =   2;
//const EXECUTABLE_DEVICE_CMD_SLEEP =  3;

/// Response to CMD_RESET
const EXECUTABLE_DEVICE_RESP_RESET_COMPLETE: u8 = 1;

/// Commands and subcommands
const SH2_INIT_UNSOLICITED: u8 = 0x80;
const SH2_CMD_INITIALIZE: u8 = 4;
const SH2_INIT_SYSTEM: u8 = 1;
const SH2_STARTUP_INIT_UNSOLICITED: u8 =
    SH2_CMD_INITIALIZE | SH2_INIT_UNSOLICITED;

#[cfg(test)]
mod tests {
    // use super::*;
    use crate::interface::i2c::DEFAULT_ADDRESS;
    use crate::interface::mock_i2c_port::FakeI2cPort;
    use crate::wrapper::{q14_to_f32, BNO080, Q14_SCALE};

    use crate::interface::I2cInterface;

    fn f32_to_q14(input: f32) -> i16 {
        (input / Q14_SCALE) as i16
    }

    #[test]
    fn test_qval_conversions() {
        let q_val = f32_to_q14(0.5);
        let float_val = q14_to_f32(q_val);
        assert_eq!(float_val, 0.5);
    }

    #[test]
    fn test_foo() {
        let mut mock_i2c_port = FakeI2cPort::new();

        let packet = ADVERTISING_PACKET_FULL;
        mock_i2c_port.add_available_packet(&packet);

        let mut shub = BNO080::new_with_interface(I2cInterface::new(
            mock_i2c_port,
            DEFAULT_ADDRESS,
        ));
        let rc = shub.receive_packet();

        assert!(rc.is_ok());
        let next_packet_size = rc.unwrap_or(0);
        assert_eq!(next_packet_size, packet.len(), "wrong length");
    }

    // //TODO give access to sent packets for testing porpoises
    // #[test]
    // fn test_send_reset() {
    //     let mut mock_i2c_port = FakeI2cPort::new();
    //     let mut shub = Wrapper::new_with_interface(
    //         I2cInterface::new(mock_i2c_port, DEFAULT_ADDRESS));
    //     let rc = shub.soft_reset();
    //     let sent_pack = shub.sensor_interface.sent_packets.pop_front().unwrap();
    //     assert_eq!(sent_pack.len, 5);
    // }

    #[test]
    fn test_handle_adv_message() {
        let mut mock_i2c_port = FakeI2cPort::new();

        //actual startup response packet
        let raw_packet = ADVERTISING_PACKET_FULL;
        mock_i2c_port.add_available_packet(&raw_packet);

        let mut shub = BNO080::new_with_interface(I2cInterface::new(
            mock_i2c_port,
            DEFAULT_ADDRESS,
        ));

        let msg_count = shub.handle_one_message();
        assert_eq!(msg_count, 1, "wrong msg_count");
    }

    // Actual advertising packet received from sensor:
    pub const ADVERTISING_PACKET_FULL: [u8; 276] = [
        0x14, 0x81, 0x00, 0x01, 0x00, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x80,
        0x06, 0x31, 0x2e, 0x30, 0x2e, 0x30, 0x00, 0x02, 0x02, 0x00, 0x01, 0x03,
        0x02, 0xff, 0x7f, 0x04, 0x02, 0x00, 0x01, 0x05, 0x02, 0xff, 0x7f, 0x08,
        0x05, 0x53, 0x48, 0x54, 0x50, 0x00, 0x06, 0x01, 0x00, 0x09, 0x08, 0x63,
        0x6f, 0x6e, 0x74, 0x72, 0x6f, 0x6c, 0x00, 0x01, 0x04, 0x01, 0x00, 0x00,
        0x00, 0x08, 0x0b, 0x65, 0x78, 0x65, 0x63, 0x75, 0x74, 0x61, 0x62, 0x6c,
        0x65, 0x00, 0x06, 0x01, 0x01, 0x09, 0x07, 0x64, 0x65, 0x76, 0x69, 0x63,
        0x65, 0x00, 0x01, 0x04, 0x02, 0x00, 0x00, 0x00, 0x08, 0x0a, 0x73, 0x65,
        0x6e, 0x73, 0x6f, 0x72, 0x68, 0x75, 0x62, 0x00, 0x06, 0x01, 0x02, 0x09,
        0x08, 0x63, 0x6f, 0x6e, 0x74, 0x72, 0x6f, 0x6c, 0x00, 0x06, 0x01, 0x03,
        0x09, 0x0c, 0x69, 0x6e, 0x70, 0x75, 0x74, 0x4e, 0x6f, 0x72, 0x6d, 0x61,
        0x6c, 0x00, 0x07, 0x01, 0x04, 0x09, 0x0a, 0x69, 0x6e, 0x70, 0x75, 0x74,
        0x57, 0x61, 0x6b, 0x65, 0x00, 0x06, 0x01, 0x05, 0x09, 0x0c, 0x69, 0x6e,
        0x70, 0x75, 0x74, 0x47, 0x79, 0x72, 0x6f, 0x52, 0x76, 0x00, 0x80, 0x06,
        0x31, 0x2e, 0x31, 0x2e, 0x30, 0x00, 0x81, 0x64, 0xf8, 0x10, 0xf5, 0x04,
        0xf3, 0x10, 0xf1, 0x10, 0xfb, 0x05, 0xfa, 0x05, 0xfc, 0x11, 0xef, 0x02,
        0x01, 0x0a, 0x02, 0x0a, 0x03, 0x0a, 0x04, 0x0a, 0x05, 0x0e, 0x06, 0x0a,
        0x07, 0x10, 0x08, 0x0c, 0x09, 0x0e, 0x0a, 0x08, 0x0b, 0x08, 0x0c, 0x06,
        0x0d, 0x06, 0x0e, 0x06, 0x0f, 0x10, 0x10, 0x05, 0x11, 0x0c, 0x12, 0x06,
        0x13, 0x06, 0x14, 0x10, 0x15, 0x10, 0x16, 0x10, 0x17, 0x00, 0x18, 0x08,
        0x19, 0x06, 0x1a, 0x00, 0x1b, 0x00, 0x1c, 0x06, 0x1d, 0x00, 0x1e, 0x10,
        0x1f, 0x00, 0x20, 0x00, 0x21, 0x00, 0x22, 0x00, 0x23, 0x00, 0x24, 0x00,
        0x25, 0x00, 0x26, 0x00, 0x27, 0x00, 0x28, 0x0e, 0x29, 0x0c, 0x2a, 0x0e,
    ];
}
