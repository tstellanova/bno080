use crate::interface::{
    SensorInterface,
    PACKET_HEADER_LENGTH};
use embedded_hal::{
    blocking::delay::{ DelayMs},
};

use core::ops::{Shl, Shr};
// use core::fmt::Write;
use cortex_m::asm::bkpt;

#[cfg(debug_assertions)]
use cortex_m_semihosting::{hprint, hprintln};

const PACKET_SEND_BUF_LEN: usize = 256;
const PACKET_RECV_BUF_LEN: usize = 1024;

const NUM_CHANNELS: usize = 6;

#[derive(Debug)]
pub enum WrapperError<E> {

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
    last_control_chan_rid: u8,

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
            last_control_chan_rid: 0,
        }
    }
}

impl<SI, SE> BNO080<SI>
    where
        SI: SensorInterface<SensorError = SE>,
{
    /// Receive and ignore one message
    pub fn eat_one_message(&mut self) -> usize {
        let res = self.receive_packet();
        res.unwrap_or(0)
    }

    /// Consume all available messages on the port without processing them
    pub fn eat_all_messages(&mut self, delay: &mut dyn DelayMs<u8>) {
        let mut miss_count = 0;
        while miss_count < 10 {
            let received_len = self.eat_one_message();
            if received_len == 0 {
                miss_count += 1;
                delay.delay_ms(2);
            } else {
                //give some time to other parts of the system
                delay.delay_ms(1);
            }
        }
    }

    pub fn handle_all_messages(&mut self, delay: &mut dyn DelayMs<u8>) {
        loop {
            let handled_count = self.handle_one_message();
            if handled_count == 0 {
                break;
            } else {
                //give some time to other parts of the system
                delay.delay_ms(1);
            }
        }
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
        }

        msg_count
    }


    fn handle_advertise_response(&mut self, received_len: usize) {
        let payload_len = received_len - PACKET_HEADER_LENGTH;
        let payload = &self.packet_recv_buf[PACKET_HEADER_LENGTH..received_len];
        let mut cursor:usize = 1; //skip response type

        hprintln!("AdvRsp: {}", payload_len).unwrap();
        while cursor < payload_len {
            let _tag: u8 = payload[cursor]; cursor += 1;
            let len: u8 = payload[cursor]; cursor +=1;
            //let val: u8 = payload + cursor;
            cursor += len as usize;
        }

        self.advert_received = true;
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
                hprintln!("rotv").unwrap();
            },
            _ => {
                hprintln!("unhin[{}]: {:x} ", received_len, feature_report_id).unwrap();
            }
        }
    }

    fn handle_error_list(&mut self, received_len: usize) {
        let payload_len = received_len - PACKET_HEADER_LENGTH;
        let payload = &self.packet_recv_buf[PACKET_HEADER_LENGTH..received_len];

        self.error_list_received = true;
        for cursor in 1..payload_len {
            let err: u8 = payload[cursor];
            self.last_error_received = err;
            hprintln!("lerr: {:x}", err).unwrap();
        }
    }

    pub fn handle_received_packet(&mut self, received_len: usize) {
        let msg = &self.packet_recv_buf[..received_len];
        let chan_num =  msg[2];
        //let _seq_num =  msg[3];
        let report_id: u8 = msg[4];

        self.last_chan_received = chan_num;
        match chan_num {

            CHANNEL_COMMAND => {
                match report_id {
                    CMD_RESP_ADVERTISEMENT => {
                        self.handle_advertise_response(received_len);
                    },
                    CMD_RESP_ERROR_LIST => {
                        self.handle_error_list(received_len);
                    },
                    _ => {
                        self.last_command_chan_rid = report_id;
                        hprintln!("unh cmd: {}", report_id).unwrap();
                    }
                }
            },
            CHANNEL_EXECUTABLE => {
                match report_id {
                    EXECUTABLE_DEVICE_RESP_RESET_COMPLETE => {
                        self.device_reset = true;
                        hprintln!("resp_reset").unwrap();
                    },
                    _ => {
                        self.last_exec_chan_rid = report_id;
                        hprintln!("unh exe: {:x}", report_id).unwrap();
                    }
                }
            },
            CHANNEL_HUB_CONTROL => {
                match report_id {
                    SENSORHUB_COMMAND_RESP => { // 0xF1 / 241
                        let cmd_resp = msg[6];
                        if cmd_resp == SH2_STARTUP_INIT_UNSOLICITED {
                            self.init_received = true;
                        }
                        else if cmd_resp == SH2_INIT_SYSTEM {
                            self.init_received = true;
                        }
                        hprintln!("CMD_RESP: {:x}", cmd_resp).unwrap();
                    },
                    SENSORHUB_PROD_ID_RESP => { // 0xF8 / 248
                        hprintln!("PID_RESP").unwrap();
                        self.prod_id_verified = true;
                    },
                    _ =>  {
                        self.last_control_chan_rid = report_id;
                        hprintln!("unh hbc: {:x}", report_id).unwrap();
                    }
                }
            },
            CHANNEL_SENSOR_REPORTS => {
                self.handle_input_report(received_len);
            },
            _ => {
                self.last_chan_received = chan_num;
                hprintln!("unh chan {:x}", chan_num).unwrap();
            }
        }

    }

    /// The BNO080 starts up with all sensors disabled,
    /// waiting for the application to configure it.
    pub fn init(&mut self, delay_source: &mut impl DelayMs<u8>) -> Result<(), WrapperError<SE>> {
        //Section 5.1.1.1 : On system startup, the SHTP control application will send
        // its full advertisement response, unsolicited, to the host.

        self.sensor_interface.setup( delay_source).map_err(WrapperError::CommError)?;
        self.soft_reset(delay_source)?;

        delay_source.delay_ms(50u8);
        self.handle_all_messages(delay_source);
        delay_source.delay_ms(50u8);
        self.handle_all_messages(delay_source);

        //self.eat_all_messages(delay_source);
        //delay_source.delay_ms(50);
        //self.handle_all_messages(delay_source);
        
        self.verify_product_id(delay_source)?;

        Ok(())
    }


    // pub fn set_debug_log(&mut self, dbglog: &mut impl Printer) {
    //     unimplemented!()
    // }

    /// Tell the sensor to start reporting the fused rotation vector
    /// on a regular cadence. Note that the maximum valid update rate
    /// is 1 kHz, based on the max update rate of the sensor's gyros.
    pub fn enable_rotation_vector(&mut self, millis_between_reports: u16) -> Result<(), WrapperError<SE>> {
        self.enable_report(SENSOR_REPORTID_ROTATION_VECTOR, millis_between_reports)
    }

    /// Enable a particular report
    fn enable_report(&mut self, report_id: u8, millis_between_reports: u16) -> Result<(), WrapperError<SE>> {
        hprintln!("enable_report {}", report_id).unwrap();
        let micros_between_reports: u32 = (millis_between_reports as u32) * 1000;
        let cmd_body: [u8; 17] = [
            SHTP_REPORT_SET_FEATURE_COMMAND,
            report_id,
            0, //feature flags
            0, //LSB change sensitivity
            0, //MSB change sensitivity
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

        //self.send_and_receive_packet(CHANNEL_HUB_CONTROL, &cmd_body)?;
        self.send_packet(CHANNEL_HUB_CONTROL, &cmd_body)?;
        Ok(())
    }

    /// Prepare a packet for sending, in our send buffer
    fn prep_send_packet(&mut self, channel: u8, body_data: &[u8]) -> usize {
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

        packet_length
    }

    fn send_packet(&mut self, channel: u8, body_data: &[u8]) -> Result<usize, WrapperError<SE>> {
        let packet_length = self.prep_send_packet(channel, body_data);
        self.sensor_interface
            .write_packet( &self.packet_send_buf[..packet_length])
            .map_err(WrapperError::CommError)?;
        Ok(packet_length)
    }

    /// Read one packet into the receive buffer
    pub fn receive_packet(&mut self) -> Result<usize, WrapperError<SE>> {
        self.packet_recv_buf[0] = 0;
        self.packet_recv_buf[1] = 0;
        
        let packet_len = self.sensor_interface
            .read_packet(&mut self.packet_recv_buf)
            .map_err(WrapperError::CommError)?;

        self.last_packet_len_received = packet_len;


        // if self.debug_func.is_some() {
        //     let blob: u32 =
        //         (self.packet_recv_buf[0] as u32).shl(24)
        //         + (self.packet_recv_buf[1] as u32).shl(16)
        //         + (self.packet_recv_buf[2] as u32).shl(8)
        //         + (self.packet_recv_buf[3] as u32);
        //     self.debug_func.unwrap()(blob as usize);
        // }

        Ok(packet_len)
    }

    fn verify_product_id(&mut self, delay_source: &mut impl DelayMs<u8>) -> Result<(), WrapperError<SE> > {
        let cmd_body: [u8; 2] = [
            SENSORHUB_PROD_ID_REQ, //request product ID
            0, //reserved
        ];

        let recv_len = self.send_and_receive_packet(CHANNEL_HUB_CONTROL, cmd_body.as_ref(), delay_source)?;
        if recv_len > 0 {
            self.handle_received_packet(recv_len);
        }

        if !self.prod_id_verified {
            return Err(WrapperError::InvalidChipId(0));
        }
        Ok(())

    }

    /// Read normalized quaternion
    /// QX normalized quaternion – X, or Heading | range: 0.0 – 1.0 ( ±π )
    /// QY normalized quaternion – Y, or Pitch   | range: 0.0 – 1.0 ( ±π/2 )
    /// QZ normalized quaternion – Z, or Roll    | range: 0.0 – 1.0 ( ±π )
    /// QW normalized quaternion – W, or 0.0     | range: 0.0 – 1.0
    pub fn read_quaternion(&mut self) ->  Result<[f32; 4], WrapperError<SE>> {
        Ok([0.1, 0.2, 0.3, 0.4])
    }

    pub fn soft_reset(&mut self,  delay_source: &mut impl DelayMs<u8>) -> Result<(), WrapperError<SE>> {
        let data:[u8; 1] = [EXECUTABLE_DEVICE_CMD_RESET]; //reset execute
        // send command packet and ignore received packets
        // self.send_packet(CHANNEL_EXECUTABLE, data.as_ref())?;
        let _received = self.send_and_receive_packet(CHANNEL_EXECUTABLE, data.as_ref(), delay_source);
        Ok(())
    }

    /// Send a packet and receive the response
    fn send_and_receive_packet(&mut self, channel: u8, body_data: &[u8], delay_source: &mut impl DelayMs<u8>) ->  Result<usize, WrapperError<SE>> {
        //self.send_packet(channel, &body_data)?;
        //self.sensor_interface.wait_for_data_available(150, delay_source);
        //let recv_packet_length = self.sensor_interface.read_packet(&mut self.packet_recv_buf).map_err(WrapperError::CommError)?;

        let send_packet_length = self.prep_send_packet(channel, body_data);
        let recv_packet_length = self.sensor_interface
            .send_and_receive_packet(
                &self.packet_send_buf[..send_packet_length].as_ref(),
                &mut self.packet_recv_buf,
                delay_source)
            .map_err(WrapperError::CommError)?;
        Ok(recv_packet_length)
    }
}

// The BNO080 supports six communication channels:
const CHANNEL_COMMAND: u8 = 0; /// the SHTP command channel
const CHANNEL_EXECUTABLE: u8 = 1; /// executable channel
const CHANNEL_HUB_CONTROL: u8 = 2; /// sensor hub control channel
const CHANNEL_SENSOR_REPORTS: u8 = 3; /// input sensor reports (non-wake, not gyroRV)
//const  CHANNEL_WAKE_REPORTS: usize = 4; /// wake input sensor reports (for sensors configured as wake up sensors)
//const  CHANNEL_GYRO_ROTATION: usize = 5; ///  gyro rotation vector (gyroRV)



/// Command Channel requests / responses
///
// Commands
//const CMD_GET_ADVERTISEMENT: u8 = 0;
//const CMD_SEND_ERROR_LIST: u8 = 1;

/// Responses
const CMD_RESP_ADVERTISEMENT: u8 = 0;
const CMD_RESP_ERROR_LIST: u8 = 1;

/// SHTP constants
const SENSORHUB_PROD_ID_REQ: u8 = 0xF9;
const SENSORHUB_PROD_ID_RESP: u8 =  0xF8;

const SHTP_REPORT_SET_FEATURE_COMMAND: u8 = 0xFD;

const SENSOR_REPORTID_ROTATION_VECTOR: u8 = 0x05;


/// requests
//const SENSORHUB_COMMAND_REQ:u8 =  0xF2;
const SENSORHUB_COMMAND_RESP:u8 = 0xF1;


/// executable/device channel responses
/// Figure 1-27: SHTP executable commands and response
// const EXECUTABLE_DEVICE_CMD_UNKNOWN: u8 =  0;
const EXECUTABLE_DEVICE_CMD_RESET: u8 =  1;
//const EXECUTABLE_DEVICE_CMD_ON: u8 =   2;
//const EXECUTABLE_DEVICE_CMD_SLEEP =  3;

/// Response to CMD_RESET
const EXECUTABLE_DEVICE_RESP_RESET_COMPLETE: u8 = 1;

/// Commands and subcommands
const SH2_INIT_UNSOLICITED: u8 = 0x80;
const SH2_CMD_INITIALIZE: u8 = 4;
const SH2_INIT_SYSTEM: u8 = 1;
const SH2_STARTUP_INIT_UNSOLICITED:u8 = SH2_CMD_INITIALIZE | SH2_INIT_UNSOLICITED;

#[cfg(test)]
mod tests {
    use crate::interface::mock_i2c_port::FakeI2cPort;
    use super::BNO080;
    //use super::*;

    use crate::interface::I2cInterface;
    use crate::interface::i2c::DEFAULT_ADDRESS;



//    #[test]
//    fn test_receive_unsized_under() {
//        let mut mock_i2c_port = FakeI2cPort::new();
//
//        let packet: [u8; 3] = [0; 3];
//        mock_i2c_port.add_available_packet( &packet);
//
//        let mut shub = BNO080::new(mock_i2c_port);
//        let rc = shub.read_unsized_packet();
//        assert!(rc.is_err());
//    }

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

    pub const MIDPACK: [u8; 52] = [
        0x34, 0x00, 0x02, 0x7B,
        0xF8, 0x00, 0x01, 0x02,
        0x96, 0xA4, 0x98, 0x00,
        0xE6, 0x00, 0x00, 0x00,
        0x04, 0x00, 0x00, 0x00,
        0xF8, 0x00, 0x04, 0x04,
        0x36, 0xA3, 0x98, 0x00,
        0x95, 0x01, 0x00, 0x00,
        0x02, 0x00, 0x00, 0x00,
        0xF8, 0x00, 0x04, 0x02,
        0xE3, 0xA2, 0x98, 0x00,
        0xD9, 0x01, 0x00, 0x00,
        0x07, 0x00, 0x00, 0x00,
    ];

    #[test]
    fn test_receive_midpack() {
        let mut mock_i2c_port = FakeI2cPort::new();

        let packet = MIDPACK;
        mock_i2c_port.add_available_packet( &packet);

        let mut shub = BNO080::new_with_interface(
            I2cInterface::new(mock_i2c_port, DEFAULT_ADDRESS));
        let rc = shub.receive_packet();
        assert!(rc.is_ok());
    }


    #[test]
    fn test_handle_adv_message() {
        let mut mock_i2c_port = FakeI2cPort::new();

        //actual startup response packet
        let raw_packet = ADVERTISING_PACKET_FULL;
        mock_i2c_port.add_available_packet( &raw_packet);

        let mut shub = BNO080::new_with_interface(
            I2cInterface::new(mock_i2c_port, DEFAULT_ADDRESS));

        let msg_count = shub.handle_one_message();
        assert_eq!(msg_count, 1, "wrong msg_count");

    }

    // Actual advertising packet received from sensor:
    pub const ADVERTISING_PACKET_FULL: [u8; 276] = [
        0x14, 0x81, 0x00, 0x01,
        0x00, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x80, 0x06, 0x31, 0x2e, 0x30, 0x2e, 0x30, 0x00, 0x02, 0x02, 0x00, 0x01, 0x03, 0x02, 0xff, 0x7f, 0x04, 0x02, 0x00, 0x01, 0x05,
        0x02, 0xff, 0x7f, 0x08, 0x05, 0x53, 0x48, 0x54, 0x50, 0x00, 0x06, 0x01, 0x00, 0x09, 0x08, 0x63, 0x6f, 0x6e, 0x74, 0x72, 0x6f, 0x6c, 0x00, 0x01, 0x04, 0x01, 0x00, 0x00,
        0x00, 0x08, 0x0b, 0x65, 0x78, 0x65, 0x63, 0x75, 0x74, 0x61, 0x62, 0x6c, 0x65, 0x00, 0x06, 0x01, 0x01, 0x09, 0x07, 0x64, 0x65, 0x76, 0x69, 0x63, 0x65, 0x00, 0x01, 0x04,
        0x02, 0x00, 0x00, 0x00, 0x08, 0x0a, 0x73, 0x65, 0x6e, 0x73, 0x6f, 0x72, 0x68, 0x75, 0x62, 0x00, 0x06, 0x01, 0x02, 0x09, 0x08, 0x63, 0x6f, 0x6e, 0x74, 0x72, 0x6f, 0x6c,
        0x00, 0x06, 0x01, 0x03, 0x09, 0x0c, 0x69, 0x6e, 0x70, 0x75, 0x74, 0x4e, 0x6f, 0x72, 0x6d, 0x61, 0x6c, 0x00, 0x07, 0x01, 0x04, 0x09, 0x0a, 0x69, 0x6e, 0x70, 0x75, 0x74,
        0x57, 0x61, 0x6b, 0x65, 0x00, 0x06, 0x01, 0x05, 0x09, 0x0c, 0x69, 0x6e, 0x70, 0x75, 0x74, 0x47, 0x79, 0x72, 0x6f, 0x52, 0x76, 0x00, 0x80, 0x06, 0x31, 0x2e, 0x31, 0x2e,
        0x30, 0x00, 0x81, 0x64, 0xf8, 0x10, 0xf5, 0x04, 0xf3, 0x10, 0xf1, 0x10, 0xfb, 0x05, 0xfa, 0x05, 0xfc, 0x11, 0xef, 0x02, 0x01, 0x0a, 0x02, 0x0a, 0x03, 0x0a, 0x04, 0x0a,
        0x05, 0x0e, 0x06, 0x0a, 0x07, 0x10, 0x08, 0x0c, 0x09, 0x0e, 0x0a, 0x08, 0x0b, 0x08, 0x0c, 0x06, 0x0d, 0x06, 0x0e, 0x06, 0x0f, 0x10, 0x10, 0x05, 0x11, 0x0c, 0x12, 0x06,
        0x13, 0x06, 0x14, 0x10, 0x15, 0x10, 0x16, 0x10, 0x17, 0x00, 0x18, 0x08, 0x19, 0x06, 0x1a, 0x00, 0x1b, 0x00, 0x1c, 0x06, 0x1d, 0x00, 0x1e, 0x10, 0x1f, 0x00, 0x20, 0x00,
        0x21, 0x00, 0x22, 0x00, 0x23, 0x00, 0x24, 0x00, 0x25, 0x00, 0x26, 0x00, 0x27, 0x00, 0x28, 0x0e, 0x29, 0x0c, 0x2a, 0x0e
    ];


}
