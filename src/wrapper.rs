use crate::interface::SensorInterface;
use embedded_hal::{
    blocking::delay::{DelayUs, DelayMs},
};
use crate::{PACKET_HEADER_LENGTH, Error};
use core::ops::{Shl, Shr};

pub struct Wrapper<SI> {

    sensor_interface: SI,

}

impl<SI> Wrapper<SI>
    where
        SI: SensorInterface
{
    /// Receive and ignore one message
    pub fn eat_one_message(&mut self) -> usize {
        let res = self.sensor_interface.receive_packet();
        res.unwrap_or(0)
    }

    /// Consume all available messages on the port without processing them
    pub fn eat_all_messages(&mut self, delay: &mut dyn DelayMs<u8>) {
        loop {
            let received_len = self.eat_one_message();
            if received_len == 0 {
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

        let res = self.sensor_interface.receive_packet();
        if res.is_ok() {
            let received_len = res.unwrap_or(0);
            if received_len > 0 {
                msg_count += 1;
                self.handle_received_packet(received_len);
            }
        }

        msg_count
    }

    fn parse_packet_header(packet: &[u8]) -> usize {
        const CONTINUATION_FLAG_CLEAR: u16 = !(0x80);
        if packet.len() < PACKET_HEADER_LENGTH {
            return 0;
        }
        //Bits 14:0 are used to indicate the total number of bytes in the body plus header
        //maximum packet length is ... 32767?
        let raw_pack_len: u16 =
            (packet[0] as u16) + ((packet[1] as u16)
                & CONTINUATION_FLAG_CLEAR).shl(8);
        let packet_len: usize = raw_pack_len as usize;

        //let is_continuation:bool = (packet[1] & 0x80) != 0;
        //let chan_num =  packet[2];
        //let seq_num =  packet[3];

        packet_len
    }


    pub fn handle_received_packet(&mut self, received_len: usize) {
        let msg = &self.packet_recv_buf[..received_len];
        let chan_num = msg[2];
        //let _seq_num =  msg[3];
        let report_id: u8 = msg[4];
    }

    /// The BNO080 starts up with all sensors disabled,
    /// waiting for the application to configure it.
    pub fn init(&mut self, delay: &mut dyn DelayMs<u8>) -> Result<(), Error<E>> {
        //Section 5.1.1.1 : On system startup, the SHTP control application will send
        // its full advertisement response, unsolicited, to the host.

        self.soft_reset()?;
        delay.delay_ms(50);
        self.eat_one_message();
        delay.delay_ms(50);
        self.eat_all_messages(delay);
        // delay.delay_ms(50);
        // self.eat_all_messages(delay);

        self.verify_product_id()?;
    }

    /// Tell the sensor to start reporting the fused rotation vector
/// on a regular cadence. Note that the maximum valid update rate
/// is 1 kHz, based on the max update rate of the sensor's gyros.
    pub fn enable_rotation_vector(&mut self, millis_between_reports: u16)  -> Result<(), Error<E>> {
        self.enable_report(SENSOR_REPORTID_ROTATION_VECTOR, millis_between_reports)
    }

    /// Enable a particular report
    fn enable_report(&mut self, report_id: u8, millis_between_reports: u16)  -> Result<(), Error<E>> {
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

        self.send_packet(CHANNEL_HUB_CONTROL, &cmd_body)?;
        Ok(())
    }


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
    //const SH2_INIT_SYSTEM: u8 = 1;
    const SH2_STARTUP_INIT_UNSOLICITED:u8 = SH2_CMD_INITIALIZE | SH2_INIT_UNSOLICITED;

}