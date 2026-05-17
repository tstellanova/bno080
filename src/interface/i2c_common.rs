use super::{SensorCommon, PACKET_HEADER_LENGTH};

/// the i2c address normally used by BNO080
pub const DEFAULT_ADDRESS: u8 = 0x4A;
/// alternate i2c address for BNO080
pub const ALTERNATE_ADDRESS: u8 = 0x4B;

/// Length of our receive buffer:
/// Note that this likely needs to be < 256 to accommodate underlying HAL
pub(crate) const SEG_RECV_BUF_LEN: usize = 240;
pub(crate) const MAX_SEGMENT_READ: usize = SEG_RECV_BUF_LEN;

pub(crate) struct I2cCommon {
    /// address for i2c communications with the sensor hub
    address: u8,
    /// buffer for receiving segments of packets from the sensor hub
    seg_recv_buf: [u8; SEG_RECV_BUF_LEN],
    /// number of packets received
    received_packet_count: usize,
}

impl I2cCommon {
    pub(crate) fn new(address: u8) -> Self {
        Self {
            address,
            seg_recv_buf: [0; SEG_RECV_BUF_LEN],
            received_packet_count: 0,
        }
    }

    pub(crate) fn address(&self) -> u8 {
        self.address
    }

    pub(crate) fn seg_recv_buf(&self) -> &[u8; SEG_RECV_BUF_LEN] {
        &self.seg_recv_buf
    }

    pub(crate) fn seg_recv_buf_mut(&mut self) -> &mut [u8; SEG_RECV_BUF_LEN] {
        &mut self.seg_recv_buf
    }

    pub(crate) fn zero_recv_packet_header(&mut self) {
        Self::zero_buffer(&mut self.seg_recv_buf[..PACKET_HEADER_LENGTH]);
    }

    pub(crate) fn zero_buffer(buf: &mut [u8]) {
        for byte in buf {
            *byte = 0;
        }
    }

    pub(crate) fn packet_len_from_header(&self) -> usize {
        SensorCommon::parse_packet_header(
            &self.seg_recv_buf[..PACKET_HEADER_LENGTH],
        )
    }

    pub(crate) fn record_received_packet(&mut self, packet_len: usize) {
        if packet_len > 0 {
            self.received_packet_count += 1;
        }
    }

    pub(crate) fn sized_read(
        total_packet_len: usize,
        packet_recv_buf: &mut [u8],
    ) -> SizedRead {
        SizedRead::new(total_packet_len, packet_recv_buf)
    }
}

pub(crate) struct SizedRead {
    remaining_body_len: usize,
    already_read_len: usize,
    total_packet_len: usize,
}

impl SizedRead {
    fn new(total_packet_len: usize, packet_recv_buf: &mut [u8]) -> Self {
        let remaining_body_len = total_packet_len - PACKET_HEADER_LENGTH;

        Self::zero_packet_header(packet_recv_buf);

        Self {
            remaining_body_len,
            already_read_len: 0,
            total_packet_len,
        }
    }

    pub(crate) fn direct_read_len(&self) -> Option<usize> {
        if self.total_packet_len < MAX_SEGMENT_READ && self.total_packet_len > 0
        {
            Some(self.total_packet_len)
        } else {
            None
        }
    }

    pub(crate) fn finish_direct_read(
        &mut self,
        already_read_len: usize,
    ) -> usize {
        self.already_read_len = already_read_len;
        self.already_read_len
    }

    pub(crate) fn has_remaining_segments(&self) -> bool {
        self.remaining_body_len > 0
    }

    pub(crate) fn next_segment_read_len(&self) -> usize {
        let whole_segment_length =
            self.remaining_body_len + PACKET_HEADER_LENGTH;
        if whole_segment_length > MAX_SEGMENT_READ {
            MAX_SEGMENT_READ
        } else {
            whole_segment_length
        }
    }

    pub(crate) fn transcribe_segment(
        &mut self,
        segment_read_len: usize,
        seg_recv_buf: &[u8],
        packet_recv_buf: &mut [u8],
    ) {
        let transcribe_start_idx = if self.already_read_len > 0 {
            PACKET_HEADER_LENGTH
        } else {
            0
        };
        let transcribe_len = if self.already_read_len > 0 {
            segment_read_len - PACKET_HEADER_LENGTH
        } else {
            segment_read_len
        };

        packet_recv_buf
            [self.already_read_len..self.already_read_len + transcribe_len]
            .copy_from_slice(
                &seg_recv_buf[transcribe_start_idx
                    ..transcribe_start_idx + transcribe_len],
            );
        self.already_read_len += transcribe_len;

        let body_read_len = segment_read_len - PACKET_HEADER_LENGTH;
        self.remaining_body_len -= body_read_len;
    }

    pub(crate) fn already_read_len(&self) -> usize {
        self.already_read_len
    }

    fn zero_packet_header(packet_recv_buf: &mut [u8]) {
        for byte in &mut packet_recv_buf[..PACKET_HEADER_LENGTH] {
            *byte = 0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_recv_packet_header_only_clears_header() {
        let mut common = I2cCommon::new(DEFAULT_ADDRESS);
        common.seg_recv_buf[..8].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);

        common.zero_recv_packet_header();

        assert_eq!(&common.seg_recv_buf[..8], &[0, 0, 0, 0, 5, 6, 7, 8]);
    }

    #[test]
    fn segmented_read_transcribes_first_header_then_body_only() {
        let total_packet_len = 276;
        let mut recv_buf = [0xAA; 276];
        let mut read = I2cCommon::sized_read(total_packet_len, &mut recv_buf);

        let first_segment_len = read.next_segment_read_len();
        assert_eq!(first_segment_len, MAX_SEGMENT_READ);

        let mut first_segment = [0; SEG_RECV_BUF_LEN];
        for (idx, byte) in
            first_segment[..first_segment_len].iter_mut().enumerate()
        {
            *byte = idx as u8;
        }
        read.transcribe_segment(
            first_segment_len,
            &first_segment,
            &mut recv_buf,
        );
        assert_eq!(read.already_read_len(), MAX_SEGMENT_READ);
        assert_eq!(&recv_buf[..6], &[0, 1, 2, 3, 4, 5]);

        let second_segment_len = read.next_segment_read_len();
        assert_eq!(second_segment_len, 40);

        let mut second_segment = [0; SEG_RECV_BUF_LEN];
        second_segment[..second_segment_len].fill(0xEE);
        second_segment[PACKET_HEADER_LENGTH..second_segment_len].fill(0xBB);
        read.transcribe_segment(
            second_segment_len,
            &second_segment,
            &mut recv_buf,
        );

        assert_eq!(read.already_read_len(), total_packet_len);
        assert_eq!(&recv_buf[240..276], &[0xBB; 36]);
        assert!(!read.has_remaining_segments());
    }

    #[test]
    fn record_received_packet_ignores_zero_length_packets() {
        let mut common = I2cCommon::new(DEFAULT_ADDRESS);

        common.record_received_packet(0);
        common.record_received_packet(4);
        common.record_received_packet(0);
        common.record_received_packet(12);

        assert_eq!(common.received_packet_count, 2);
    }
}
