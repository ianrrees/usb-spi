#![no_std]

/// Protocol used between the USB host and device, for USB-SPI

// Thinking: Use control requests to pick the slave device (inc CS, bus clock, etc) and set whether
// the bulk endpoints are used for reading/writing/both.  Reject requests when there's data waiting.
//
// Unsure if current Rust usb-device supports control requests with more data than the control EP
// can handle in a single transaction?

// TODO SPI mode

use byteorder::{ByteOrder, LittleEndian};
use enumn::N;

/// Functions as something like a communications protocol version
#[derive(Copy, Clone, Debug, N)]
#[repr(u8)]
pub enum HardwareType {
    SpiMaster,
}

#[repr(C)]
pub struct MasterInfo {
    /// Encode as SpiMaster
    hardware: HardwareType,
    slave_count: u16,
    /// Bytes that can be read in by the SPI before USB IN transfers start
    in_buf_size: u16,
}

impl MasterInfo {
    pub fn new(slave_count: u16, in_buf_size: u16) -> Self {
        Self {
            hardware: HardwareType::SpiMaster,
            slave_count,
            in_buf_size,
        }
    }
    
    // TODO should just be able to memcpy all this, after checking that we're little-endian...
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.hardware as u8;
        // pad byte
        buf[2..4].copy_from_slice(&self.slave_count.to_le_bytes());
        buf[4..6].copy_from_slice(&self.in_buf_size.to_le_bytes());
        6 // Length
    }
}

/// Returned by SPI masters for REQUEST_IN_LINUX_SLAVE_INFO
#[repr(C)]
pub struct ConnectedSlaveInfoLinux {
    // TODO
    // pub platform_data_len: u16,
    pub has_interrupt: u8, // TODO bitfield
    /// NULL-terminated, 32 comes from Linux's SPI_NAME_SIZE TODO use const
    pub modalias: [u8; 32],
    // TODO how to encode this in Rust?
    // char platform_data[0],

    // Is there any point in passing the mode or speed?
}

impl ConnectedSlaveInfoLinux {
    pub fn new(has_interrupt: bool, modalias: &'static str) -> Self {
        let mut modalias_arr = [0; 32];
        if modalias.len() < 32 {
            modalias_arr[0..modalias.len()].copy_from_slice(modalias.as_bytes());
        } else {
            // defmt::warn!("Truncating {:?}", modalias);
            modalias_arr.copy_from_slice(&modalias.as_bytes()[..32]);
        }

        Self {
            has_interrupt: has_interrupt as u8,
            modalias: modalias_arr,
        }
    }

    pub fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.has_interrupt;
        buf[1..33].copy_from_slice(&self.modalias);
        33
    }
}

#[derive(Copy, Clone, Debug, N)]
#[repr(u8)]
pub enum EventType {
    NoEvent,
    Interrupt,
}

#[repr(C)]
pub struct Event {
    /// For event types like Interrupt, this conveys which device is interrupting
    pub data: u16,
    pub event_type: EventType,
}

impl Event {
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0..2].copy_from_slice(&self.data.to_le_bytes());
        buf[2] = self.event_type as u8;
        4 // Needs to be == sizeof(struct usb_spi_Event) in C
    }
}

/// Used as the request field of IN control transfers
#[derive(Copy, Clone, Debug, N)]
#[repr(u8)]
pub enum ControlIn {
    // TODO rename these to fit Rust conventions
    HwInfo,
    GetEvent,
    /// Slave ID is sent in the value field
    LinuxSlaveInfo,
}

// /// Used as the request field of OUT control transfers
// #[derive(Copy, Clone, Debug, N)]
// #[repr(u8)]
// pub enum ControlOut {
// }

/// USB convention of the directions used in this transfer
///
/// USB is used because the protocol could be used for attaching SPI controllers
/// or peripherals, but the logic around headers going in OUT transfer wouldn't
/// change based on the SPI direction.
#[derive(Copy, Clone, Debug, Eq, PartialEq, N)]
#[repr(u8)]
pub enum Direction { // TODO rename
    None,
    OutOnly,
    InOnly,
    Both,
    /// bytes field is chip select index
    CsAssert,
    /// bytes field is ignored
    CsDeassert,
}

/// Sent through the bulk OUT endpoint, possibly before any OUT data
#[repr(C)]
pub struct TransferHeader {
    pub bytes: u16, // TODO rename 'data'
    pub direction: Direction,
}

impl TransferHeader {
    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() <3 { // Not !=, because data often follows
            None
        } else {
            match Direction::n(buf[2]) {
                Some(direction) => {
                    Some(Self{
                        bytes: LittleEndian::read_u16(&buf[0..2]),
                        direction,
                    })
                }
                None => {
                    None
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
