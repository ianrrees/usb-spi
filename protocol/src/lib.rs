#![no_std]
#![allow(non_camel_case_types, dead_code)]


/// Protocol used between the USB host and device, for USB-SPI

// Thinking: Use control requests to pick the slave device (inc CS, bus clock, etc) and set whether
// the bulk endpoints are used for reading/writing/both.  Reject requests when there's data waiting.
//
// Unsure if current Rust usb-device supports control requests with more data than the control EP
// can handle in a single transaction?

// TODO SPI mode

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
}

impl MasterInfo {
    pub fn new(slave_count: u16) -> Self {
        Self {
            hardware: HardwareType::SpiMaster,
            slave_count,
        }
    }
    
    // TODO should just be able to memcpy all this, after checking that we're little-endian...
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.hardware as u8;
        // pad byte
        buf[2..4].copy_from_slice(&self.slave_count.to_le_bytes());
        4 // Length
    }
}

/// Returned by SPI masters for REQUEST_IN_LINUX_SLAVE_INFO
#[repr(C)]
pub struct ConnectedSlaveInfoLinux {
    // TODO
    // pub platform_data_len: u16,
    pub has_interrupt: u8,
    /// NULL-terminated
    pub modalias: [u8; 32],
    // TODO how to encode this in Rust?
    // char platform_data[0],

    // Is there any point in passing the mode or speed?
}

#[repr(u8)]
pub enum EventType {
    NONE,
}

#[repr(C)]
pub struct Event {
    pub event: EventType,
    // TODO more fields, FIFO like Miss Thripp?
}

/// Used as the request field of IN control transfers
#[derive(Copy, Clone, Debug, N)]
#[repr(u8)]
pub enum ControlIn {
    REQUEST_IN_HW_INFO,
    REQUEST_IN_GET_EVENT,
    /// Slave ID is sent in the value field
    REQUEST_IN_LINUX_SLAVE_INFO,
}

/// Used as the request field of OUT control transfers
#[derive(Copy, Clone, Debug, N)]
#[repr(u8)]
pub enum ControlOut {
    /// Slave ID is sent in the value field
    REQUEST_OUT_SET_SLAVE,
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
