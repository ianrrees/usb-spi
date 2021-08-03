#![no_std]

/// Protocol used between the USB host and device, for USB-SPI
use bitflags::bitflags;
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

bitflags! {
    pub struct SpiDeviceCapabilities: u8 {
        const MODE_0 = 0x01;
        const MODE_1 = 0x02;
        const MODE_2 = 0x04;
        const MODE_3 = 0x08;
        const ANY_MODE = Self::MODE_0.bits | Self::MODE_1.bits | Self::MODE_2.bits | Self::MODE_3.bits;
        const MSB_FIRST = 0x10;
        const LSB_FIRST = 0x20;
        const INTERRUPT = 0x40;
        const RESET = 0x80;
    }
}

/// Returned by SPI masters for REQUEST_IN_LINUX_SLAVE_INFO
#[repr(C)]
pub struct ConnectedSlaveInfoLinux {
    // TODO
    // pub platform_data_len: u16,
    /// NULL-terminated, 32 comes from Linux's SPI_NAME_SIZE TODO use const
    pub modalias: [u8; 32],
    // TODO how to encode this in Rust?
    // char platform_data[0],

    // pub max_clock_speed_hz: u32,

    // pub capabilities: SpiDeviceCapabilities,
}

impl ConnectedSlaveInfoLinux {
    pub fn new(modalias: &'static str) -> Self {
        let mut modalias_arr = [0; 32];
        if modalias.len() < 32 {
            modalias_arr[0..modalias.len()].copy_from_slice(modalias.as_bytes());
        } else {
            // defmt::warn!("Truncating {:?}", modalias);
            modalias_arr.copy_from_slice(&modalias.as_bytes()[..32]);
        }

        Self {
            modalias: modalias_arr,
        }
    }

    pub fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0..32].copy_from_slice(&self.modalias);
        32
    }
}

#[repr(C)]
pub enum Error {
    IndexOutOfRange,
    UnsupportedByDevice,
}

#[derive(Copy, Clone, Debug, N)]
#[repr(u8)]
pub enum EventType {
    NoEvent,
    Interrupt,
    Error,
}

#[repr(C)]
pub struct Event {
    /// Some events contain more information, it goes here
    ///
    /// Interrupt: which device index is interrupting
    /// Error: enum value of the specific error
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
pub enum Direction {
    None,
    OutOnly,
    InOnly,
    Both,
}

/// Sent through the bulk OUT endpoint, possibly before any OUT data
#[repr(C)]
pub struct TransferHeader {
    pub bytes: u16,
    /// Index of the slave device the transfer is intended for
    pub index: u16,
    pub direction: Direction,
}

impl TransferHeader {
    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < 5 {
            // Not !=, because data often follows
            None
        } else {
            match Direction::n(buf[4]) {
                Some(direction) => Some(Self {
                    bytes: LittleEndian::read_u16(&buf[0..2]),
                    index: LittleEndian::read_u16(&buf[2..4]),
                    direction,
                }),
                None => None,
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
