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
    max_speed_hz: u32,
    slave_count: u16,
    /// Bytes that can be read in by the SPI before USB IN transfers start
    in_buf_size: u16,
}

impl MasterInfo {
    pub fn new(max_speed_hz: u32, slave_count: u16, in_buf_size: u16) -> Self {
        Self {
            hardware: HardwareType::SpiMaster,
            max_speed_hz,
            slave_count,
            in_buf_size,
        }
    }

    // TODO should just be able to memcpy all this, after checking that we're little-endian...
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.hardware as u8;
        // 3 pad bytes
        buf[4..8].copy_from_slice(&self.max_speed_hz.to_le_bytes());
        buf[8..10].copy_from_slice(&self.slave_count.to_le_bytes());
        buf[10..12].copy_from_slice(&self.in_buf_size.to_le_bytes());
        12 // Length
    }
}

// The doubling up is just because cbindgen makes a mess out of bitflags
pub mod capabilities {
    pub const MODE_0: u8 = 0x01;
    pub const MODE_1: u8 = 0x02;
    pub const MODE_2: u8 = 0x04;
    pub const MODE_3: u8 = 0x08;
    pub const ANY_MODE: u8 = 0x0F;
    pub const MSB_FIRST: u8 = 0x10;
    pub const LSB_FIRST: u8 = 0x20;
    pub const INTERRUPT: u8 = 0x40;
    pub const RESET: u8 = 0x80;
}

bitflags! {
    #[repr(C)]
    pub struct SpiDeviceCapabilities: u8 {
        const MODE_0 = capabilities::MODE_0;
        const MODE_1 = capabilities::MODE_1;
        const MODE_2 = capabilities::MODE_2;
        const MODE_3 = capabilities::MODE_3;
        const ANY_MODE = capabilities::ANY_MODE;
        const MSB_FIRST = capabilities::MSB_FIRST;
        const LSB_FIRST = capabilities::LSB_FIRST;
        const INTERRUPT = capabilities::INTERRUPT;
        const RESET = capabilities::RESET;
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

    pub max_speed_hz: u32,

    // This is just a bit cleaner to deal with in C
    pub capabilities: u8,
}

impl ConnectedSlaveInfoLinux {
    pub fn new(modalias: &'static str, max_speed_hz: u32, capabilities: SpiDeviceCapabilities) -> Self {
        let mut modalias_arr = [0; 32];
        if modalias.len() < 32 {
            modalias_arr[0..modalias.len()].copy_from_slice(modalias.as_bytes());
        } else {
            // defmt::warn!("Truncating {:?}", modalias);
            modalias_arr.copy_from_slice(&modalias.as_bytes()[..32]);
        }

        Self {
            modalias: modalias_arr,
            max_speed_hz,
            capabilities: capabilities.bits,
        }
    }

    pub fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0..32].copy_from_slice(&self.modalias);
        buf[32..36].copy_from_slice(&self.max_speed_hz.to_le_bytes());
        buf[36] = self.capabilities;

        buf[37..40].fill(0); // Needs to round up to a u32 boundary in C-land
        40
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

// TODO this could really be a data field in Direction
bitflags! {
    #[repr(C)]
    pub struct TransferMode: u8 {
        const CPHA = 0x01;
        const CPOL = 0x02;
        const MODE_MASK = 0x03;
        const MODE_0 = 0x00;
        const MODE_1 = 0x01;
        const MODE_2 = 0x02;
        const MODE_3 = 0x03;

        const LSB_FIRST = 0x08; // Just to line up with Linux
    }
}

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
    pub speed_hz: u32,
    pub direction: Direction,
    pub mode: u8, // Wants to be a TransferMode, but cbindgen, insomina...
}

impl TransferHeader {
    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < 10 {
            // Not !=, because data follows
            return None;
        }
        match Direction::n(buf[8]) {
            Some(Direction::None) | None => None,
            Some(direction) => Some(Self {
                bytes: LittleEndian::read_u16(&buf[0..2]),
                index: LittleEndian::read_u16(&buf[2..4]),
                speed_hz: LittleEndian::read_u32(&buf[4..8]),
                direction,
                mode: buf[9],
            }),
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
