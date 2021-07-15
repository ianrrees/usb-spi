#![no_std]

extern crate atsamd_hal;
extern crate defmt_rtt;

use atsamd_hal::{
    gpio::v2::*,
    hal::digital::v2::OutputPin, // embedded_hal traits
    prelude::*,
    sercom::v2::spi::{
        Config, EightBit, Error as SpiError, Flags, Master, Rx, Spi, Tx, ValidPads,
    },
    // time::Hertz,
    typelevel::NoneT,
};

// TODO use const generics, so our customers don't need to see this
pub use bbqueue::consts::*;
use typenum::marker_traits::Unsigned; // Also get rid of this dependency

// See note in Cargo.toml as to why this not heapless
use bbqueue::{BBBuffer, Consumer, Error as BBError, Producer};
use usb_device::{class_prelude::*, Result};
use usb_spi_protocol as protocol;
use protocol::{
    Direction,
    TransferHeader,
};

const USB_CLASS_VENDOR_SPECIFIC: u8 = 0xFF; // TODO push up to a protocol crate
const USB_SUBCLASS_VENDOR_SPECIFIC_USB_SPI: u8 = 0x02; // TODO push up to a protocol crate
const USB_SPI_PROTOCOL_0: u8 = 0x00; // TODO push up to a protocol crate

/// SPI reads require writes, use this when there isn't other data to write.
const USB_SPI_TX_FILL: u8 = 0x00;

/// Describes an SPI-connected peripheral device
///
/// This is a trait because the chip select handling might be more complicated
/// than toggling a GPIO pin, and because we might support passing a device-
/// specific struct up to the device's driver on the USB host.
pub trait SpiDevice {
    /// Assert the device's chip select
    fn select(&mut self);

    /// Deassert the device's chip select
    fn deselect(&mut self);

    /// modalias is how Linux SPI drivers associate devices with controllers
    fn modalias(&self) -> &'static str;
}

// TODO add interrupt
pub struct BasicSpiDevice<P: AnyPin<Mode = PushPullOutput>> {
    pub modalias: Option<&'static str>,

    pub cs_pin: P,
}

impl <P: AnyPin<Mode = PushPullOutput> + OutputPin> SpiDevice for BasicSpiDevice<P> {
    fn select(&mut self) {
        defmt::info!("In BasicSpiDevice::select() for {:?}", self.modalias); // TODO
        self.cs_pin.set_low().ok().unwrap();
    }

    fn deselect(&mut self) {
        defmt::info!("In BasicSpiDevice::deselect() for {:?}", self.modalias); // TODO
        self.cs_pin.set_high().ok().unwrap();
    }

    fn modalias(&self) -> &'static str {
        self.modalias.unwrap_or("")
    }
}

pub type SpiDeviceList<'a> = [&'a mut dyn SpiDevice];

type BufferSize = U256;

/// TX and RX buffers used by the UsbSpi
///
/// Due to the BBQueue API, combined with Rust's rules about structs that
/// contain references to other members in the struct, we need a separate struct
/// to contain the BBQueue storage.  This structure should never be moved in
/// memory once it's in use, because any outstanding grant from before the move
/// would point to memory which is no longer inside the buffer.
pub struct UsbSpiStorage {
    /// For data from the SPI to the USB; from the DCE to DTE, device to host
    rx_buffer: BBBuffer<BufferSize>,
    /// Other direction from `rx_buffer`
    tx_buffer: BBBuffer<BufferSize>,
}

impl UsbSpiStorage {
    pub fn new() -> Self {
        Self {
            rx_buffer: BBBuffer::new(),
            tx_buffer: BBBuffer::new(),
        }
    }
}

/// If this many full size packets have been sent in a row, a short packet will
/// be sent so that the host sees the data in a timely manner.
const SHORT_PACKET_INTERVAL: usize = 10;

// TODO rename here and in atsamd-usbd-uart
/// Keeps track of the type of the last written packet.
enum WriteState {
    /// The last packet written wasn't a full packet
    NotFull,

    /// Full packet current in-flight. A full packet must be followed by a short
    /// packet for the host OS to see the transaction. The data is the number of
    /// subsequent full packets sent so far. A short packet is forced every
    /// SHORT_PACKET_INTERVAL packets so that the OS sees data in a timely
    /// manner.
    Full(usize),
}

/// Tracks progress through USB and SPI transactions
///
/// This is necessary because SPI transactions on full duplex hardware always
/// involve both reading and writing data, but the host might only want to read
/// or write independently.  There would be no point in buffering read in data
/// if the host isn't interested in it, nor buffering data to write which would
/// (hopefully!) be ignored by the peripheral.  We also might make longer SPI
/// transactions than will fit in a USB transaction.
struct TransactionState {
    direction: Direction,
    expected: usize,
    received: usize,
    sent: usize,
}

/// TODO could turn this in to one of those fancy typestatemachines...
impl TransactionState {
    fn new() -> Self {
        Self {
            direction: Direction::None,
            expected: 0,
            received: 0,
            sent: 0,
        }
    }
}

/// A USB CDC to SPI
pub struct UsbSpi<'a, B, P, const DEVICE_COUNT: usize, const ENDPOINT_SIZE: usize>
where
    B: UsbBus,
    P: ValidPads<SS = NoneT> + Tx + Rx,
{
    usb_interface: InterfaceNumber,
    read_endpoint: EndpointOut<'a, B>,
    write_endpoint: EndpointIn<'a, B>,

    /// SPI end of the SPI->USB buffer
    spi_to_usb_producer: Producer<'a, BufferSize>,

    /// USB end of the SPI->USB buffer
    spi_to_usb_consumer: Consumer<'a, BufferSize>,

    /// USB end of the USB->SPI buffer
    usb_to_spi_producer: Producer<'a, BufferSize>,

    /// SPI end of the USB->SPI buffer
    usb_to_spi_consumer: Consumer<'a, BufferSize>,

    write_state: WriteState,

    /// The enabled SPI hardware
    spi: Spi<Config<P, Master, EightBit>>,

    /// Only modified in the USB ISR, except transition out of OutOnly in SPI ISR
    usb_state: TransactionState,

    /// Only modified in the SPI ISR, except transition out of Idle in the USB ISR
    spi_state: TransactionState,

    devices: [&'a mut dyn SpiDevice; DEVICE_COUNT],

    // Index in to Self.devices, of the currently-selected device
    selected_device: Option<usize>,
}

impl<'a, B, P, const DEVICE_COUNT: usize, const ENDPOINT_SIZE: usize> UsbSpi<'a, B, P, DEVICE_COUNT, ENDPOINT_SIZE>
where
    B: UsbBus,
    P: ValidPads<SS = NoneT> + Tx + Rx,
{
    pub fn new(
        alloc: &'a UsbBusAllocator<B>,
        storage: &'a UsbSpiStorage,
        spi_hardware: Config<P, Master, EightBit>,
        devices: [&'a mut dyn SpiDevice; DEVICE_COUNT],
    ) -> Self {
        let (spi_to_usb_producer, spi_to_usb_consumer) = storage.rx_buffer.try_split().unwrap();
        let (usb_to_spi_producer, usb_to_spi_consumer) = storage.tx_buffer.try_split().unwrap();

        let mut spi = spi_hardware.enable();
        spi.enable_interrupts(Flags::RXC);

        Self {
            usb_interface: alloc.interface(),
            read_endpoint: alloc.bulk(ENDPOINT_SIZE as u16),
            write_endpoint: alloc.bulk(ENDPOINT_SIZE as u16),

            spi_to_usb_producer,
            spi_to_usb_consumer,
            usb_to_spi_producer,
            usb_to_spi_consumer,

            spi_state: TransactionState::new(),
            usb_state: TransactionState::new(),

            write_state: WriteState::NotFull,

            spi,
            devices,
            selected_device: None,
        }
    }

    // TODO perhaps a better name
    pub fn flush_usb(&mut self) {
        // TODO Use usb_state
        match self.spi_to_usb_consumer.read() {
            // There is more data to write
            Ok(grant) => {
                // Deciding what to write is a bit more complicated,
                // due to the way USB bulk transfers work...
                let full_packet_count = match self.write_state {
                    WriteState::Full(c) => c,
                    WriteState::NotFull => 0,
                };

                let max_write_size = if full_packet_count >= SHORT_PACKET_INTERVAL {
                    ENDPOINT_SIZE - 1
                } else {
                    ENDPOINT_SIZE
                };

                let write_slice = if grant.buf().len() > max_write_size {
                    grant.buf().split_at(max_write_size).0
                } else {
                    grant.buf()
                };

                match self.write_endpoint.write(write_slice) {
                    Ok(count) => {
                        // TODO it would be nice to release only after we get
                        // the endpoint_in_complete() callback, so we're sure
                        // the data was read by the host.
                        grant.release(count);

                        self.write_state = if count >= ENDPOINT_SIZE {
                            WriteState::Full(full_packet_count + 1)
                        } else {
                            WriteState::NotFull
                        };
                        // Ok(())
                    }

                    Err(UsbError::WouldBlock) => {}

                    Err(_) => {
                        defmt::error!("Error writing packet")
                        // Err(e)
                    }
                }
            }

            // No more data to write
            Err(BBError::InsufficientSize) => {
                if let WriteState::Full(_) = self.write_state {
                    // Need to send a Zero Length Packet to
                    // signal the end of a transaction
                    match self.write_endpoint.write(&[]) {
                        Ok(_) => {
                            self.write_state = WriteState::NotFull;
                        }
                        Err(UsbError::WouldBlock) => {}
                        Err(_) => {
                            defmt::error!("Error writing ZLP")
                        }
                    }
                } else {
                    self.write_state = WriteState::NotFull;
                }

                // Ok(())
            }

            Err(_) => {
                // TODO handle this better
                defmt::error!("Couldn't get spi_to_usb_consumer grant");
                // Ok(())
            }
        }
    }

    // TODO perhaps a better name
    /// Attempts to write a out over SPI
    fn flush_spi(&mut self) {
        match self.spi_state.direction {
            Direction::OutOnly | Direction::Both => {
                // Try to send the next available byte
                match self.usb_to_spi_consumer.read() {
                    Ok(grant) => {
                        // The buffer returned is guaranteed to have at least one byte
                        match self.spi.send(grant.buf()[0]) {
                            Ok(()) => {
                                self.spi_state.sent += 1;
                                defmt::info!("Sent {:X}", grant.buf()[0]); // TODO

                                // The only way to clear DRE is to write data, so only
                                // enable it when we'll have more data to write.
                                if grant.buf().len() > 1 {
                                    self.spi.enable_interrupts(Flags::DRE);
                                } else {
                                    self.spi.disable_interrupts(Flags::DRE);

                                    if self.spi_state.direction == Direction::OutOnly
                                    && self.spi_state.sent == self.spi_state.expected {
                                        self.usb_state.direction = Direction::None;
                                        self.spi_state.direction = Direction::None;
                                    }
                                }

                                grant.release(1);
                            }
                            Err(nb::Error::WouldBlock) => {
                                // SPI isn't ready for the next byte
                            }
                            Err(_) => {
                                defmt::error!("error in flush_spi() Out|Both"); // TODO
                            }
                        };
                    }
                    Err(BBError::InsufficientSize) => {
                        // There's no more data in the buffer to write
                        // TODO We seem to get continual DRE callbacks; is that expected?
                    }
                    Err(BBError::GrantInProgress) => {
                        // TODO does this still hold once SPI is working?
                        // We'll hit this when the USB or SPI ISR interrupts the other;
                        // it's effectively a mutex
                    }
                    Err(BBError::AlreadySplit) => {
                        unreachable!();
                    }
                }
            }
            Direction::InOnly => {
                if self.spi_state.sent < self.spi_state.expected {
                    match self.spi.send(USB_SPI_TX_FILL) {
                        Ok(()) => {
                            self.spi_state.sent += 1;
                            defmt::info!("Fill sent"); // TODO

                            // The only way to clear DRE is to write data, so only
                            // enable it when we'll have more data to write.
                            if self.spi_state.sent < self.spi_state.expected {
                                self.spi.enable_interrupts(Flags::DRE);
                            } else {
                                self.spi.disable_interrupts(Flags::DRE);
                            }
                        }
                        Err(nb::Error::WouldBlock) => {
                            // SPI isn't ready for the next byte
                        }
                        Err(_) => {
                            defmt::error!("error in flush_spi() Direction::InOnly"); // TODO
                        }
                    };
                }
            }
            Direction::None => {
                defmt::info!("In flush_spi() with Direction::None");
            }
        }
    }

    // TODO split this in to another struct so users don't need two ISRs that both reference Self
    pub fn spi_callback(&mut self) {
        match self.spi_state.direction {
            Direction::InOnly | Direction::Both => {
                match self.spi_to_usb_producer.grant_exact(1) {
                    Ok(mut grant) => {
                        match self.spi.read() {
                            Ok(c) => {
                                defmt::info!("Read {:X}", c);
                                grant.buf()[0] = c;
                                grant.commit(1);
        
                                self.spi_state.received += 1;
                            }
                            Err(nb::Error::WouldBlock) => {
                                // Nothing to read here
                                // Drop the grant without committing
                            }
                            Err(nb::Error::Other(SpiError::Overflow)) => {
                                defmt::error!("Overflow"); // TODO
                            }
                        }
                    }
                    Err(_) => {
                        // TODO better error reporting
                        defmt::error!("SPI->USB overflow");
                    }
                }
            }

            Direction::OutOnly | Direction::None => {
                match self.spi.read() {
                    Ok(c) => {
                        defmt::info!("Discarded read character {:X}", c); // TODO
                        if self.spi_state.direction == Direction::OutOnly {
                            self.spi_state.received += 1;
                        }
                    }
                    Err(nb::Error::WouldBlock) => {
                        // Nothing to read here
                    }
                    Err(nb::Error::Other(SpiError::Overflow)) => {
                        defmt::error!("Overflow"); // TODO
                    }
                }
            }
        }

        self.flush_spi();

        match self.spi.read_flags_errors() {
            Ok(flags) => {
                if flags & !Flags::DRE != Flags::empty() {
                    defmt::info!("End of spi_callback() flags:{:X}", flags.bits());
                }
            }
            Err(SpiError::Overflow) => {
                defmt::info!("End of spi_callback() Error::Overflow");
            }
        }
    }

    pub fn select(&mut self, device_index: Option<usize>) {
        // TODO we'll want a mutex or similar to select this, due to deselect when finishing a transaction
        if device_index != self.selected_device {
            if let Some(index) = self.selected_device {
                self.devices[index].deselect();
            }
            if let Some(index) = device_index {
                self.devices[index].select();
            }
            self.selected_device = device_index;
        }
    }
}

impl<B, P, const DEVICE_COUNT: usize, const ENDPOINT_SIZE: usize> UsbClass<B> for UsbSpi<'_, B, P, DEVICE_COUNT, ENDPOINT_SIZE>
where
    B: UsbBus,
    P: ValidPads<SS = NoneT> + Tx + Rx,
{
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        writer.interface(
            self.usb_interface,
            USB_CLASS_VENDOR_SPECIFIC,
            USB_SUBCLASS_VENDOR_SPECIFIC_USB_SPI,
            USB_SPI_PROTOCOL_0,
        )?;

        writer.endpoint(&self.write_endpoint)?;
        writer.endpoint(&self.read_endpoint)?;

        Ok(())
    }

    fn reset(&mut self) {
        // TODO
        // self.read_buf.clear();
        // self.write_buf.clear();
        self.write_state = WriteState::NotFull;
    }

    fn poll(&mut self) {
        // Make sure we've got space to store new data, because once the
        // endpoint is read, the host can overwrite whatever data is in it.
        match self.usb_to_spi_producer.grant_exact(ENDPOINT_SIZE) {
            Ok(mut grant) => {
                let mut start_transaction = false;
                let mut continue_transaction = false;
                
                match (self.usb_state.direction, self.spi_state.direction) {
                    // Normal start to a transaction, should receive a header
                    (Direction::None, Direction::None) => {
                        start_transaction = true;
                    }

                    // More data for an ongoing transaction
                    (Direction::OutOnly, Direction::OutOnly) | (Direction::Both, Direction::Both) => {
                        continue_transaction = true;
                    }

                    _ => {
                        // TODO 
                        defmt::error!("USB OUT transfer when in an invalid state");
                    }
                }

                if start_transaction || continue_transaction {
                    let mut buf = [0u8; ENDPOINT_SIZE];
                    match self.read_endpoint.read(&mut buf) {
                        Ok(count) => {
                            if start_transaction {
                                if let Some(header) = TransferHeader::decode(&buf[..count]) {
                                    let header_len = core::mem::size_of::<TransferHeader>();
                        
                                    let direction = header.direction;
                
                                    // These differ if the SPI transfer is bigger than a USB packet
                                    let expected = header.bytes as usize;
                                    let received = count - header_len;
                
                                    if direction == Direction::OutOnly || direction == Direction::Both {
                                        grant.buf()[0..received].copy_from_slice(&buf[header_len..count]);
                                        grant.commit(received);
                                    }
                
                                    self.usb_state.direction = direction;
                                    self.usb_state.expected = expected;
                                    self.usb_state.received = received;
                                    self.usb_state.sent = 0;
                
                                    self.spi_state.direction = direction;
                                    self.spi_state.expected = expected;
                                    self.spi_state.received = 0;
                                    self.spi_state.sent = 0;
                                } else {
                                    // Header failed to decode
                                    unimplemented!();
                                }
                            } else {
                                grant.buf()[..count].copy_from_slice(&buf[..count]);
                                grant.commit(count);

                                self.usb_state.received += count;
                            }
            
                            self.flush_spi(); // NOP if the SPI is already busy
                        }
                        Err(UsbError::WouldBlock) => {
                            // No data to read, just drop the grant
                        }
                        Err(_) => {
                            // TODO handle this better
                            defmt::error!("Error reading OUT data");
                        }
                    }
                }
            }
            Err(_) => {
                // Since we don't have anywhere to put more data, we can't read
                // data out of the USB endpoint.

                self.flush_spi(); // Ensure we continue draining the buffer
            }
        }

        self.flush_usb();
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        if addr == self.write_endpoint.address() {
            // We've written a bulk transfer out; flush in case there's more buffered data
            self.flush_usb();
        }
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();

        
        if !(req.request_type == control::RequestType::Vendor
             && req.recipient == control::Recipient::Interface
             && req.index == u8::from(self.usb_interface) as u16) {
            return;
        }
            
        // defmt::info!("USB-SPI interface {:?} Control IN vendor:{:?} interface:{:?} request:{:?} value:{:?}",
        //     u8::from(self.usb_interface), req.request_type == control::RequestType::Vendor, req.index, req.request, req.value);

        match protocol::ControlIn::n(req.request) {
            Some(protocol::ControlIn::REQUEST_IN_HW_INFO) => {
                xfer.accept(|buf| 
                    Ok(protocol::MasterInfo::new(self.devices.len() as u16, BufferSize::to_u16())
                       .encode(buf))
                ).unwrap_or_else(|_| {
                    defmt::error!("USB-SPI Failed to accept REQUEST_IN_HW_INFO")
                });
            }
            Some(protocol::ControlIn::REQUEST_IN_GET_EVENT) => {
                unimplemented!();
            }
            Some(protocol::ControlIn::REQUEST_IN_LINUX_SLAVE_INFO) => {
                let i = usize::from(req.value);
                if i < self.devices.len() {
                    xfer.accept(|buf|
                        Ok(protocol::ConnectedSlaveInfoLinux::new(false, self.devices[i].modalias()).encode(buf))
                    ).unwrap_or_else(|_| {
                        defmt::error!("USB-SPI Failed to accept REQUEST_IN_LINUX_SLAVE_INFO")
                    });
                } else {
                    defmt::info!("USB-SPI rejecting out-of-range REQUEST_IN_LINUX_SLAVE_INFO");
                    xfer.reject().unwrap_or_else(|_| {
                        defmt::error!("USB-SPI Failed to reject control IN request")
                    });
                }
            }
            None => {
                defmt::info!("USB-SPI rejecting control_in request");
                xfer.reject().unwrap_or_else(|_| {
                    defmt::error!("USB-SPI Failed to reject control IN request")
                });
            }
        }
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();

        if !(req.request_type == control::RequestType::Vendor
            && req.recipient == control::Recipient::Interface
            && req.index == u8::from(self.usb_interface) as u16)
        {
            return;
        }

        defmt::info!("USB-SPI got a Control OUT request! {:?}", req.request);

        match protocol::ControlOut::n(req.request) {
            Some(protocol::ControlOut::SetSlave) => {
                match protocol::SetSlave::decode(xfer.data()) {
                    Some(setting) => {
                        let device_index = usize::from(setting.slave_id);
                        if device_index < self.devices.len() {
                            self.select(Some(device_index));
                        } else {
                            defmt::info!("USB-SPI rejecting out-of-range SetSlave request");
                            xfer.reject().unwrap_or_else(|_| {
                                defmt::error!("USB-SPI Failed to reject control OUT request")
                            });
                        }
                    }
                    None => {
                        defmt::warn!("USB-SPI rejecting corrupt SetSlave request");
                        xfer.reject().unwrap_or_else(|_| {
                            defmt::error!("USB-SPI Failed to reject control OUT request")
                        });
                    }
                }
            }
            None => {
                defmt::warn!("USB-SPI rejecting unknown control_out request {:?}", req.request);
                xfer.reject().unwrap_or_else(|_| {
                    defmt::error!("USB-SPI Failed to reject control OUT request")
                });
            }
        }
    }
}
