#![no_std]

use atsamd_hal::{
    ehal::spi::{self, Mode},
    prelude::*,
    sercom::spi::{
        BitOrder, Config, Duplex, EightBit, Error as SpiError, Flags, Master, Spi, Status,
        ValidPads,
    },
    typelevel::NoneT,
};

mod spi_device;
pub use spi_device::*;

// TODO use const generics, so our customers don't need to see this
pub use bbqueue::consts::*;
use typenum::marker_traits::Unsigned; // Also get rid of this dependency

// See note in Cargo.toml as to why this not heapless
use bbqueue::{BBBuffer, Consumer, Error as BBError, GrantR, GrantW, Producer};
use core::ptr::NonNull;
use cortex_m::interrupt;
use heapless::spsc;
use protocol::Error;
pub use protocol::SpiDeviceCapabilities;
use protocol::{Direction, Event, TransferHeader, TransferMode};
use usb_device::{class_prelude::*, Result as UsbResult};
use usb_spi_protocol as protocol;

type UsbSpiResult<T> = Result<T, Error>;

const USB_CLASS_VENDOR_SPECIFIC: u8 = 0xFF; // TODO push up to a protocol crate
const USB_SUBCLASS_VENDOR_SPECIFIC_USB_SPI: u8 = 0x02; // TODO push up to a protocol crate
const USB_SPI_PROTOCOL_0: u8 = 0x00; // TODO push up to a protocol crate

/// SPI reads require writes, use this when there isn't other data to write.
const USB_SPI_TX_FILL: u8 = 0x00;

pub type SpiDeviceList<'a> = [&'a mut dyn SpiDevice];

// This needs to be at least as big as the USB bulk endpoint size
type BufferSize = U256;

/// If this many full size packets have been sent in a row, a short packet will
/// be sent so that the host sees the data in a timely manner.
const SHORT_PACKET_INTERVAL: usize = 10;

// TODO rename here and in atsamd-usbd-uart
// TODO add back the idle state, transition to it in ep_in_complete() when
// appropriate, and maybe make flush_usb() conditional on it
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
/// or write independently.  We also might make longer SPI transactions than
/// will fit in a single USB transaction.
struct TransactionState {
    direction: Direction,
    expected: usize,
    /// Count of words loaded in to the USB (OUT) -> SPI (MOSI) buffer
    from_usb: usize,
    /// Count of words from the SPI that have been handled in a USB context
    from_spi: usize,
}

/// TODO could turn this in to one of those fancy typestatemachines...
impl TransactionState {
    fn new() -> Self {
        Self {
            direction: Direction::None,
            expected: 0,
            from_usb: 0,
            from_spi: 0,
        }
    }

    fn reset(&mut self) {
        *self = Self::new();
    }
}

/// Queue for events - mainly interrupts from attached chips
///
/// This should be sized such that all attached IRQs can fire simultaneously.
/// The USB interface moves one event at a time as a control-IN transfer, so
/// this is not a good design if very frequent interrupting is expected.  The
/// host driver decides how frequently to poll, and could vary that interval.
///
/// Down the track, a header could be used for the bulk IN transfers, to send
/// events with a higher bandwidth and probably lower latency.
pub type EventQueue = spsc::Queue<Event, 16>; // stores N-1=15 elements
pub type EventConsumer<'a> = spsc::Consumer<'a, Event, 16>;
pub type EventProducer<'a> = spsc::Producer<'a, Event, 16>;

/// TX and RX buffers used by the UsbSide
///
/// This structure should never be moved in/ memory once it's in use, both the
/// UsbSide and SpiSide have pointers to it.
pub struct Static<P, const DEVICE_COUNT: usize>
where
    P: ValidPads<SS = NoneT, Capability = Duplex>,
{
    /// For data from the SPI to the USB
    rx_buffer: BBBuffer<BufferSize>,

    /// Other direction from `rx_buffer`
    tx_buffer: BBBuffer<BufferSize>,

    /// Queue of interrupt or similar events waiting for the host to fetch
    event_queue: EventQueue,

    /// The enabled SPI hardware
    spi: Spi<Config<P, Master, EightBit>, Duplex>,

    /// External SPI devices we're connected to
    devices: [&'static mut dyn SpiDevice; DEVICE_COUNT],

    /// Index in to Self.devices of the currently-selected device
    selected_device: Option<usize>,
}

// TODO add enable+disable methods and events for the attached devices.  Our driver could then
// register/deregister the drivers for those devices, and our user could activate/deactivate
// based on the hardware reset signal.
//
/// A USB to SPI USB class
pub struct UsbSide<'a, B, P, const DEVICE_COUNT: usize, const ENDPOINT_SIZE: usize>
where
    B: UsbBus,
    P: ValidPads<SS = NoneT, Capability = Duplex>,
{
    usb_interface: InterfaceNumber,
    read_endpoint: EndpointOut<'a, B>,
    write_endpoint: EndpointIn<'a, B>,

    /// SPI->USB buffer
    spi_to_usb_consumer: Consumer<'a, BufferSize>,

    /// USB->SPI buffer
    usb_to_spi_producer: Producer<'a, BufferSize>,

    /// USB IN endpoint state
    write_state: WriteState,

    /// Only modified in a USB interrupt context
    transaction_state: TransactionState,

    event_consumer: EventConsumer<'a>,

    statics: NonNull<Static<P, DEVICE_COUNT>>,
}

/// Used by the SPI ISR, handles the SPI hardware
pub struct SpiSide<'a, P, const DEVICE_COUNT: usize>
where
    P: ValidPads<SS = NoneT, Capability = Duplex>,
{
    /// SPI->USB buffer
    spi_to_usb_producer: Producer<'a, BufferSize>,

    /// USB->SPI buffer
    usb_to_spi_consumer: Consumer<'a, BufferSize>,

    event_producer: EventProducer<'a>,

    statics: NonNull<Static<P, DEVICE_COUNT>>,
}

impl<P, const DEVICE_COUNT: usize> Static<P, DEVICE_COUNT>
where
    P: ValidPads<SS = NoneT, Capability = Duplex>,
{
    pub fn new(
        spi_hardware: Config<P, Master, EightBit>,
        mut devices: [&'static mut dyn SpiDevice; DEVICE_COUNT],
    ) -> Self {
        for device in &mut devices {
            device.deselect();
            device.assert_reset();
        }

        // The default mode/speed don't matter, they'll be set in select_device()
        let mut spi = spi_hardware.enable();

        spi.enable_interrupts(Flags::RXC);

        Self {
            rx_buffer: BBBuffer::new(),
            tx_buffer: BBBuffer::new(),
            event_queue: EventQueue::new(),
            spi,
            devices,
            selected_device: None,
        }
    }

    pub fn split<'a, B, const ENDPOINT_SIZE: usize>(
        &'a mut self,
        alloc: &'a UsbBusAllocator<B>,
    ) -> (
        UsbSide<'a, B, P, DEVICE_COUNT, ENDPOINT_SIZE>,
        SpiSide<'a, P, DEVICE_COUNT>,
    )
    where
        B: UsbBus,
    {
        // No need to track whether self has been split, because the bbqueues do
        let (spi_to_usb_producer, spi_to_usb_consumer) = self.rx_buffer.try_split().unwrap();
        let (usb_to_spi_producer, usb_to_spi_consumer) = self.tx_buffer.try_split().unwrap();

        let (usb_statics, spi_statics) = unsafe {
            (
                NonNull::new_unchecked(self as *const _ as *mut _),
                NonNull::new_unchecked(self as *const _ as *mut _),
            )
        };

        let (event_producer, event_consumer) = self.event_queue.split();

        (
            UsbSide {
                usb_interface: alloc.interface(),
                read_endpoint: alloc.bulk(ENDPOINT_SIZE as u16),
                write_endpoint: alloc.bulk(ENDPOINT_SIZE as u16),
                spi_to_usb_consumer,
                usb_to_spi_producer,
                transaction_state: TransactionState::new(),
                write_state: WriteState::NotFull,
                event_consumer,
                statics: usb_statics,
            },
            SpiSide {
                spi_to_usb_producer,
                usb_to_spi_consumer,
                event_producer,
                statics: spi_statics,
            },
        )
    }

    fn deselect_device(&mut self) {
        interrupt::free(|_| {
            // TODO implement the minimum level scheme instead
            if let Some(index) = self.selected_device {
                self.devices[index].deselect();
                self.selected_device = None;
            }
        });
    }

    /// Asserts chip select for the specified device, and sets up SPI peripheral
    fn select_device(
        &mut self,
        index: usize,
        clock_speed: u32,
        mode: Mode,
        bit_order: BitOrder,
    ) -> UsbSpiResult<()> {
        interrupt::free(|_| {
            // TODO implement the minimum level scheme instead
            if Some(index) != self.selected_device {
                if let Some(old_index) = self.selected_device {
                    self.devices[old_index].deselect();
                }

                let device = if index >= DEVICE_COUNT {
                    self.event_queue
                        .enqueue(Event {
                            event_type: protocol::EventType::Error,
                            data: Error::IndexOutOfRange as u16,
                        })
                        .unwrap_or_else(|_| {
                            defmt::error!(
                                "Failed to enqueue IndexOutOfRange error in select_device()"
                            );
                        });

                    return Err(Error::IndexOutOfRange);
                } else {
                    &mut self.devices[index]
                };

                let capabilities = device.capabilities();

                if clock_speed > device.max_clock_speed_hz()
                    || match mode {
                        spi::MODE_0 => (capabilities & SpiDeviceCapabilities::MODE_0).is_empty(),
                        spi::MODE_1 => (capabilities & SpiDeviceCapabilities::MODE_1).is_empty(),
                        spi::MODE_2 => (capabilities & SpiDeviceCapabilities::MODE_2).is_empty(),
                        spi::MODE_3 => (capabilities & SpiDeviceCapabilities::MODE_3).is_empty(),
                    }
                    || bit_order == BitOrder::MsbFirst
                        && (capabilities & SpiDeviceCapabilities::MSB_FIRST).is_empty()
                    || bit_order == BitOrder::LsbFirst
                        && (capabilities & SpiDeviceCapabilities::LSB_FIRST).is_empty()
                {
                    self.event_queue
                        .enqueue(Event {
                            event_type: protocol::EventType::Error,
                            data: Error::UnsupportedByDevice as u16,
                        })
                        .unwrap_or_else(|_| {
                            defmt::error!(
                                "Failed to enqueue UnsupportedByDevice error in select_device()"
                            );
                        });
                    Err(Error::UnsupportedByDevice)
                } else {
                    self.spi.reconfigure(|config| {
                        config.set_baud(clock_speed.Hz());
                        config.set_spi_mode(mode);
                        config.set_bit_order(bit_order);
                    });

                    device.select();
                    self.selected_device = Some(index);
                    Ok(())
                }
            } else {
                Ok(()) // No change
            }
        })
    }

    fn pre_word_write(&mut self) {
        if let Some(index) = self.selected_device {
            self.devices[index].pre_word_write();
        }
    }
}

impl<'a, B, P, const DEVICE_COUNT: usize, const ENDPOINT_SIZE: usize>
    UsbSide<'a, B, P, DEVICE_COUNT, ENDPOINT_SIZE>
where
    B: UsbBus,
    P: ValidPads<SS = NoneT, Capability = Duplex>,
{
    fn deselect_device(&mut self) {
        // Safe because Static::deselect_device() does its work in an ISR free block
        unsafe { self.statics.as_mut() }.deselect_device();
    }

    /// Triggers an SPI interrupt if SPI isn't already transferring data
    fn run_spi(&mut self) {
        // Safe because this is an atomic write
        unsafe { self.statics.as_mut() }
            .spi
            .enable_interrupts(Flags::DRE);
    }

    fn maybe_done(&mut self) {
        if self.transaction_state.from_spi >= self.transaction_state.expected {
            if self.transaction_state.from_spi > self.transaction_state.expected {
                defmt::error!("Received more than expected from SPI");
            }
            self.deselect_device();
            self.transaction_state.reset();
        }
    }

    /// Helper for flush_usb() when in state InOnly Both transaction states
    fn write_usb_in(&mut self, grant: GrantR<BufferSize>) {
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
                self.transaction_state.from_spi += count;

                self.maybe_done();

                self.write_state = if count >= ENDPOINT_SIZE {
                    WriteState::Full(full_packet_count + 1)
                } else {
                    WriteState::NotFull
                };
            }

            Err(UsbError::WouldBlock) => {}

            Err(_) => {
                defmt::error!("Error writing packet"); // TODO
            }
        }
    }

    // TODO perhaps a better name; also used in transactions that don't send USB
    pub fn flush_usb(&mut self) {
        match self.transaction_state.direction {
            Direction::None => {}
            Direction::OutOnly => {
                // Host isn't intersted in SPI reads
                match self.spi_to_usb_consumer.read() {
                    Ok(grant) => {
                        let len = grant.buf().len();
                        self.transaction_state.from_spi += len;
                        grant.release(len);

                        self.maybe_done();
                    }
                    _ => {}
                }
            }
            Direction::InOnly | Direction::Both => {
                if self.transaction_state.direction == Direction::InOnly
                    && self.transaction_state.expected < self.transaction_state.from_usb
                {
                    let remaining =
                        self.transaction_state.expected - self.transaction_state.from_usb;
                    if let Ok(mut grant) = self.usb_to_spi_producer.grant_max_remaining(remaining) {
                        let generated = grant.buf().len();

                        for b in grant.buf() {
                            *b = USB_SPI_TX_FILL;
                        }

                        grant.commit(generated);
                        self.transaction_state.from_usb += generated;
                        self.run_spi();
                    }
                }

                match self.spi_to_usb_consumer.read() {
                    Ok(grant) => {
                        self.write_usb_in(grant);
                    }

                    // TODO is WriteState important now that we're not using CDC?
                    // No more SPI data to write over USB
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
                                    defmt::error!("Error writing ZLP"); // TODO
                                }
                            }
                        } else {
                            self.write_state = WriteState::NotFull;
                        }
                    }

                    Err(_) => {
                        // TODO handle this better
                        defmt::error!("Couldn't get spi_to_usb_consumer grant");
                    }
                }
            }
        }
    }

    /// Process a USB packet that starts a new transaction
    fn start_transaction(&mut self, mut grant: GrantW<BufferSize>) {
        let mut buf = [0u8; ENDPOINT_SIZE];
        // This is rather inefficient; there's a memcpy() in the read(), then we do it again...
        match self.read_endpoint.read(&mut buf) {
            Ok(count) => {
                if let Some(header) = TransferHeader::decode(&buf[..count]) {
                    let direction = header.direction;
                    let header_len = core::mem::size_of::<TransferHeader>();

                    // These differ if the SPI transfer is bigger than a USB packet
                    let expected = header.bytes as usize;
                    let received = count - header_len;

                    match direction {
                        Direction::OutOnly | Direction::Both | Direction::InOnly => {
                            let index = usize::from(header.index);

                            // Safe because Static::select_device() does its work in an ISR free block
                            if let Err(_) = unsafe { self.statics.as_mut() }.select_device(
                                index,
                                header.speed_hz,
                                match TransferMode::from_bits_truncate(header.mode)
                                    & TransferMode::MODE_MASK
                                {
                                    TransferMode::MODE_0 => spi::MODE_0,
                                    TransferMode::MODE_1 => spi::MODE_1,
                                    TransferMode::MODE_2 => spi::MODE_2,
                                    TransferMode::MODE_3 => spi::MODE_3,
                                    _ => unreachable!(),
                                },
                                BitOrder::MsbFirst,
                            ) {
                                // select_device() has enqueued the error message
                                return;
                            }
                        }
                        _ => {}
                    }
                    match direction {
                        Direction::OutOnly | Direction::Both => {
                            grant.buf()[..received].copy_from_slice(&buf[header_len..count]);
                            grant.commit(received);

                            self.transaction_state.direction = direction;
                            self.transaction_state.expected = expected;
                            self.transaction_state.from_usb = received;
                            self.transaction_state.from_spi = 0;

                            self.run_spi();
                        }
                        Direction::InOnly => {
                            let generated = expected.min(grant.buf().len());

                            for b in &mut grant.buf()[..generated] {
                                *b = USB_SPI_TX_FILL;
                            }
                            grant.commit(generated);

                            self.transaction_state.direction = direction;
                            self.transaction_state.expected = expected;
                            self.transaction_state.from_usb = generated;
                            self.transaction_state.from_spi = 0;

                            self.run_spi();
                        }
                        Direction::None => {
                            unreachable!();
                        }
                    }
                } else {
                    defmt::error!("Failed to decode header");
                }
            }
            Err(UsbError::WouldBlock) => {
                // No data to read from endpoint, just drop the grant
            }
            Err(_) => {
                // TODO handle this better
                defmt::error!("Error reading OUT data");
            }
        }
    }

    /// Process a USB packet that provides more OUT/MOSI data
    // TODO get rid of this.  We don't seem to make enough long SPI transactions
    // to justify it, and with this here need to carefully handle a lockup that
    // could happen if the SpiSide runs out of data at the same time as we
    // receive more
    fn continue_transaction(&mut self, mut grant: GrantW<BufferSize>) {
        match self.read_endpoint.read(grant.buf()) {
            Ok(count) => {
                grant.commit(count);

                self.transaction_state.from_usb += count;

                if self.transaction_state.from_usb > self.transaction_state.expected {
                    defmt::warn!("Received more than expected amount of data over USB");
                }

                self.run_spi();
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

impl<B, P, const DEVICE_COUNT: usize, const ENDPOINT_SIZE: usize> UsbClass<B>
    for UsbSide<'_, B, P, DEVICE_COUNT, ENDPOINT_SIZE>
where
    B: UsbBus,
    P: ValidPads<SS = NoneT, Capability = Duplex>,
{
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> UsbResult<()> {
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
        // TODO disable the SPI hardware while doing this
        loop {
            match self.spi_to_usb_consumer.read() {
                Ok(grant) => {
                    let length = grant.buf().len();
                    grant.release(length);
                }
                Err(BBError::InsufficientSize) => {
                    break;
                }
                _ => {
                    // TODO better
                    panic!("USB-SPI Unexpected result draining buffers for reset");
                }
            }
        }
        // TODO what to do about this?
        // loop {
        //     match self.usb_to_spi_consumer.read() {
        //         Ok(grant) => {
        //             let length = grant.buf().len();
        //             grant.release(length);
        //         }
        //         Err(BBError::InsufficientSize) => {
        //             break;
        //         }
        //         _ => {
        //             // TODO better
        //             panic!("USB-SPI Unexpected result draining buffers for reset");
        //         }
        //     }
        // }

        self.write_state = WriteState::NotFull;
        self.deselect_device();
        self.transaction_state.reset();
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        if addr != self.read_endpoint.address() {
            return;
        }

        // Make sure we've got space to store new data, because once the
        // endpoint is read, the host can overwrite whatever data is in it.
        if let Ok(grant) = self.usb_to_spi_producer.grant_exact(ENDPOINT_SIZE) {
            match self.transaction_state.direction {
                Direction::None => {
                    self.start_transaction(grant);
                }

                Direction::OutOnly | Direction::Both => {
                    if self.transaction_state.expected > self.transaction_state.from_usb {
                        self.continue_transaction(grant);
                    }
                }

                Direction::InOnly => {
                    // Waiting for an InOnly transaction to finish
                }
            }
        }
    }

    fn poll(&mut self) {
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
            && req.index == u8::from(self.usb_interface) as u16)
        {
            return;
        }

        // defmt::info!("USB-SPI interface {:?} Control IN vendor:{:?} interface:{:?} request:{:?} value:{:?}",
        //     u8::from(self.usb_interface), req.request_type == control::RequestType::Vendor, req.index, req.request, req.value);

        match protocol::ControlIn::n(req.request) {
            Some(protocol::ControlIn::HwInfo) => {
                xfer.accept(|buf| {
                    Ok(
                        // TODO grab the max clock speed from hardware
                        protocol::MasterInfo::new(
                            24_000_000,
                            DEVICE_COUNT as u16,
                            BufferSize::to_u16(),
                        )
                        .encode(buf),
                    )
                })
                .unwrap_or_else(|_| defmt::error!("USB-SPI Failed to accept REQUEST_IN_HW_INFO"));
            }
            Some(protocol::ControlIn::GetEvent) => {
                let event = self.event_consumer.dequeue().unwrap_or(Event {
                    event_type: protocol::EventType::NoEvent,
                    data: 0,
                });

                xfer.accept(|buf| Ok(event.encode(buf)))
                    .unwrap_or_else(|_| {
                        defmt::error!("USB-SPI Failed to accept REQUEST_IN_HW_INFO")
                    });
            }
            Some(protocol::ControlIn::LinuxSlaveInfo) => {
                let i = usize::from(req.value);
                if i < DEVICE_COUNT {
                    // Safe because the methods we call on device act on &self
                    let device = &unsafe { self.statics.as_mut() }.devices[i];

                    xfer.accept(|buf| {
                        Ok(protocol::ConnectedSlaveInfoLinux::new(
                            device.modalias(),
                            device.max_clock_speed_hz(),
                            device.capabilities(),
                        )
                        .encode(buf))
                    })
                    .unwrap_or_else(|_| {
                        defmt::error!("USB-SPI Failed to accept LinuxSlaveInfo request")
                    });
                } else {
                    defmt::info!("USB-SPI rejecting out-of-range LinuxSlaveInfo request");
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

        match protocol::ControlOut::n(req.request) {
            Some(protocol::ControlOut::AssertReset) => {
                let i = usize::from(req.value);
                if i < DEVICE_COUNT {
                    let device = &mut unsafe { self.statics.as_mut() }.devices[i];
                    device.assert_reset();

                    xfer.accept()
                        .unwrap_or_else(|_| defmt::error!("USB-SPI Failed to accept AssertReset"));
                } else {
                    defmt::info!("USB-SPI rejecting out-of-range AssertReset");
                    xfer.reject().unwrap_or_else(|_| {
                        defmt::error!("USB-SPI Failed to reject control OUT request")
                    });
                }
            }
            Some(protocol::ControlOut::DeassertReset) => {
                let i = usize::from(req.value);
                if i < DEVICE_COUNT {
                    let device = &mut unsafe { self.statics.as_mut() }.devices[i];
                    device.deassert_reset();

                    xfer.accept().unwrap_or_else(|_| {
                        defmt::error!("USB-SPI Failed to accept DeassertReset")
                    });
                } else {
                    defmt::info!("USB-SPI rejecting out-of-range DeassertReset");
                    xfer.reject().unwrap_or_else(|_| {
                        defmt::error!("USB-SPI Failed to reject control OUT request")
                    });
                }
            }
            None => {
                defmt::warn!(
                    "USB-SPI rejecting unknown control_out request {:?}",
                    req.request
                );
                xfer.reject().unwrap_or_else(|_| {
                    defmt::error!("USB-SPI Failed to reject control OUT request")
                });
            }
        }
    }
}

impl<'a, P, const DEVICE_COUNT: usize> SpiSide<'a, P, DEVICE_COUNT>
where
    P: ValidPads<SS = NoneT, Capability = Duplex>,
{
    /// This is only ever called in an SPI interrupt context
    pub fn spi_callback(&mut self) {
        // Safe because the only other access to Static::spi is
        // UsbSide::run_spi()
        let spi = &mut unsafe { self.statics.as_mut() }.spi;

        match self.spi_to_usb_producer.grant_exact(1) {
            Ok(mut grant) => {
                match spi.read() {
                    Ok(c) => {
                        grant.buf()[0] = c;
                        grant.commit(1);
                    }
                    Err(nb::Error::WouldBlock) => {
                        // Nothing to read from SPI
                        // Drop the grant without committing
                    }
                    Err(nb::Error::Other(SpiError::Overflow)) => {
                        defmt::error!("SPI Overflow in callback read"); // TODO
                        spi.clear_status(Status::BUFOVF);
                    }
                    Err(nb::Error::Other(SpiError::LengthError)) => {
                        defmt::error!("SPI Length Error in callback write"); // TODO
                    }
                }
            }
            Err(BBError::InsufficientSize) => {
                // TODO better error reporting
                defmt::error!("SPI->USB overflow");
                match spi.read() {
                    Ok(_discard) => {}
                    Err(nb::Error::WouldBlock) => {}
                    Err(nb::Error::Other(SpiError::Overflow)) => {
                        defmt::error!("SPI Overflow in callback discard");
                        spi.clear_status(Status::BUFOVF);
                    }
                    Err(nb::Error::Other(SpiError::LengthError)) => {
                        defmt::error!("SPI Length Error in callback write"); // TODO
                    }
                }
            }
            Err(BBError::GrantInProgress) => {
                defmt::panic!("GrantInProgress in spi_callback()"); // TODO
            }
            Err(BBError::AlreadySplit) => {
                unreachable!();
            }
        }

        match self.usb_to_spi_consumer.read() {
            Ok(grant) => {
                // The buffer returned is guaranteed to have at least one byte
                if spi.read_flags().contains(Flags::DRE) {
                    unsafe { self.statics.as_mut() }.pre_word_write();
                }
                match spi.send(grant.buf()[0]) {
                    Ok(()) => {
                        // Don't be tempted to disable DRE here; the ring buffer
                        // could have more data after a wrap.
                        grant.release(1);
                    }
                    Err(nb::Error::WouldBlock) => {
                        // SPI isn't ready for the next byte
                    }
                    Err(nb::Error::Other(SpiError::Overflow)) => {
                        defmt::error!("SPI Overflow in callback write"); // TODO
                        spi.clear_status(Status::BUFOVF);
                    }
                    Err(nb::Error::Other(SpiError::LengthError)) => {
                        defmt::error!("SPI Length Error in callback write"); // TODO
                    }
                };
            }
            Err(BBError::InsufficientSize) => {
                // There's no more data in the buffer to write
                spi.disable_interrupts(Flags::DRE);
            }
            Err(BBError::GrantInProgress) => {
                defmt::panic!("GrantInProgress in spi_callback()"); // TODO
            }
            Err(BBError::AlreadySplit) => {
                unreachable!();
            }
        }
    }

    /// Call when a particular device interrupts
    pub fn interrupt(&mut self, device_id: u16) {
        if (device_id as usize) < DEVICE_COUNT {
            self.event_producer
                .enqueue(Event {
                    event_type: protocol::EventType::Interrupt,
                    data: device_id,
                })
                .unwrap_or_else(|_| {
                    defmt::error!("Failed to enqueue interrupt for device {:?}", device_id);
                })
        } else {
            defmt::error!("UsbSide::interrupt() called with device_id higher than allowed");
        }
    }
}
