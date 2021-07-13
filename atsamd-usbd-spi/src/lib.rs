#![no_std]

extern crate atsamd_hal;
extern crate defmt_rtt;

use atsamd_hal::{
    hal::serial::{
        // embedded_hal serial traits
        Read,
        Write,
    },
    prelude::*,
    time::Hertz,
};

// TODO use const generics, so our customers don't need to see this
pub use bbqueue::consts::*;

// See note in Cargo.toml as to why this not heapless
use bbqueue::{BBBuffer, Consumer, Error as BBError, Producer};
use core::convert::TryInto;
use usb_device::{class_prelude::*, Result};
use usb_spi_protocol as protocol;

/// This should be used as `device_class` when building the `UsbDevice`.
pub const USB_CLASS_CDC: u8 = 0x02;

const USB_CLASS_VENDOR_SPECIFIC: u8 = 0xFF; // TODO push up to a protocol crate
const USB_SUBCLASS_VENDOR_SPECIFIC_USB_SPI: u8 = 0x02; // TODO push up to a protocol crate
const USB_SPI_PROTOCOL_0: u8 = 0x00; // TODO push up to a protocol crate

/// TX and RX buffers used by the UsbSpi
///
/// Due to the BBQueue API, combined with Rust's rules about structs that
/// contain references to other members in the struct, we need a separate struct
/// to contain the BBQueue storage.  This structure should never be moved in
/// memory once it's in use, because any outstanding grant from before the move
/// would point to memory which is no longer inside the buffer.
pub struct UsbSpiStorage {
    /// For data from the UART to the USB; from the DCE to DTE, device to host
    rx_buffer: BBBuffer<U256>,
    /// Other direction from `rx_buffer`
    tx_buffer: BBBuffer<U256>,
}

impl UsbSpiStorage {
    pub fn new() -> Self {
        Self {
            rx_buffer: BBBuffer::new(),
            tx_buffer: BBBuffer::new(),
        }
    }
}

/// A USB CDC to SPI
pub struct UsbSpi<'a, B, const ENDPOINT_SIZE: usize>
where
    B: UsbBus,
{
    usb_interface: InterfaceNumber,
    read_endpoint: EndpointOut<'a, B>,
    write_endpoint: EndpointIn<'a, B>,

    /// UART end of the UART->USB buffer
    uart_to_usb_producer: Producer<'a, U256>,

    /// USB end of the UART->USB buffer
    uart_to_usb_consumer: Consumer<'a, U256>,

    /// USB end of the USB->UART buffer
    usb_to_uart_producer: Producer<'a, U256>,

    /// UART end of the USB->UART buffer
    usb_to_uart_consumer: Consumer<'a, U256>,

    write_state: WriteState,
}

/// If this many full size packets have been sent in a row, a short packet will
/// be sent so that the host sees the data in a timely manner.
const SHORT_PACKET_INTERVAL: usize = 10;

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

impl<'a, B, const ENDPOINT_SIZE: usize> UsbSpi<'a, B, ENDPOINT_SIZE>
where
    B: UsbBus,
{
    pub fn new(
        alloc: &'a UsbBusAllocator<B>,
        storage: &'a UsbSpiStorage,
    ) -> Self {
        let (uart_to_usb_producer, uart_to_usb_consumer) = storage.rx_buffer.try_split().unwrap();
        let (usb_to_uart_producer, usb_to_uart_consumer) = storage.tx_buffer.try_split().unwrap();

        Self {
            usb_interface: alloc.interface(),
            read_endpoint: alloc.bulk(ENDPOINT_SIZE as u16),
            write_endpoint: alloc.bulk(ENDPOINT_SIZE as u16),

            uart_to_usb_producer,
            uart_to_usb_consumer,
            usb_to_uart_producer,
            usb_to_uart_consumer,

            write_state: WriteState::NotFull,
        }
    }

    // TODO perhaps a better name
    pub fn flush_usb(&mut self) {
        match self.uart_to_usb_consumer.read() {
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
                defmt::error!("Couldn't get uart_to_usb_consumer grant");
                // Ok(())
            }
        }
    }

    // TODO perhaps a better name
    /// Attempts to write a byte out to the UART
    fn flush_spi(&mut self) {
        // Try to send the next available byte
        match self.usb_to_uart_consumer.read() {
            Ok(grant) => {
                // // The buffer returned is guaranteed to have at least one byte
                // // write() clears the TXC flag if it's set
                // match self.uart.write(grant.buf()[0]) {
                //     // '+'
                //     Ok(()) => {
                //         grant.release(1);
                //     }
                //     Err(nb::Error::WouldBlock) => {
                //         // UART isn't ready for the next byte
                //     }
                //     Err(_) => {
                //         defmt::error!("error in flush_spi()"); // TODO
                //     }
                // };
            }
            Err(BBError::InsufficientSize) => {
                // // There's no more data in the buffer to write
                // self.uart.clear_flags(Flags::TXC);
            }
            Err(BBError::GrantInProgress) => {
                // We'll hit this when the USB or UART ISR interrupts the other;
                // it's effectively a mutex that protects the uart_tx
            }
            Err(BBError::AlreadySplit) => {
                unreachable!();
            }
        }
    }

    // TODO split this in to another struct so users don't need two ISRs that both reference Self
    pub fn spi_callback(&mut self) {
        // match self.uart_to_usb_producer.grant_exact(1) {
        //     Ok(mut grant) => {
        //         match self.uart.read() {
        //             Ok(c) => {
        //                 grant.buf()[0] = c;
        //                 grant.commit(1);
        //             }
        //             Err(nb::Error::WouldBlock) => {
        //                 // Nothing to read here
        //                 // Drop the grant without committing
        //             }
        //             Err(nb::Error::Other(error)) => {
        //                 match error {
        //                     // TODO the CDC standard probably has a way to report these
        //                     Error::ParityError => {
        //                         defmt::error!("UART RX ParityError");
        //                         self.uart.clear_status(Status::PERR);
        //                     }
        //                     Error::FrameError => {
        //                         defmt::error!("UART RX FrameError");
        //                         self.uart.clear_status(Status::FERR);
        //                     }
        //                     Error::Overflow => {
        //                         defmt::error!("UART RX Overflow");
        //                         self.uart.clear_status(Status::BUFOVF);
        //                     }
        //                     Error::InconsistentSyncField => {
        //                         defmt::error!("UART RX InconsistentSyncField");
        //                         self.uart.clear_status(Status::ISF);
        //                     }
        //                     Error::CollisionDetected => {
        //                         defmt::error!("UART RX CollisionDetected");
        //                         self.uart.clear_status(Status::COLL);
        //                     }
        //                 }
        //             }
        //         }
        //     }

        //     Err(_) => {
        //         // No room in the uart_to_usb buffer
        //         // TODO ensure that the DTR does what it's supposed to in this case
        //     }
        // }

        self.flush_spi();
    }
}

impl<B, const ENDPOINT_SIZE: usize> UsbClass<B> for UsbSpi<'_, B, ENDPOINT_SIZE>
where
    B: UsbBus,
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
        // See if the host has more data to send over the UART
        match self.usb_to_uart_producer.grant_exact(ENDPOINT_SIZE) {
            Ok(mut grant) => {
                match self.read_endpoint.read(grant.buf()) {
                    Ok(count) => {
                        grant.commit(count);
                        // TODO note this is only reliable if the UART ISR is higher prio than USB; may be better to put that flag back...
                        self.flush_spi(); // NOP if the UART is already busy
                    }
                    Err(UsbError::WouldBlock) => {
                        // No data to read, just drop the grant
                    }
                    Err(_) => {
                        // TODO handle this better
                        defmt::error!("Error reading TX data");
                    }
                };
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
            
        defmt::info!("USB-SPI interface {:?} Control IN vendor:{:?} interface:{:?} request:{:?}",
            u8::from(self.usb_interface), req.request_type == control::RequestType::Vendor, req.index, req.request);

        match protocol::ControlIn::n(req.request) {
            Some(protocol::ControlIn::REQUEST_IN_HW_INFO) => {
                xfer.accept(|buf| 
                    Ok(protocol::MasterInfo::new(1) // TODO make this dynamic
                       .encode(buf))
                ).unwrap_or_else(|_| {
                    defmt::error!("USB-SPI Failed to accept REQUEST_IN_HW_INFO")
                });
            }
            Some(protocol::ControlIn::REQUEST_IN_GET_EVENT) => {
                unimplemented!();
            }
            Some(protocol::ControlIn::REQUEST_IN_LINUX_SLAVE_INFO) => {
                xfer.accept(|buf| 
                    Ok(protocol::ConnectedSlaveInfoLinux::new().encode(buf))
                ).unwrap_or_else(|_| {
                    defmt::error!("USB-SPI Failed to accept REQUEST_IN_LINUX_SLAVE_INFO")
                });
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

        match req.request {
            _ => {
                defmt::info!("USB-SPI rejecting control_out request");
                xfer.reject().unwrap_or_else(|_| {
                    defmt::error!("USB-SPI Failed to reject control OUT request")
                });
            }
        };
    }
}
