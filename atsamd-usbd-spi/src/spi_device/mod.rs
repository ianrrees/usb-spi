// embedded_hal digital output trait
use atsamd_hal::{
    ehal::digital::v2::OutputPin,
    gpio::{
        AnyPin,
        PushPullOutput,
    },
};
use cortex_m::asm::delay as cycle_delay;
use crate::protocol::SpiDeviceCapabilities;

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

    /// NOP for chips that don't have a reset
    fn assert_reset(&mut self);
    
    /// NOP for chips that don't have a reset
    fn deassert_reset(&mut self);

    /// modalias is how Linux SPI drivers associate devices with controllers
    fn modalias(&self) -> &'static str;

    /// Returns the maximum clock speed supported by the device, in Hz
    fn max_clock_speed_hz(&self) -> u32;

    /// Returns the capabilities bitmask associated with the device
    fn capabilities(&self) -> SpiDeviceCapabilities;

    /// Called in SPI interrupt context, just before a word is written
    ///
    /// This is a half-ass implementation of cs_change functionality.  Would be
    /// good if this could pause indefinitely, but for now just twiddling the CS
    /// GPIO is good enough.
    fn pre_word_write(&mut self);
}

pub enum CsMode {
    Normal,
    /// CS needs to be deasserted after each byte
    CsPerByte,
}

// TODO add interrupt
// TODO change this to accept embedded_hal's output pin
pub struct BasicSpiDevice<P, R>
where
    P: AnyPin<Mode = PushPullOutput>,
    R: AnyPin<Mode = PushPullOutput>, // TODO change this to an OptionalPin
{
    cs_pin: P,
    reset_pin: R,
    modalias: Option<&'static str>,
    max_clock_speed_hz: u32,
    capabilities: SpiDeviceCapabilities,
    cs_mode: CsMode,
    first_word: bool,
}

impl<P, R> BasicSpiDevice<P, R>
where
    P: AnyPin<Mode = PushPullOutput> + OutputPin,
    R: AnyPin<Mode = PushPullOutput> + OutputPin,
{
    /// Construct an SPI device that has both a chip select and reset pin
    ///
    /// reset_pin is assumed to be active low
    pub fn new(
        cs_pin: impl Into<P>,
        reset_pin: impl Into<R>,
        modalias: Option<&'static str>,
        max_clock_speed_hz: u32,
        capabilities: SpiDeviceCapabilities,
        cs_mode: CsMode,
    ) -> Self {
        Self {
            cs_pin: cs_pin.into(),
            reset_pin: reset_pin.into(),
            modalias,
            max_clock_speed_hz,
            capabilities,
            cs_mode,
            first_word: false,
        }
    }
}

impl<P, R> SpiDevice for BasicSpiDevice<P, R>
where
    P: AnyPin<Mode = PushPullOutput> + OutputPin,
    R: AnyPin<Mode = PushPullOutput> + OutputPin,
{
    fn select(&mut self) {
        self.cs_pin.set_low().ok().unwrap();
        self.first_word = true;
    }

    fn deselect(&mut self) {
        self.cs_pin.set_high().ok().unwrap();
    }

    fn assert_reset(&mut self) {
        self.reset_pin.set_low().ok().unwrap();
    }

    fn deassert_reset(&mut self) {
        self.reset_pin.set_high().ok().unwrap();
    }

    fn modalias(&self) -> &'static str {
        self.modalias.unwrap_or("")
    }

    fn max_clock_speed_hz(&self) -> u32 {
        self.max_clock_speed_hz
    }

    fn capabilities(&self) -> SpiDeviceCapabilities {
        self.capabilities
    }

    fn pre_word_write(&mut self) {
        match self.cs_mode {
            CsMode::Normal => {}
            CsMode::CsPerByte => {
                if !self.first_word {
                    self.deselect();
                    // TODO figure out a better way to do this, we're in the SERCOM ISR...
                    cycle_delay(1);
                    self.select();
                }
                self.first_word = false;
            }
        }
    }
}
