//! Octo SPI (OCTOSPI) bus
//!
//! See the parent module for documentation

#[allow(unused)] // TODO remove
use core::fmt;

use crate::{
    gpio::{
        gpioa::{PA1, PA2, PA3, PA6, PA7},
        gpiob::{PB0, PB1, PB10, PB12, PB13, PB2, PB6},
        gpioc::{PC1, PC10, PC11, PC2, PC3, PC5, PC9},
        gpiod::{PD11, PD12, PD13, PD4, PD5, PD6, PD7},
        gpioe::{PE10, PE11, PE2, PE7, PE8, PE9},
        gpiof::{
            PF0, PF1, PF10, PF11, PF12, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9,
        },
        gpiog::{PG0, PG1, PG10, PG11, PG12, PG14, PG15, PG6, PG7, PG9},
        gpioh::{PH2, PH3},
        Alternate, AF0, AF10, AF11, AF12, AF3, AF4, AF6, AF9,
    },
    rcc::{rec, CoreClocks, ResetEnable},
    stm32,
    time::{Hertz, MicroSeconds},
};

use super::{Config, Octospi, SamplingEdge};

/// A structure for specifying a Hyperbus configuration.
///
/// This structure uses builder semantics to generate the configuration. The
/// default values used are documented [`here`](#method.new), but in most cases
/// you will need to set at least some configuration parameters to those needed
/// for your memory device.
///
/// ```
/// let config = HyperbusConfig::new(50.mhz())
///     .device_size_bytes(24) // 16 Mbyte
///     .refresh_interval(4.us())
///     .read_write_recovery(4);
/// ```
#[derive(Copy, Clone)]
pub struct HyperbusConfig {
    pub(super) frequency: Hertz,
    size_order: u8,
    refresh_interval: MicroSeconds,
    chip_select_high: u8,
    read_write_recovery: u8,
    access_initial_latency: u8,
}

impl HyperbusConfig {
    /// Create a default Hyperbus configuration.
    ///
    /// # Arguments
    ///
    /// `frequency` - Bus clock frequency for the hyperbus interface.
    ///
    /// # Defaults
    ///
    /// * Device Size = 23 (8 MByte)
    /// * Refresh Interval = 4µs
    /// * Chip select high between transactions = 4
    /// * Read-write recovery = 4
    /// * Access initial latency = 6
    pub fn new<T: Into<Hertz>>(frequency: T) -> Self {
        HyperbusConfig {
            frequency: frequency.into(),
            size_order: 23, // 8 MByte
            refresh_interval: MicroSeconds(4),
            chip_select_high: 4,       // 40ns @ 100MHz
            read_write_recovery: 4,    // 40ns @ 100MHz
            access_initial_latency: 6, // 60ns @ 100MHz
        }
    }

    /// Number of bytes in the device, expressed as a power of 2.
    ///
    /// | Value | Memory Size (Bytes) | Memory Size (bits)
    /// |---|---|---
    /// | 23 | 8 MByte | 64 Mbit
    /// | 24 | 16 MByte | 128 Mbit
    /// | ... | |
    ///
    /// ```
    /// let hyperbusconfig = hyperbusconfig.device_size(23);
    /// ```
    pub fn device_size_bytes(mut self, size_order: u8) -> Self {
        debug_assert!(size_order > 4, "Memory size must be at least 32 bytes");
        debug_assert!(
            size_order <= 26,
            "Maximum memory size that can be mapped is 256 MBytes"
        );

        self.size_order = size_order;
        self
    }

    /// The refresh interval sets an upper limit on the length of read and write
    /// transactions, so that the distributed refresh mechanism in the memory
    /// can operate.
    ///
    /// Typically calculated by dividing the array refresh interval by the
    /// number of rows in the array, with some margin. Called t_CSM in the
    /// memory datasheet.
    ///
    /// Set to zero to disable the upper limit on the length of read and write
    /// transactions. In this case you become reponsible for issuing the reads
    /// needed to cover the required refreshes.
    ///
    /// ```
    /// let hyperbusconfig = hyperbusconfig.refresh_interval(4.us());
    /// ```
    pub fn refresh_interval(mut self, refresh_interval: MicroSeconds) -> Self {
        self.refresh_interval = refresh_interval;
        self
    }

    /// Chip select high time t_CSHI between transactions in clock cycles
    ///
    /// The chip select high time between transactions is specified in the
    /// memory datasheet. This should be converted to clock cycles based on the
    /// maximum bus frequency. The minimum is one cycle.
    ///
    /// ```
    /// let hyperbusconfig = hyperbusconfig.chip_select_high(4);
    /// ```
    pub fn chip_select_high(mut self, chip_select_high: u8) -> Self {
        debug_assert!(
            chip_select_high > 0,
            "There is a minimum of one clock cycle between transcations"
        );
        #[cfg(feature = "rm0468")]
        debug_assert!(
            chip_select_high <= 64,
            "Maximum 64 cycles between transcations"
        );
        #[cfg(not(feature = "rm0468"))]
        debug_assert!(
            chip_select_high <= 8,
            "Maximum 8 cycles between transcations"
        );

        self.chip_select_high = chip_select_high;
        self
    }

    /// Read-write recovery time t_RWR in clock cycles
    ///
    /// The read-write recovery time is specified in the memory datasheet. This
    /// should be converted to clock cycles based on the maximum bus frequency.
    ///
    /// ```
    /// let hyperbusconfig = hyperbusconfig.read_write_recovery(4);
    /// ```
    pub fn read_write_recovery(mut self, read_write_recovery: u8) -> Self {
        self.read_write_recovery = read_write_recovery;
        self
    }

    /// Initial access time t_ACC in clock cycles
    ///
    /// This parameter is given as a time period in the memory timing
    /// characteristics. However there is typically a default number of clock
    /// cycles used by the memory which can only be changed through a
    /// re-configuration. This parameter must be set equal to the currently
    /// configured number of cycles.
    ///
    /// ```
    /// let hyperbusconfig = hyperbusconfig.access_initial_latency(6);
    /// ```
    pub fn access_initial_latency(
        mut self,
        access_initial_latency: u8,
    ) -> Self {
        self.access_initial_latency = access_initial_latency;
        self
    }
}

/// Type for a Hyperbus interface
pub struct Hyperbus<OSPI> {
    rb: OSPI,

    /// Achieved bus frequency
    frequency: Hertz,

    /// Configured refresh interval in clock cycles
    refresh_cycles: u32,
}

impl<OSPI> fmt::Display for Hyperbus<OSPI> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Hyperbus with Clock {}; ", self.frequency)?;
        write!(f, "Refresh interval: {} cycles", self.refresh_cycles)
    }
}

pub trait PinClk<OSPI> {}
pub trait PinNclk<OSPI> {}
pub trait PinDQS<OSPI> {}
pub trait PinNCS<OSPI> {}
pub trait PinIo0<OSPI> {}
pub trait PinIo1<OSPI> {}
pub trait PinIo2<OSPI> {}
pub trait PinIo3<OSPI> {}
pub trait PinIo4<OSPI> {}
pub trait PinIo5<OSPI> {}
pub trait PinIo6<OSPI> {}
pub trait PinIo7<OSPI> {}

// impl<OSPI, CLK, NCS, IO0, IO1, IO2, IO3> PinsQuad<OSPI>
//     for (CLK, NCS, IO0, IO1, IO2, IO3)
// where
//     CLK: PinClk<OSPI>,
//     NCS: PinNCS<OSPI>,
//     IO0: PinIo0<OSPI>,
//     IO1: PinIo1<OSPI>,
//     IO2: PinIo2<OSPI>,
//     IO3: PinIo3<OSPI>,
// {
// }

macro_rules! pins {
    (
        $($BANK:ident:
            CLK: [$($CLK:ty),*] NCLK: [$($NCLK:ty),*] DQS: [$($DQS:ty),*] NCS: [$($NCS:ty),*]
                IO0: [$($IO0:ty),*] IO1: [$($IO1:ty),*] IO2: [$($IO2:ty),*] IO3: [$($IO3:ty),*]
                IO4: [$($IO4:ty),*] IO5: [$($IO5:ty),*] IO6: [$($IO6:ty),*] IO7: [$($IO7:ty),*]
        )*
    ) => {
        $(
            $(
                impl PinClk<stm32::$BANK> for $CLK {}
            )*
            $(
                impl PinNclk<stm32::$BANK> for $NCLK {}
            )*
            $(
                impl PinDQS<stm32::$BANK> for $DQS {}
            )*
            $(
                impl PinNCS<stm32::$BANK> for $NCS {}
            )*
            $(
                impl PinIo0<stm32::$BANK> for $IO0 {}
            )*
            $(
                impl PinIo1<stm32::$BANK> for $IO1 {}
            )*
            $(
                impl PinIo2<stm32::$BANK> for $IO2 {}
            )*
            $(
                impl PinIo3<stm32::$BANK> for $IO3 {}
            )*
            $(
                impl PinIo4<stm32::$BANK> for $IO4 {}
            )*
            $(
                impl PinIo5<stm32::$BANK> for $IO5 {}
            )*
            $(
                impl PinIo6<stm32::$BANK> for $IO6 {}
            )*
            $(
                impl PinIo7<stm32::$BANK> for $IO7 {}
            )*
        )*
    };
}

#[cfg(any(feature = "rm0468"))] // TODO
pins! {
    OCTOSPI1:
        CLK: [
            PA3<Alternate<AF12>>,
            PB2<Alternate<AF9>>,
            PF10<Alternate<AF9>>
        ]
        NCLK: [
            PB12<Alternate<AF3>>,
            PF11<Alternate<AF9>>
        ]
        DQS: [
            PA1<Alternate<AF12>>,
            PB2<Alternate<AF10>>,
            PC5<Alternate<AF10>>
        ]
        NCS: [
            PB6<Alternate<AF10>>,
            PB10<Alternate<AF9>>,
            PC11<Alternate<AF9>>,
            PE11<Alternate<AF11>>,
            PG6<Alternate<AF10>>
        ]
        IO0: [
            PA2<Alternate<AF6>>,
            PB1<Alternate<AF4>>,
            PB12<Alternate<AF12>>,
            PC3<Alternate<AF9>>,
            PC3<Alternate<AF0>>,
            PC9<Alternate<AF9>>,
            PD11<Alternate<AF9>>,
            PF8<Alternate<AF10>>
        ]
        IO1: [
            PB0<Alternate<AF4>>,
            PC10<Alternate<AF9>>,
            PD12<Alternate<AF9>>,
            PF9<Alternate<AF10>>
        ]
        IO2: [
            PA3<Alternate<AF6>>,
            PA7<Alternate<AF10>>,
            PB13<Alternate<AF4>>,
            PC2<Alternate<AF9>>,
            PC2<Alternate<AF0>>,
            PE2<Alternate<AF9>>,
            PF7<Alternate<AF10>>
        ]
        IO3: [
            PA1<Alternate<AF9>>,
            PA6<Alternate<AF6>>,
            PD13<Alternate<AF9>>,
            PF6<Alternate<AF10>>
        ]
        IO4: [
            PC1<Alternate<AF10>>,
            PD4<Alternate<AF10>>,
            PE7<Alternate<AF10>>,
            PH2<Alternate<AF9>>
        ]
        IO5: [
            PC2<Alternate<AF4>>,
            PC2<Alternate<AF0>>,
            PD5<Alternate<AF10>>,
            PE8<Alternate<AF10>>,
            PH3<Alternate<AF9>>
        ]
        IO6: [
            PC3<Alternate<AF4>>,
            PC3<Alternate<AF0>>,
            PD6<Alternate<AF10>>,
            PE9<Alternate<AF10>>,
            PG9<Alternate<AF9>>
        ]
        IO7: [
            PD7<Alternate<AF10>>,
            PE10<Alternate<AF10>>,
            PG14<Alternate<AF9>>
        ]
}
pins! {
    OCTOSPI2:
        CLK: [
            PF4<Alternate<AF9>>
        ]
        NCLK: [
            PF5<Alternate<AF9>>
        ]
        DQS: [
            PF12<Alternate<AF9>>,
            PG7<Alternate<AF9>>,
            PG15<Alternate<AF9>>
        ]
        NCS: [
            PG12<Alternate<AF3>>
        ]
        IO0: [
            PF0<Alternate<AF9>>
        ]
        IO1: [
            PF1<Alternate<AF9>>
        ]
        IO2: [
            PF2<Alternate<AF9>>
        ]
        IO3: [
            PF3<Alternate<AF9>>
        ]
        IO4: [
            PG0<Alternate<AF9>>
        ]
        IO5: [
            PG1<Alternate<AF9>>
        ]
        IO6: [
            PG10<Alternate<AF3>>
        ]
        IO7: [
            PG11<Alternate<AF9>>
        ]
}

pub trait OctospiExt<OSPI>: Sized {
    /// The `ResetEnable` singleton for this peripheral
    type Rec: ResetEnable;

    /// Create and enable the Octospi peripheral
    fn octospi_unchecked<CONFIG>(
        self,
        config: CONFIG,
        clocks: &CoreClocks,
        prec: Self::Rec,
    ) -> Octospi<OSPI>
    where
        CONFIG: Into<Config>;

    /// Create and enable the Octospi peripheral as a memory-mapped hyperbus
    fn octospi_hyperbus_unchecked<CONFIG>(
        self,
        config: CONFIG,
        clocks: &CoreClocks,
        prec: Self::Rec,
    ) -> Hyperbus<OSPI>
    where
        CONFIG: Into<HyperbusConfig>;
}

macro_rules! octospi_impl {
    ($name:ident, $name_hyperbus:ident, $peripheral:ty, $rec:ty, $memaddr:literal) => {
        impl Octospi<$peripheral> {
            pub fn $name<CONFIG>(
                regs: $peripheral,
                config: CONFIG,
                clocks: &CoreClocks,
                prec: $rec,
            ) -> Self
            where
                CONFIG: Into<Config>,
            {
                prec.enable();

                // Disable OCTOSPI before configuring it.
                regs.cr.write(|w| w.en().clear_bit());

                let spi_kernel_ck = Self::kernel_clk_unwrap(clocks).0;

                while regs.sr.read().busy().bit_is_set() {}

                let config: Config = config.into();

                // Clear all pending flags.
                regs.fcr.write(|w| {
                    w.ctof()
                        .set_bit()
                        .csmf()
                        .set_bit()
                        .ctcf()
                        .set_bit()
                        .ctef()
                        .set_bit()
                });

                // Configure the communication method for OCTOSPI.
                regs.cr.write(|w| unsafe {
                    w.fmode()
                        .bits(0) // indirect mode
                        .fthres()
                        .bits(config.fifo_threshold - 1)
                });

                regs.dcr1.write(|w| unsafe {
                    w.mtyp()
                        .bits(2) // standard mode
                        // Configure the FSIZE to maximum. It appears that even when addressing
                        // is not used, the flash size violation may still trigger.
                        .devsize()
                        .bits(0x1F)
                });

                // Communications configuration register
                regs.ccr.write(|w| unsafe {
                    w.dmode()
                        .bits(config.mode.reg_value())
                        .admode()
                        .bits(config.mode.reg_value())
                        .adsize()
                        .bits(0) // Eight-bit address
                        .imode()
                        .bits(0) // No instruction phase
                });

                // Prescaler
                let spi_frequency = config.frequency.0;
                let divisor =
                    match (spi_kernel_ck + spi_frequency - 1) / spi_frequency {
                        divisor @ 1..=256 => divisor - 1,
                        _ => panic!("Invalid OCTOSPI frequency requested"),
                    };
                regs.dcr2
                    .write(|w| unsafe { w.prescaler().bits(divisor as u8) });

                // Note that we default to setting SSHIFT (sampling on the falling
                // edge). This is because it appears that the QSPI may have signal
                // contention issues when reading with zero dummy cycles. Setting SSHIFT
                // forces the read to occur on the falling edge instead of the rising
                // edge. Refer to https://github.com/quartiq/stabilizer/issues/101 for
                // more information
                //
                // SSHIFT must not be set in DDR mode.
                regs.tcr.write(|w| unsafe {
                    w.sshift()
                        .bit(config.sampling_edge == SamplingEdge::Falling)
                        .dcyc()
                        .bits(config.dummy_cycles)
                });

                // Enable the peripheral
                regs.cr.modify(|_, w| w.en().set_bit());

                Octospi {
                    rb: regs,
                    mode: config.mode,
                }
            }

            pub fn $name_hyperbus<CONFIG>(
                regs: $peripheral,
                hyperbus: CONFIG,
                clocks: &CoreClocks,
                prec: $rec,
            ) -> Hyperbus<$peripheral>
            where
                CONFIG: Into<HyperbusConfig>,
            {
                prec.enable().reset();

                // Disable OCTOSPI before configuring it.
                regs.cr.write(|w| w.en().clear_bit());

                let spi_kernel_ck = Self::kernel_clk_unwrap(clocks).0;

                // Configure clock
                let hyperbus: HyperbusConfig = hyperbus.into();
                let spi_frequency = hyperbus.frequency.0;
                let divisor =
                    match (spi_kernel_ck + spi_frequency - 1) / spi_frequency {
                        divisor @ 1..=256 => divisor as u8,
                        _ => panic!("Invalid OCTOSPI frequency requested"),
                    };
                let frequency = Hertz(spi_kernel_ck / divisor as u32);

                // Calculate the achieved clock period in ns
                let period_ns = 1e9 * (divisor as f32) / (spi_kernel_ck as f32);
                let period_ns = period_ns as u32; // floor

                while regs.sr.read().busy().bit_is_set() {}

                // Clear all pending flags.
                regs.fcr.write(|w| {
                    w.ctof()
                        .set_bit()
                        .csmf()
                        .set_bit()
                        .ctcf()
                        .set_bit()
                        .ctef()
                        .set_bit()
                });

                // Configure the communication method for OCTOSPI
                regs.cr.write(|w| unsafe {
                    w.fmode()
                        .bits(3) // Memory-mapped
                        .fthres()
                        .bits(4 - 1) // TODO?
                });

                regs.dcr1.write(|w| unsafe {
                    w.mtyp()
                        .bits(4) // Hyperbus memory mode
                        .devsize()
                        .bits(hyperbus.size_order)
                        .csht()
                        .bits(hyperbus.chip_select_high - 1)
                });

                // Prescaler
                regs.dcr2
                    .write(|w| unsafe { w.prescaler().bits(divisor - 1) });
                // CS boundary. We actually use this feature to ensure the
                // transcation is re-started when crossing between each half of
                // the memory. These are separate dies on some parts (thus
                // separate transactions may be required) and has a very minimal
                // performance penalty if not.
                regs.dcr3.write(|w| unsafe {
                    w.csbound().bits(hyperbus.size_order - 1)
                });
                // Release nCS for refresh
                let refresh_cycles = {
                    let interval_ns =
                        (1000 * hyperbus.refresh_interval.0 as u32);
                    (interval_ns + period_ns - 1) / period_ns
                };
                regs.dcr4
                    .write(|w| unsafe { w.refresh().bits(refresh_cycles) });

                // 8-wide, DDR
                regs.ccr.write(|w| unsafe {
                    w.dqse()
                        .set_bit() // DQS enable
                        .ddtr()
                        .set_bit() // DDR mode enabled
                        .dmode()
                        .bits(4) // 8-wide
                        .adsize()
                        .bits(3) // 32-bit
                        .addtr()
                        .set_bit() // DDR mode enabled
                        .admode()
                        .bits(4) // 8-wide
                });
                // 8-wide, DDR
                regs.wccr.write(|w| unsafe {
                    w.dqse()
                        .set_bit() // DQS enable
                        .ddtr()
                        .set_bit() // DDR mode enabled
                        .dmode()
                        .bits(4) // 8-wide
                        .adsize()
                        .bits(3) // 32-bit
                        .addtr()
                        .set_bit() // DDR mode enabled
                        .admode()
                        .bits(4) // 8-wide
                });
                // TCR
                regs.tcr.write(|w| {
                    w.dhqc().set_bit() // Delay hold quarter cycle
                });

                // Hyperbus
                regs.hlcr.modify(|_, w| unsafe {
                    w.trwr()
                        .bits(hyperbus.read_write_recovery)
                        .tacc()
                        .bits(hyperbus.access_initial_latency)
                        .wzl()
                        .clear_bit() // latency on write accesses
                        .lm()
                        .set_bit() // fixed latency mode
                });

                Hyperbus {
                    rb: regs,
                    frequency,
                    refresh_cycles,
                }
            }
        }

        impl OctospiExt<$peripheral> for $peripheral {
            type Rec = $rec;

            fn octospi_unchecked<CONFIG>(
                self,
                config: CONFIG,
                clocks: &CoreClocks,
                prec: Self::Rec,
            ) -> Octospi<$peripheral>
            where
                CONFIG: Into<Config>,
            {
                Octospi::$name(self, config, clocks, prec)
            }
            fn octospi_hyperbus_unchecked<CONFIG>(
                self,
                config: CONFIG,
                clocks: &CoreClocks,
                prec: Self::Rec,
            ) -> Hyperbus<$peripheral>
            where
                CONFIG: Into<HyperbusConfig>,
            {
                Octospi::$name_hyperbus(self, config, clocks, prec)
            }
        }

        impl Hyperbus<$peripheral> {
            /// Initialise a memory-mapped Hyperbus peripheral and return a raw
            /// pointer to the memory
            pub fn init(self) -> *mut u32 {
                // Enable the peripheral
                self.rb.cr.modify(|_, w| w.en().set_bit());

                // Wait for the peripheral to indicate it is no longer busy
                while self.rb.sr.read().busy().bit_is_set() {}

                // Transition to memory-mapped mode
                self.rb.cr.modify(|_, w| unsafe {
                    w.fmode().bits(3) // Memory mapped
                });

                // Wait for the peripheral to indicate it is no longer busy
                while self.rb.sr.read().busy().bit_is_set() {}

                // Mapped to memory
                $memaddr as *mut u32
            }
        }
    };
}

#[cfg(any(feature = "rm0468"))] // TODO feature = "rm0455"
octospi_impl! {
    octospi1_unchecked,
    octospi1_hyperbus_unchecked,
    stm32::OCTOSPI1, rec::Octospi1, 0x9000_0000
}

octospi_impl! {
    octospi2_unchecked,
    octospi2_hyperbus_unchecked,
    stm32::OCTOSPI2, rec::Octospi2, 0x7000_0000
}
