//! OCTOSPI Serial Peripheral Interface
//!
//! Notes on OCTOSPIM (OctoSPI manager / mux)
//! Some chips have an OCTOSPIM peripheral, such chips, like the STM32H735, have a default mapping as follows:
//!
//! RM0468 Rev 3:
//! "In the default out-of-reset configuration, all the OCTOSPI1 and OCTOSPI2 signals are mapped, respectively, on
//! Port 1 and on Port 2."
//!
//! This mapping is maintained by this implementation in that:
//! * OCTOSPIM.P1CR is used for OCTOSPI1.
//! * OCTOSPIM.P2CR is used for OCTOSPI2.
//!
//! However, it is possible to have a bootloader which does not follow this convention and runs your code from
//! memory-mapped flash, which will then break things when you use the functions in this module.
//! In this very special case, use a function that resides in RAM to re-configure the OctoSPI.
//!
//! Additionally, there are other issues when running code from memory-mapped mode, since the reference manual states:
//!
//! "The OCTOSPIM configuration can be changed only when all OCTOSPIs are disabled"
//!
//! this implementation does indeed disable both OCTOSPI peripherals before OCTOSPIM reconfiguration.
//!
//! Finally, for simplicity, this implementation currently enforces that DQS, NCS and CLK are on the same physical
//! group, however this is not a requirement of the hardware.  i.e. using P1_NCS and P1_CLK is ok, but P1_NCS and P2_CLK is
//! not and you will get a compile error if you try. PR's welcome to change this as needed.
#![macro_use]

pub mod enums;

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{Ordering, compiler_fence};
use core::task::Poll;

use embassy_embedded_hal::{GetConfig, SetConfig};
use embassy_hal_internal::PeripheralType;
use embassy_sync::waitqueue::AtomicWaker;
pub use enums::*;
use stm32_metapac::octospi::vals::{PhaseMode, SizeInBits};

use crate::dma::{ChannelAndRequest, word};
use crate::gpio::{AfType, Flex, OutputType, Pull, Speed};
use crate::interrupt::{self, typelevel::Interrupt};
use crate::mode::{Async, Blocking, Mode as PeriMode};
use crate::pac::octospi::{Octospi as Regs, vals};
#[cfg(octospim_v1)]
use crate::pac::octospim::Octospim;
use crate::rcc::{self, RccPeripheral};
use crate::{Peri, peripherals};

//
// OCTOSPIM Physical Groups
// bit 0 high = low group (if used), bit 1 high = hig group (if used)
// bit 1 high = port 2, bit 1 low = port 1
//

#[allow(unused)]
#[cfg(octospim_v1)]
mod octospin_v1_constants {
    pub(crate) const OCTOSPIM_P1_LOW: u8 = 0b00;
    pub(crate) const OCTOSPIM_P1_HIGH: u8 = 0b01;
    pub(crate) const OCTOSPIM_P2_LOW: u8 = 0b10;
    pub(crate) const OCTOSPIM_P2_HIGH: u8 = 0b11;
    pub(crate) const OCTOSPIM_P1_CTRL: u8 = 0b00;
    pub(crate) const OCTOSPIM_P2_CTRL: u8 = 0b10;
}
#[allow(unused)]
#[cfg(octospim_v1)]
pub use octospin_v1_constants::*;

/// OPSI driver config.
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Fifo threshold used by the peripheral to generate the interrupt indicating data
    /// or space is available in the FIFO
    pub fifo_threshold: FIFOThresholdLevel,
    /// Indicates the type of external device connected
    pub memory_type: MemoryType, // Need to add an additional enum to provide this public interface
    /// Defines the size of the external device connected to the OSPI corresponding
    /// to the number of address bits required to access the device.
    /// When using indirect mode, [`TransferConfig::address`] + the length of the data being read
    /// or written must fit within the configured `device_size`, otherwise an error is returned.
    pub device_size: MemorySize,
    /// Sets the minimum number of clock cycles that the chip select signal must be held high
    /// between commands
    pub chip_select_high_time: ChipSelectHighTime,
    /// Enables the free running clock
    pub free_running_clock: bool,
    /// Sets the clock level when the device is not selected
    pub clock_mode: bool,
    /// Indicates the wrap size corresponding to the external device configuration
    pub wrap_size: WrapSize,
    /// Specified the prescaler factor used for generating the external clock based
    /// on the AHB clock
    pub clock_prescaler: u8,
    /// Allows the delay of 1/2 cycle the data sampling to account for external
    /// signal delays
    pub sample_shifting: bool,
    /// Allows hold to 1/4 cycle the data
    pub delay_hold_quarter_cycle: bool,
    /// Enables the transaction boundary feature and defines the boundary to release
    /// the chip select
    pub chip_select_boundary: u8,
    /// Enables the delay block bypass so the sampling is not affected by the delay block
    pub delay_block_bypass: bool,
    /// Enables communication regulation feature. Chip select is released when the other
    /// OctoSpi requests access to the bus
    pub max_transfer: u8,
    /// Enables the refresh feature, chip select is released every refresh + 1 clock cycles
    pub refresh: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            fifo_threshold: FIFOThresholdLevel::_16Bytes, // 32 bytes FIFO, half capacity
            memory_type: MemoryType::Micron,
            device_size: MemorySize::Other(0),
            chip_select_high_time: ChipSelectHighTime::_5Cycle,
            free_running_clock: false,
            clock_mode: false,
            wrap_size: WrapSize::None,
            clock_prescaler: 0,
            sample_shifting: false,
            delay_hold_quarter_cycle: false,
            chip_select_boundary: 0, // Acceptable range 0 to 31
            delay_block_bypass: true,
            max_transfer: 0,
            refresh: 0,
        }
    }
}

/// OSPI transfer configuration.
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TransferConfig {
    /// Instruction width (IMODE)
    pub iwidth: OspiWidth,
    /// Instruction Id
    pub instruction: Option<u32>,
    /// Number of Instruction Bytes
    pub isize: AddressSize,
    /// Instruction Double Transfer rate enable
    pub idtr: bool,
    /// Address width (ADMODE)
    pub adwidth: OspiWidth,
    /// Device memory address.
    /// In indirect mode, this value + the length of the data being read or written must be within
    /// configured [`Config::device_size`], otherwise the transfer returns an error.
    pub address: Option<u32>,
    /// Number of Address Bytes
    pub adsize: AddressSize,
    /// Address Double Transfer rate enable
    pub addtr: bool,

    /// Alternate bytes width (ABMODE)
    pub abwidth: OspiWidth,
    /// Alternate Bytes
    pub alternate_bytes: Option<u32>,
    /// Number of Alternate Bytes
    pub absize: AddressSize,
    /// Alternate Bytes Double Transfer rate enable
    pub abdtr: bool,

    /// Data width (DMODE)
    pub dwidth: OspiWidth,
    /// Data Double Transfer rate enable
    pub ddtr: bool,

    /// Number of dummy cycles (DCYC)
    pub dummy: DummyCycles,

    /// Data strobe (DQS) management enable
    pub dqse: bool,
    /// Send instruction only once (SIOO) mode enable
    pub sioo: bool,
}

impl Default for TransferConfig {
    fn default() -> Self {
        Self {
            iwidth: OspiWidth::NONE,
            instruction: None,
            isize: AddressSize::_8Bit,
            idtr: false,

            adwidth: OspiWidth::NONE,
            address: None,
            adsize: AddressSize::_8Bit,
            addtr: false,

            abwidth: OspiWidth::NONE,
            alternate_bytes: None,
            absize: AddressSize::_8Bit,
            abdtr: false,

            dwidth: OspiWidth::NONE,
            ddtr: false,

            dummy: DummyCycles::_0,

            dqse: false,
            sioo: true,
        }
    }
}

/// OSPI autopoll configuration
pub struct AutopollConfig {
    /// Specifies the value to be compared with the masked status register to get a match.
    /// This parameter can be any value between 0 and 0xFFFFFFFF.
    pub match_value: u32,
    /// Specifies the mask to be applied to the status bytes received.
    /// This parameter can be any value between 0 and 0xFFFFFFFF,
    pub match_mask: u32,
    /// Specifies the method used for determining a match.
    pub match_mode: AutopollMatchMode,
    /// Specifies if automatic polling is stopped after a match.
    pub auto_stop: bool,
    /// Specifies the number of clock cycles between two read during automatic polling phases.
    pub interval: u16,
}

/// OSPI multiplex configuration
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MultiplexConfig {
    /// Time between two OCTOSPI requests and the OCTOSPIM acknowledge.
    pub req2ack_time: u8,
}

/// Error used for Octospi implementation
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OspiError {
    /// Peripheral configuration is invalid
    InvalidConfiguration,
    /// Operation configuration is invalid
    InvalidCommand,
    /// Size zero buffer passed to instruction
    EmptyBuffer,
    /// The transfer failed
    TransferError,
}

/// OSPI driver.
pub struct Ospi<'d, T: Instance, M: PeriMode> {
    _peri: Peri<'d, T>,
    _sck: Option<Flex<'d>>,
    _d0: Option<Flex<'d>>,
    _d1: Option<Flex<'d>>,
    _d2: Option<Flex<'d>>,
    _d3: Option<Flex<'d>>,
    _d4: Option<Flex<'d>>,
    _d5: Option<Flex<'d>>,
    _d6: Option<Flex<'d>>,
    _d7: Option<Flex<'d>>,
    _nss: Option<Flex<'d>>,
    _dqs: Option<Flex<'d>>,
    dma: Option<ChannelAndRequest<'d>>,
    _marker: PhantomData<M>,
    config: Config,
    width: OspiWidth,
}

impl<'d, T: Instance, M: PeriMode> Ospi<'d, T, M> {
    /// Enter memory mode.
    /// The Input `read_config` is used to configure the read operation in memory mode
    pub fn enable_memory_mapped_mode(
        &mut self,
        read_config: TransferConfig,
        write_config: TransferConfig,
        timeout: Option<u16>,
    ) -> Result<(), OspiError> {
        // Use configure command to set read config
        self.configure_command(&read_config, None)?;

        let reg = T::REGS;
        while reg.sr().read().busy() {}

        if let Some(instruction) = write_config.instruction {
            reg.wir().write(|r| {
                r.set_instruction(instruction);
            });
        }

        // Set writing configurations, there are separate registers for write configurations in memory mapped mode
        reg.wccr().modify(|w| {
            w.set_imode(PhaseMode::from_bits(write_config.iwidth.into()));
            w.set_idtr(write_config.idtr);
            w.set_isize(SizeInBits::from_bits(write_config.isize.into()));

            w.set_admode(PhaseMode::from_bits(write_config.adwidth.into()));
            w.set_addtr(write_config.addtr);
            w.set_adsize(SizeInBits::from_bits(write_config.adsize.into()));

            w.set_dmode(PhaseMode::from_bits(write_config.dwidth.into()));
            w.set_ddtr(write_config.ddtr);

            w.set_abmode(PhaseMode::from_bits(write_config.abwidth.into()));
            w.set_dqse(write_config.dqse);
        });

        reg.wtcr().modify(|w| w.set_dcyc(write_config.dummy.into()));

        reg.lptr().modify(|w| {
            w.set_timeout(timeout.unwrap_or(0));
        });

        // Enable memory mapped mode
        reg.cr().modify(|r| {
            r.set_fmode(crate::ospi::vals::FunctionalMode::MemoryMapped);
            r.set_tcen(timeout.is_some());
        });
        Ok(())
    }

    /// Quit from memory mapped mode
    pub fn disable_memory_mapped_mode(&mut self) {
        let reg = T::REGS;

        reg.cr().modify(|r| {
            r.set_fmode(crate::ospi::vals::FunctionalMode::IndirectWrite);
            r.set_abort(true);
            r.set_dmaen(false);
            r.set_en(false);
        });

        // Clear transfer complete flag
        reg.fcr().write(|w| w.set_ctcf(true));

        // Re-enable ospi
        reg.cr().modify(|r| {
            r.set_en(true);
        });
    }

    #[cfg(octospim_v1)]
    fn disable_octospis_for_octospim_config() -> (bool, bool) {
        let octospi1_enabled = crate::peripherals::OCTOSPI1::REGS.cr().read().en();

        #[cfg(all(octospim_v1, peri_octospi2))]
        let octospi2_enabled = crate::peripherals::OCTOSPI2::REGS.cr().read().en();

        #[cfg(not(all(octospim_v1, peri_octospi2)))]
        let octospi2_enabled = false;

        crate::peripherals::OCTOSPI1::REGS.cr().modify(|w| {
            w.set_en(false);
        });

        #[cfg(all(octospim_v1, peri_octospi2))]
        crate::peripherals::OCTOSPI2::REGS.cr().modify(|w| {
            w.set_en(false);
        });

        (octospi1_enabled, octospi2_enabled)
    }

    #[cfg(octospim_v1)]
    fn restore_octospis_after_config(octospi1_was_enabled: bool, _octospi2_was_enabled: bool) {
        if T::OCTOSPI_IDX == 1 || octospi1_was_enabled {
            crate::peripherals::OCTOSPI1::REGS.cr().modify(|w| {
                w.set_en(true);
            });
        }

        #[cfg(all(octospim_v1, peri_octospi2))]
        if T::OCTOSPI_IDX == 2 || _octospi2_was_enabled {
            crate::peripherals::OCTOSPI2::REGS.cr().modify(|w| {
                w.set_en(true);
            });
        }
    }

    #[cfg(octospim_v1)]
    fn octospim_low_data_src() -> u8 {
        if T::OCTOSPI_IDX == 1 { 0b00 } else { 0b10 }
    }

    #[cfg(octospim_v1)]
    fn octospim_high_data_src() -> u8 {
        if T::OCTOSPI_IDX == 1 { 0b01 } else { 0b11 }
    }

    #[cfg(octospim_v1)]
    fn octospim_signal_src() -> bool {
        T::OCTOSPI_IDX == 2
    }

    #[cfg(octospim_v1)]
    fn octospim_uses_p2(physical_group: u8) -> bool {
        physical_group & 0b10 != 0
    }

    #[cfg(octospim_v1)]
    fn octospim_uses_high_group(physical_group: u8) -> bool {
        physical_group & 0b01 != 0
    }

    #[cfg(octospim_v1)]
    fn configure_octospim_data_group(physical_group: u8, data_src: u8) {
        let use_high_group = Self::octospim_uses_high_group(physical_group);

        if Self::octospim_uses_p2(physical_group) {
            T::OCTOSPIM_REGS.p2cr().modify(|w| {
                if use_high_group {
                    w.set_iohen(true);
                    w.set_iohsrc(data_src);
                } else {
                    w.set_iolen(true);
                    w.set_iolsrc(data_src);
                }
            });
        } else {
            T::OCTOSPIM_REGS.p1cr().modify(|w| {
                if use_high_group {
                    w.set_iohen(true);
                    w.set_iohsrc(data_src);
                } else {
                    w.set_iolen(true);
                    w.set_iolsrc(data_src);
                }
            });
        }
    }

    #[cfg(octospim_v1)]
    fn configure_octospim_control_group(physical_group: u8, has_dqs: bool) {
        let signal_src = Self::octospim_signal_src();

        if Self::octospim_uses_p2(physical_group) {
            T::OCTOSPIM_REGS.p2cr().modify(|w| {
                w.set_clken(true);
                w.set_clksrc(signal_src);
                w.set_ncsen(true);
                w.set_ncssrc(signal_src);

                if has_dqs {
                    w.set_dqsen(true);
                    w.set_dqssrc(signal_src);
                } else {
                    w.set_dqsen(false);
                }
            });
        } else {
            T::OCTOSPIM_REGS.p1cr().modify(|w| {
                w.set_clken(true);
                w.set_clksrc(signal_src);
                w.set_ncsen(true);
                w.set_ncssrc(signal_src);

                if has_dqs {
                    w.set_dqsen(true);
                    w.set_dqssrc(signal_src);
                } else {
                    w.set_dqsen(false);
                }
            });
        }
    }

    #[cfg(octospim_v1)]
    fn configure_octospim_clk_group(physical_group: u8, has_dqs: bool, signal_src: bool) {
        if Self::octospim_uses_p2(physical_group) {
            T::OCTOSPIM_REGS.p2cr().modify(|w| {
                w.set_clken(true);
                w.set_clksrc(signal_src);

                if has_dqs {
                    w.set_dqsen(true);
                    w.set_dqssrc(signal_src);
                } else {
                    w.set_dqsen(false);
                }
            });
        } else {
            T::OCTOSPIM_REGS.p1cr().modify(|w| {
                w.set_clken(true);
                w.set_clksrc(signal_src);

                if has_dqs {
                    w.set_dqsen(true);
                    w.set_dqssrc(signal_src);
                } else {
                    w.set_dqsen(false);
                }
            });
        }
    }

    #[cfg(octospim_v1)]
    fn configure_octospim_ncs_group(physical_group: u8, signal_src: bool) {
        if Self::octospim_uses_p2(physical_group) {
            T::OCTOSPIM_REGS.p2cr().modify(|w| {
                w.set_ncsen(true);
                w.set_ncssrc(signal_src);
            });
        } else {
            T::OCTOSPIM_REGS.p1cr().modify(|w| {
                w.set_ncsen(true);
                w.set_ncssrc(signal_src);
            });
        }
    }

    #[cfg(octospim_v1)]
    fn enable_octospim_clock() {
        // RCC for octospim should be enabled before writing register
        #[cfg(stm32l4)]
        crate::pac::RCC.ahb2smenr().modify(|w| w.set_octospimsmen(true));
        #[cfg(stm32u5)]
        crate::pac::RCC.ahb2enr1().modify(|w| w.set_octospimen(true));
        #[cfg(not(any(stm32l4, stm32u5)))]
        crate::pac::RCC.ahb3enr().modify(|w| w.set_iomngren(true));
    }

    fn configure_ospi_registers(config: Config, dual_quad: bool) {
        // Device configuration
        T::REGS.dcr1().modify(|w| {
            w.set_devsize(config.device_size.into());
            w.set_mtyp(vals::MemType::from_bits(config.memory_type.into()));
            w.set_csht(config.chip_select_high_time.into());
            w.set_dlybyp(config.delay_block_bypass);
            w.set_frck(false);
            w.set_ckmode(config.clock_mode);
        });

        T::REGS.dcr2().modify(|w| {
            w.set_wrapsize(config.wrap_size.into());
        });

        T::REGS.dcr3().modify(|w| {
            w.set_csbound(config.chip_select_boundary);
            #[cfg(octospi_v1)]
            {
                w.set_maxtran(config.max_transfer);
            }
        });

        T::REGS.dcr4().modify(|w| {
            w.set_refresh(config.refresh);
        });

        T::REGS.cr().modify(|w| {
            w.set_fthres(vals::Threshold::from_bits(config.fifo_threshold.into()));
        });

        // Wait for busy flag to clear
        while T::REGS.sr().read().busy() {}

        T::REGS.dcr2().modify(|w| {
            w.set_prescaler(config.clock_prescaler);
        });

        T::REGS.cr().modify(|w| {
            w.set_dmm(dual_quad);
        });

        T::REGS.tcr().modify(|w| {
            w.set_sshift(match config.sample_shifting {
                true => vals::SampleShift::HalfCycle,
                false => vals::SampleShift::None,
            });
            w.set_dhqc(config.delay_hold_quarter_cycle);
        });

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };
    }

    fn enable_ospi(config: Config) {
        T::REGS.cr().modify(|w| {
            w.set_en(true);
        });

        // Free running clock needs to be set after peripheral enable
        if config.free_running_clock {
            T::REGS.dcr1().modify(|w| {
                w.set_frck(config.free_running_clock);
            });
        }
    }

    fn new_inner(
        peri: Peri<'d, T>,
        d0: Option<Flex<'d>>,
        d1: Option<Flex<'d>>,
        d2: Option<Flex<'d>>,
        d3: Option<Flex<'d>>,
        d4: Option<Flex<'d>>,
        d5: Option<Flex<'d>>,
        d6: Option<Flex<'d>>,
        d7: Option<Flex<'d>>,
        sck: Option<Flex<'d>>,
        nss: Option<Flex<'d>>,
        dqs: Option<Flex<'d>>,
        dma: Option<ChannelAndRequest<'d>>,
        config: Config,
        width: OspiWidth,
        dual_quad: bool,
        #[cfg(octospim_v1)] iol_pgroup: u8,
        #[cfg(octospim_v1)] ioh_pgroup: Option<u8>,
        #[cfg(octospim_v1)] ctrl_pgroup: u8,
    ) -> Self {
        #[cfg(octospim_v1)]
        trace!("OCTOSPI_IDX: {:?}", T::OCTOSPI_IDX);

        #[cfg(octospim_v1)]
        let (octospi1_was_enabled, octospi2_was_enabled) = {
            trace!("IOL_PGROUP: 0b{:02b}", iol_pgroup);
            if let Some(ioh_pgroup) = ioh_pgroup {
                trace!("IOH_PGROUP: 0b{:02b}", ioh_pgroup);
            } else {
                trace!("IOH_PGROUP: N/A");
            }
            trace!("CLK/NCS/DQS CTRL_PGROUP: 0b{:02b}", ctrl_pgroup);

            Self::enable_octospim_clock();

            let previously_enabled_instances = Self::disable_octospis_for_octospim_config();
            trace!(
                "OCTOSPI1_ENABLED: {:?}, OCTOSPI2_ENABLED: {:?}",
                previously_enabled_instances.0, previously_enabled_instances.1
            );

            // OctoSPI IO Manager has been enabled before
            T::OCTOSPIM_REGS.cr().modify(|w| {
                w.set_muxen(false);
                w.set_req2ack_time(0xff);
            });

            Self::configure_octospim_control_group(ctrl_pgroup, dqs.is_some());
            Self::configure_octospim_data_group(iol_pgroup, Self::octospim_low_data_src());

            if dual_quad {
                debug_assert!(
                    ioh_pgroup.is_some(),
                    "dual-quad must set ioh_pgroup for the second flash chip"
                );
            }

            if let Some(ioh_pgroup) = ioh_pgroup {
                Self::configure_octospim_data_group(ioh_pgroup, Self::octospim_high_data_src());
            }

            let cr = T::OCTOSPIM_REGS.cr().read();
            let p1cr = T::OCTOSPIM_REGS.p1cr().read();
            let p2cr = T::OCTOSPIM_REGS.p2cr().read();
            debug!("OCTOSPIM_CR: 0x{:08X} - {:?}", cr.0, cr);
            debug!("OCTOSPIM_P1CR: 0x{:08X} - {:?}", p1cr.0, p1cr);
            debug!("OCTOSPIM_P2CR: 0x{:08X} - {:?}", p2cr.0, p2cr);

            previously_enabled_instances
        };

        // System configuration
        rcc::enable_and_reset::<T>();
        while T::REGS.sr().read().busy() {}

        Self::configure_ospi_registers(config, dual_quad);

        #[cfg(octospim_v1)]
        Self::restore_octospis_after_config(octospi1_was_enabled, octospi2_was_enabled);

        // Enable peripheral
        Self::enable_ospi(config);

        Self {
            _peri: peri,
            _sck: sck,
            _d0: d0,
            _d1: d1,
            _d2: d2,
            _d3: d3,
            _d4: d4,
            _d5: d5,
            _d6: d6,
            _d7: d7,
            _nss: nss,
            _dqs: dqs,
            dma,
            _marker: PhantomData,
            config,
            width,
        }
    }

    // Function to configure the peripheral for the requested command
    fn configure_command(&mut self, command: &TransferConfig, data_len: Option<usize>) -> Result<(), OspiError> {
        // Check that transaction doesn't use more than hardware initialized pins
        if <enums::OspiWidth as Into<u8>>::into(command.iwidth) > <enums::OspiWidth as Into<u8>>::into(self.width)
            || <enums::OspiWidth as Into<u8>>::into(command.adwidth) > <enums::OspiWidth as Into<u8>>::into(self.width)
            || <enums::OspiWidth as Into<u8>>::into(command.abwidth) > <enums::OspiWidth as Into<u8>>::into(self.width)
            || <enums::OspiWidth as Into<u8>>::into(command.dwidth) > <enums::OspiWidth as Into<u8>>::into(self.width)
        {
            return Err(OspiError::InvalidCommand);
        }

        T::REGS.cr().modify(|w| {
            w.set_fmode(vals::FunctionalMode::IndirectWrite);
        });

        // Configure alternate bytes
        if let Some(ab) = command.alternate_bytes {
            T::REGS.abr().write(|v| v.set_alternate(ab));
        }

        // Configure dummy cycles
        T::REGS.tcr().modify(|w| {
            w.set_dcyc(command.dummy.into());
        });

        // Configure data
        if let Some(data_length) = data_len {
            T::REGS.dlr().write(|v| {
                v.set_dl((data_length - 1) as u32);
            });
        } else {
            T::REGS.dlr().write(|v| {
                v.set_dl((0) as u32);
            });
        }

        // Configure instruction/address/alternate bytes/data/communication modes
        T::REGS.ccr().modify(|w| {
            w.set_imode(PhaseMode::from_bits(command.iwidth.into()));
            w.set_idtr(command.idtr);
            w.set_isize(SizeInBits::from_bits(command.isize.into()));

            w.set_admode(PhaseMode::from_bits(command.adwidth.into()));
            w.set_addtr(command.addtr);
            w.set_adsize(SizeInBits::from_bits(command.adsize.into()));

            w.set_abmode(PhaseMode::from_bits(command.abwidth.into()));
            w.set_abdtr(command.abdtr);
            w.set_absize(SizeInBits::from_bits(command.absize.into()));

            w.set_dmode(PhaseMode::from_bits(command.dwidth.into()));
            w.set_ddtr(command.ddtr);

            w.set_dqse(command.dqse);
            w.set_sioo(command.sioo);
        });

        // Set information required to initiate transaction
        if let Some(instruction) = command.instruction {
            if let Some(address) = command.address {
                T::REGS.ir().write(|v| {
                    v.set_instruction(instruction);
                });

                T::REGS.ar().write(|v| {
                    v.set_address(address);
                });
            } else {
                // Double check requirements for delay hold and sample shifting
                // if let None = command.data_len {
                //     if self.config.delay_hold_quarter_cycle && command.idtr {
                //         T::REGS.ccr().modify(|w| {
                //             w.set_ddtr(true);
                //         });
                //     }
                // }

                T::REGS.ir().write(|v| {
                    v.set_instruction(instruction);
                });
            }
        } else {
            if let Some(address) = command.address {
                T::REGS.ar().write(|v| {
                    v.set_address(address);
                });
            } else {
                // The only single phase transaction supported is instruction only
                return Err(OspiError::InvalidCommand);
            }
        }

        // The following errors set the TEF flag in OCTOSPI_SR register:
        // - in indirect or automatic status-polling mode, when a wrong address has been programmed
        //   in OCTOSPI_AR (according to the device size defined by DEVSIZE[4:0])
        // - in indirect mode, if the address plus the data length exceed the device size: TEF is
        // set as soon as the access is triggered.
        if T::REGS.sr().read().tef() {
            // Clear the TEF register to make it ready for the next transfer.
            T::REGS.fcr().write(|w| w.set_ctef(true));

            return Err(OspiError::InvalidCommand);
        }

        Ok(())
    }

    /// Function used to control or configure the target device without data transfer
    pub fn blocking_command(&mut self, command: &TransferConfig) -> Result<(), OspiError> {
        // Wait for peripheral to be free
        while T::REGS.sr().read().busy() {}

        // Need additional validation that command configuration doesn't have data set
        self.configure_command(command, None)?;

        // Transaction initiated by setting final configuration, i.e the instruction register
        while !T::REGS.sr().read().tcf() {}
        T::REGS.fcr().write(|w| {
            w.set_ctcf(true);
        });

        Ok(())
    }

    /// Blocking read with byte by byte data transfer
    pub fn blocking_read<W: Word>(&mut self, buf: &mut [W], transaction: TransferConfig) -> Result<(), OspiError> {
        if buf.is_empty() {
            return Err(OspiError::EmptyBuffer);
        }

        // Wait for peripheral to be free
        while T::REGS.sr().read().busy() {}

        // Ensure DMA is not enabled for this transaction
        T::REGS.cr().modify(|w| {
            w.set_dmaen(false);
        });

        let transfer_size_bytes = buf.len() * W::size().bytes();
        self.configure_command(&transaction, Some(transfer_size_bytes))?;

        let current_address = T::REGS.ar().read().address();
        let current_instruction = T::REGS.ir().read().instruction();

        // For a indirect read transaction, the transaction begins when the instruction/address is set
        T::REGS.cr().modify(|v| v.set_fmode(vals::FunctionalMode::IndirectRead));
        if T::REGS.ccr().read().admode() == vals::PhaseMode::None {
            T::REGS.ir().write(|v| v.set_instruction(current_instruction));
        } else {
            T::REGS.ar().write(|v| v.set_address(current_address));
        }

        for idx in 0..buf.len() {
            while !T::REGS.sr().read().tcf() && !T::REGS.sr().read().ftf() {}
            buf[idx] = unsafe { (T::REGS.dr().as_ptr() as *mut W).read_volatile() };
        }

        while !T::REGS.sr().read().tcf() {}
        T::REGS.fcr().write(|v| v.set_ctcf(true));

        Ok(())
    }

    /// Blocking write with byte by byte data transfer
    pub fn blocking_write<W: Word>(&mut self, buf: &[W], transaction: TransferConfig) -> Result<(), OspiError> {
        if buf.is_empty() {
            return Err(OspiError::EmptyBuffer);
        }

        // Wait for peripheral to be free
        while T::REGS.sr().read().busy() {}

        T::REGS.cr().modify(|w| {
            w.set_dmaen(false);
        });

        let transfer_size_bytes = buf.len() * W::size().bytes();
        self.configure_command(&transaction, Some(transfer_size_bytes))?;

        T::REGS
            .cr()
            .modify(|v| v.set_fmode(vals::FunctionalMode::IndirectWrite));

        for idx in 0..buf.len() {
            while !T::REGS.sr().read().ftf() {}
            unsafe { (T::REGS.dr().as_ptr() as *mut W).write_volatile(buf[idx]) };
        }

        while !T::REGS.sr().read().tcf() {}
        T::REGS.fcr().write(|v| v.set_ctcf(true));

        Ok(())
    }

    /// Set new bus configuration
    pub fn set_config(&mut self, config: &Config) {
        // Wait for busy flag to clear
        while T::REGS.sr().read().busy() {}

        // Disable DMA channel while configuring the peripheral
        T::REGS.cr().modify(|w| {
            w.set_dmaen(false);
        });

        // Device configuration
        T::REGS.dcr1().modify(|w| {
            w.set_devsize(config.device_size.into());
            w.set_mtyp(vals::MemType::from_bits(config.memory_type.into()));
            w.set_csht(config.chip_select_high_time.into());
            w.set_dlybyp(config.delay_block_bypass);
            w.set_frck(false);
            w.set_ckmode(config.clock_mode);
        });

        T::REGS.dcr2().modify(|w| {
            w.set_wrapsize(config.wrap_size.into());
        });

        T::REGS.dcr3().modify(|w| {
            w.set_csbound(config.chip_select_boundary);
            #[cfg(octospi_v1)]
            {
                w.set_maxtran(config.max_transfer);
            }
        });

        T::REGS.dcr4().modify(|w| {
            w.set_refresh(config.refresh);
        });

        T::REGS.cr().modify(|w| {
            w.set_fthres(vals::Threshold::from_bits(config.fifo_threshold.into()));
        });

        // Wait for busy flag to clear
        while T::REGS.sr().read().busy() {}

        T::REGS.dcr2().modify(|w| {
            w.set_prescaler(config.clock_prescaler);
        });

        T::REGS.tcr().modify(|w| {
            w.set_sshift(match config.sample_shifting {
                true => vals::SampleShift::HalfCycle,
                false => vals::SampleShift::None,
            });
            w.set_dhqc(config.delay_hold_quarter_cycle);
        });

        // Enable peripheral
        T::REGS.cr().modify(|w| {
            w.set_en(true);
        });

        // Free running clock needs to be set after peripheral enable
        if config.free_running_clock {
            T::REGS.dcr1().modify(|w| {
                w.set_frck(config.free_running_clock);
            });
        }

        self.config = *config;
    }

    /// Get current configuration
    pub fn get_config(&self) -> Config {
        self.config
    }
}

impl<'d, T: Instance> Ospi<'d, T, Blocking> {
    /// Create new blocking OSPI driver for a single spi external chip
    #[cfg(not(octospim_v1))]
    pub fn new_blocking_singlespi(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        d1: Peri<'d, impl D1Pin<T>>,
        nss: Peri<'d, impl NSSPin<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::input(Pull::None)),
            None,
            None,
            None,
            None,
            None,
            None,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            None,
            config,
            OspiWidth::SING,
            false,
        )
    }

    /// Create new blocking OSPI driver for a single spi external chip
    #[cfg(octospim_v1)]
    pub fn new_blocking_singlespi<const IOL_PGROUP: u8, const CTRL_PGROUP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckSrc<T, CTRL_PGROUP>>,
        d0: Peri<'d, impl D0Src<T, IOL_PGROUP>>,
        d1: Peri<'d, impl D1Src<T, IOL_PGROUP>>,
        nss: Peri<'d, impl NSSSrc<T, CTRL_PGROUP>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::input(Pull::None)),
            None,
            None,
            None,
            None,
            None,
            None,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            None,
            config,
            OspiWidth::SING,
            false,
            IOL_PGROUP,
            None,
            CTRL_PGROUP,
        )
    }

    /// Create new blocking OSPI driver for a dualspi external chip
    #[cfg(not(octospim_v1))]
    pub fn new_blocking_dualspi(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        d1: Peri<'d, impl D1Pin<T>>,
        nss: Peri<'d, impl NSSPin<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            None,
            None,
            None,
            None,
            None,
            None,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            None,
            config,
            OspiWidth::DUAL,
            false,
        )
    }

    /// Create new blocking OSPI driver for a dualspi external chip
    #[cfg(octospim_v1)]
    pub fn new_blocking_dualspi<const IOL_PGROUP: u8, const CTRL_PGROUP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckSrc<T, CTRL_PGROUP>>,
        d0: Peri<'d, impl D0Src<T, IOL_PGROUP>>,
        d1: Peri<'d, impl D1Src<T, IOL_PGROUP>>,
        nss: Peri<'d, impl NSSSrc<T, CTRL_PGROUP>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            None,
            None,
            None,
            None,
            None,
            None,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            None,
            config,
            OspiWidth::DUAL,
            false,
            IOL_PGROUP,
            None,
            CTRL_PGROUP,
        )
    }

    /// Create new blocking OSPI driver for a quadspi external chip
    #[cfg(not(octospim_v1))]
    pub fn new_blocking_quadspi(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        d1: Peri<'d, impl D1Pin<T>>,
        d2: Peri<'d, impl D2Pin<T>>,
        d3: Peri<'d, impl D3Pin<T>>,
        nss: Peri<'d, impl NSSPin<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            None,
            None,
            None,
            None,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            None,
            config,
            OspiWidth::QUAD,
            false,
        )
    }

    /// Create new blocking OSPI driver for a quadspi external chip
    #[cfg(octospim_v1)]
    pub fn new_blocking_quadspi<const IOL_PGROUP: u8, const CTRL_PGROUP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckSrc<T, CTRL_PGROUP>>,
        d0: Peri<'d, impl D0Src<T, IOL_PGROUP>>,
        d1: Peri<'d, impl D1Src<T, IOL_PGROUP>>,
        d2: Peri<'d, impl D2Src<T, IOL_PGROUP>>,
        d3: Peri<'d, impl D3Src<T, IOL_PGROUP>>,
        nss: Peri<'d, impl NSSSrc<T, CTRL_PGROUP>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            None,
            None,
            None,
            None,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            None,
            config,
            OspiWidth::QUAD,
            false,
            IOL_PGROUP,
            None,
            CTRL_PGROUP,
        )
    }

    /// Create new blocking OSPI driver for two quadspi external chips
    #[cfg(not(octospim_v1))]
    pub fn new_blocking_dualquadspi(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        d0_1: Peri<'d, impl D0Pin<T>>,
        d1_1: Peri<'d, impl D1Pin<T>>,
        d2_1: Peri<'d, impl D2Pin<T>>,
        d3_1: Peri<'d, impl D3Pin<T>>,
        d0_2: Peri<'d, impl D4Pin<T>>,
        d1_2: Peri<'d, impl D5Pin<T>>,
        d2_2: Peri<'d, impl D6Pin<T>>,
        d3_2: Peri<'d, impl D7Pin<T>>,
        nss: Peri<'d, impl NSSPin<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0_1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1_1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2_1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3_1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d0_2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1_2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2_2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3_2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            None,
            config,
            OspiWidth::QUAD,
            true,
        )
    }

    /// Create new blocking OSPI driver for two quadspi external chips
    #[cfg(octospim_v1)]
    pub fn new_blocking_dualquadspi<const IOLSRC1: u8, const IOLSRC2: u8, const CTRL_PGROUP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckSrc<T, CTRL_PGROUP>>,
        d0_1: Peri<'d, impl D0Src<T, IOLSRC1>>,
        d1_1: Peri<'d, impl D1Src<T, IOLSRC1>>,
        d2_1: Peri<'d, impl D2Src<T, IOLSRC1>>,
        d3_1: Peri<'d, impl D3Src<T, IOLSRC1>>,
        d0_2: Peri<'d, impl D4Src<T, IOLSRC2>>,
        d1_2: Peri<'d, impl D5Src<T, IOLSRC2>>,
        d2_2: Peri<'d, impl D6Src<T, IOLSRC2>>,
        d3_2: Peri<'d, impl D7Src<T, IOLSRC2>>,
        nss: Peri<'d, impl NSSSrc<T, CTRL_PGROUP>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0_1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1_1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2_1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3_1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d0_2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1_2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2_2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3_2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            None,
            config,
            OspiWidth::QUAD,
            true,
            IOLSRC1,
            Some(IOLSRC2),
            CTRL_PGROUP,
        )
    }

    /// Create new blocking OSPI driver for octospi external chips
    #[cfg(not(octospim_v1))]
    pub fn new_blocking_octospi(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        d1: Peri<'d, impl D1Pin<T>>,
        d2: Peri<'d, impl D2Pin<T>>,
        d3: Peri<'d, impl D3Pin<T>>,
        d4: Peri<'d, impl D4Pin<T>>,
        d5: Peri<'d, impl D5Pin<T>>,
        d6: Peri<'d, impl D6Pin<T>>,
        d7: Peri<'d, impl D7Pin<T>>,
        nss: Peri<'d, impl NSSPin<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d4, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d5, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d6, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d7, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            None,
            config,
            OspiWidth::OCTO,
            false,
        )
    }

    /// Create new blocking OSPI driver for octospi external chips
    #[cfg(octospim_v1)]
    pub fn new_blocking_octospi<const IOL_PGROUP: u8, const IOH_PGROUP: u8, const CTRL_PGROUP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckSrc<T, CTRL_PGROUP>>,
        d0: Peri<'d, impl D0Src<T, IOL_PGROUP>>,
        d1: Peri<'d, impl D1Src<T, IOL_PGROUP>>,
        d2: Peri<'d, impl D2Src<T, IOL_PGROUP>>,
        d3: Peri<'d, impl D3Src<T, IOL_PGROUP>>,
        d4: Peri<'d, impl D4Src<T, IOH_PGROUP>>,
        d5: Peri<'d, impl D5Src<T, IOH_PGROUP>>,
        d6: Peri<'d, impl D6Src<T, IOH_PGROUP>>,
        d7: Peri<'d, impl D7Src<T, IOH_PGROUP>>,
        nss: Peri<'d, impl NSSSrc<T, CTRL_PGROUP>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d4, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d5, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d6, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d7, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            None,
            config,
            OspiWidth::OCTO,
            false,
            IOL_PGROUP,
            Some(IOH_PGROUP),
            CTRL_PGROUP,
        )
    }

    /// Create new blocking OSPI driver for octospi external chips with DQS support
    #[cfg(not(octospim_v1))]
    pub fn new_blocking_octospi_with_dqs(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        d1: Peri<'d, impl D1Pin<T>>,
        d2: Peri<'d, impl D2Pin<T>>,
        d3: Peri<'d, impl D3Pin<T>>,
        d4: Peri<'d, impl D4Pin<T>>,
        d5: Peri<'d, impl D5Pin<T>>,
        d6: Peri<'d, impl D6Pin<T>>,
        d7: Peri<'d, impl D7Pin<T>>,
        nss: Peri<'d, impl NSSPin<T>>,
        dqs: Peri<'d, impl DQSPin<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d4, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d5, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d6, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d7, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            new_pin!(dqs, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            None,
            config,
            OspiWidth::OCTO,
            false,
        )
    }

    /// Create new blocking OSPI driver for octospi external chips with DQS support
    #[cfg(octospim_v1)]
    pub fn new_blocking_octospi_with_dqs<const IOL_PGROUP: u8, const IOH_PGROUP: u8, const CTRL_PGROUP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckSrc<T, CTRL_PGROUP>>,
        d0: Peri<'d, impl D0Src<T, IOL_PGROUP>>,
        d1: Peri<'d, impl D1Src<T, IOL_PGROUP>>,
        d2: Peri<'d, impl D2Src<T, IOL_PGROUP>>,
        d3: Peri<'d, impl D3Src<T, IOL_PGROUP>>,
        d4: Peri<'d, impl D4Src<T, IOH_PGROUP>>,
        d5: Peri<'d, impl D5Src<T, IOH_PGROUP>>,
        d6: Peri<'d, impl D6Src<T, IOH_PGROUP>>,
        d7: Peri<'d, impl D7Src<T, IOH_PGROUP>>,
        nss: Peri<'d, impl NSSSrc<T, CTRL_PGROUP>>,
        dqs: Peri<'d, impl DQSSrc<T, CTRL_PGROUP>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d4, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d5, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d6, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d7, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            new_pin!(dqs, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            None,
            config,
            OspiWidth::OCTO,
            false,
            IOL_PGROUP,
            Some(IOH_PGROUP),
            CTRL_PGROUP,
        )
    }
}

impl<'d, T: Instance, M: PeriMode> Ospi<'d, T, M> {
    #[cfg(octospim_v1)]
    fn new_multiplexed_inner<T2: Instance, M2: PeriMode>(
        peri1: Peri<'d, T>,
        peri2: Peri<'d, T2>,
        sck: Option<Flex<'d>>,
        d0: Option<Flex<'d>>,
        d1: Option<Flex<'d>>,
        d2: Option<Flex<'d>>,
        d3: Option<Flex<'d>>,
        d4: Option<Flex<'d>>,
        d5: Option<Flex<'d>>,
        d6: Option<Flex<'d>>,
        d7: Option<Flex<'d>>,
        nss1: Option<Flex<'d>>,
        nss2: Option<Flex<'d>>,
        dma1: Option<ChannelAndRequest<'d>>,
        dma2: Option<ChannelAndRequest<'d>>,
        config1: Config,
        config2: Config,
        mux_config: MultiplexConfig,
        width1: OspiWidth,
        width2: OspiWidth,
        iol_pgroup: u8,
        ioh_pgroup: Option<u8>,
        clk_pgroup: u8,
        ncs1_pgroup: u8,
        ncs2_pgroup: u8,
    ) -> (Self, Ospi<'d, T2, M2>) {
        assert_ne!(
            T::OCTOSPI_IDX,
            T2::OCTOSPI_IDX,
            "multiplexed OSPI instances must be different peripherals"
        );
        assert!(
            Self::octospim_uses_p2(ncs1_pgroup) != Self::octospim_uses_p2(ncs2_pgroup),
            "multiplexed OSPI chip selects must use different OCTOSPIM control groups"
        );
        assert!(
            (!matches!(width1, OspiWidth::OCTO) && !matches!(width2, OspiWidth::OCTO)) || ioh_pgroup.is_some(),
            "multiplexed octospi must set an IOH physical group"
        );

        Self::enable_octospim_clock();
        rcc::enable_and_reset::<T>();
        rcc::enable_and_reset::<T2>();
        while T::REGS.sr().read().busy() {}
        while T2::REGS.sr().read().busy() {}

        Self::disable_octospis_for_octospim_config();

        T::OCTOSPIM_REGS.cr().modify(|w| {
            w.set_muxen(false);
            w.set_req2ack_time(0xff);
        });
        T::OCTOSPIM_REGS.p1cr().write(|_| {});
        T::OCTOSPIM_REGS.p2cr().write(|_| {});

        Self::configure_octospim_clk_group(clk_pgroup, false, Self::octospim_signal_src());
        Self::configure_octospim_data_group(iol_pgroup, Self::octospim_low_data_src());
        if let Some(ioh_pgroup) = ioh_pgroup {
            Self::configure_octospim_data_group(ioh_pgroup, Self::octospim_high_data_src());
        }

        // In multiplexed mode CLK/IO/DQS are arbitrated between OCTOSPIs, while
        // NCS is not. The secondary port still has to enable the shared CLK/IO
        // mapping, but those signals use the primary OSPI source. NCS is then
        // sourced independently from the second OSPI.
        Self::configure_octospim_clk_group(ncs2_pgroup, false, Self::octospim_signal_src());
        Self::configure_octospim_data_group(ncs2_pgroup, Self::octospim_low_data_src());
        if matches!(width2, OspiWidth::OCTO) {
            Self::configure_octospim_data_group(ncs2_pgroup | 0b01, Self::octospim_high_data_src());
        }
        Self::configure_octospim_ncs_group(ncs1_pgroup, T::OCTOSPI_IDX == 2);
        Self::configure_octospim_ncs_group(ncs2_pgroup, T2::OCTOSPI_IDX == 2);

        T::OCTOSPIM_REGS.cr().modify(|w| {
            w.set_req2ack_time(mux_config.req2ack_time);
            w.set_muxen(true);
        });

        Self::configure_ospi_registers(config1, false);
        Ospi::<T2, M2>::configure_ospi_registers(config2, false);

        Self::enable_ospi(config1);
        Ospi::<T2, M2>::enable_ospi(config2);

        let ospi1 = Self {
            _peri: peri1,
            _sck: sck,
            _d0: d0,
            _d1: d1,
            _d2: d2,
            _d3: d3,
            _d4: d4,
            _d5: d5,
            _d6: d6,
            _d7: d7,
            _nss: nss1,
            _dqs: None,
            dma: dma1,
            _marker: PhantomData,
            config: config1,
            width: width1,
        };

        let ospi2 = Ospi {
            _peri: peri2,
            _sck: None,
            _d0: None,
            _d1: None,
            _d2: None,
            _d3: None,
            _d4: None,
            _d5: None,
            _d6: None,
            _d7: None,
            _nss: nss2,
            _dqs: None,
            dma: dma2,
            _marker: PhantomData,
            config: config2,
            width: width2,
        };

        (ospi1, ospi2)
    }
}

#[cfg(octospim_v1)]
/// Create a builder for two multiplexed OSPI instances.
pub fn new_multiplexed<'d, T: Instance, T2: Instance>(
    peri1: Peri<'d, T>,
    peri2: Peri<'d, T2>,
) -> MultiplexedOspiBuilder<'d, T, T2> {
    MultiplexedOspiBuilder::new(peri1, peri2)
}

#[cfg(octospim_v1)]
/// Builder for two multiplexed OSPI instances.
pub struct MultiplexedOspiBuilder<'d, T: Instance, T2: Instance, M1: PeriMode = Blocking, M2: PeriMode = Blocking> {
    peri1: Peri<'d, T>,
    peri2: Peri<'d, T2>,
    sck: Option<Flex<'d>>,
    d0: Option<Flex<'d>>,
    d1: Option<Flex<'d>>,
    d2: Option<Flex<'d>>,
    d3: Option<Flex<'d>>,
    d4: Option<Flex<'d>>,
    d5: Option<Flex<'d>>,
    d6: Option<Flex<'d>>,
    d7: Option<Flex<'d>>,
    nss1: Option<Flex<'d>>,
    nss2: Option<Flex<'d>>,
    dma1: Option<ChannelAndRequest<'d>>,
    dma2: Option<ChannelAndRequest<'d>>,
    config1: Config,
    config2: Config,
    mux_config: MultiplexConfig,
    width1: OspiWidth,
    width2: OspiWidth,
    widths_set: bool,
    iol_pgroup: u8,
    ioh_pgroup: Option<u8>,
    clk_pgroup: u8,
    ncs1_pgroup: u8,
    ncs2_pgroup: u8,
    _marker: PhantomData<(M1, M2)>,
}

#[cfg(octospim_v1)]
impl<'d, T: Instance, T2: Instance> MultiplexedOspiBuilder<'d, T, T2> {
    /// Create a new multiplexed OSPI builder.
    pub fn new(peri1: Peri<'d, T>, peri2: Peri<'d, T2>) -> Self {
        Self {
            peri1,
            peri2,
            sck: None,
            d0: None,
            d1: None,
            d2: None,
            d3: None,
            d4: None,
            d5: None,
            d6: None,
            d7: None,
            nss1: None,
            nss2: None,
            dma1: None,
            dma2: None,
            config1: Config::default(),
            config2: Config::default(),
            mux_config: MultiplexConfig { req2ack_time: 1 },
            width1: OspiWidth::QUAD,
            width2: OspiWidth::QUAD,
            widths_set: false,
            iol_pgroup: OCTOSPIM_P1_LOW,
            ioh_pgroup: None,
            clk_pgroup: OCTOSPIM_P1_CTRL,
            ncs1_pgroup: OCTOSPIM_P1_CTRL,
            ncs2_pgroup: OCTOSPIM_P2_CTRL,
            _marker: PhantomData,
        }
    }
}

#[cfg(octospim_v1)]
impl<'d, T: Instance, T2: Instance, M1: PeriMode, M2: PeriMode> MultiplexedOspiBuilder<'d, T, T2, M1, M2> {
    fn remap<N1: PeriMode, N2: PeriMode>(
        self,
        dma1: Option<ChannelAndRequest<'d>>,
        dma2: Option<ChannelAndRequest<'d>>,
    ) -> MultiplexedOspiBuilder<'d, T, T2, N1, N2> {
        MultiplexedOspiBuilder {
            peri1: self.peri1,
            peri2: self.peri2,
            sck: self.sck,
            d0: self.d0,
            d1: self.d1,
            d2: self.d2,
            d3: self.d3,
            d4: self.d4,
            d5: self.d5,
            d6: self.d6,
            d7: self.d7,
            nss1: self.nss1,
            nss2: self.nss2,
            dma1,
            dma2,
            config1: self.config1,
            config2: self.config2,
            mux_config: self.mux_config,
            width1: self.width1,
            width2: self.width2,
            widths_set: self.widths_set,
            iol_pgroup: self.iol_pgroup,
            ioh_pgroup: self.ioh_pgroup,
            clk_pgroup: self.clk_pgroup,
            ncs1_pgroup: self.ncs1_pgroup,
            ncs2_pgroup: self.ncs2_pgroup,
            _marker: PhantomData,
        }
    }

    /// Set the first OSPI instance configuration.
    pub fn config1(mut self, config: Config) -> Self {
        self.config1 = config;
        self
    }

    /// Set the second OSPI instance configuration.
    pub fn config2(mut self, config: Config) -> Self {
        self.config2 = config;
        self
    }

    /// Set the OCTOSPIM multiplexing configuration.
    pub fn mux_config(mut self, config: MultiplexConfig) -> Self {
        self.mux_config = config;
        self
    }

    /// Set the maximum logical width accepted by each returned OSPI handle.
    pub fn widths(mut self, width1: OspiWidth, width2: OspiWidth) -> Self {
        self.width1 = width1;
        self.width2 = width2;
        self.widths_set = true;
        self
    }

    /// Configure a shared QuadSPI bus.
    pub fn quad_bus<const IOL_PGROUP: u8, const CLK_PGROUP: u8, const NCS1_PGROUP: u8, const NCS2_PGROUP: u8>(
        mut self,
        sck: Peri<'d, impl SckSrc<T, CLK_PGROUP>>,
        d0: Peri<'d, impl D0Src<T, IOL_PGROUP>>,
        d1: Peri<'d, impl D1Src<T, IOL_PGROUP>>,
        d2: Peri<'d, impl D2Src<T, IOL_PGROUP>>,
        d3: Peri<'d, impl D3Src<T, IOL_PGROUP>>,
        nss1: Peri<'d, impl NSSSrc<T, NCS1_PGROUP>>,
        nss2: Peri<'d, impl NSSSrc<T2, NCS2_PGROUP>>,
    ) -> Self {
        self.sck = new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh));
        self.d0 = new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh));
        self.d1 = new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh));
        self.d2 = new_pin!(d2, AfType::output(OutputType::PushPull, Speed::VeryHigh));
        self.d3 = new_pin!(d3, AfType::output(OutputType::PushPull, Speed::VeryHigh));
        self.d4 = None;
        self.d5 = None;
        self.d6 = None;
        self.d7 = None;
        self.nss1 = new_pin!(
            nss1,
            AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
        );
        self.nss2 = new_pin!(
            nss2,
            AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
        );
        if !self.widths_set {
            self.width1 = OspiWidth::QUAD;
            self.width2 = OspiWidth::QUAD;
        }
        self.iol_pgroup = IOL_PGROUP;
        self.ioh_pgroup = None;
        self.clk_pgroup = CLK_PGROUP;
        self.ncs1_pgroup = NCS1_PGROUP;
        self.ncs2_pgroup = NCS2_PGROUP;
        self
    }

    /// Configure a shared OctoSPI bus.
    pub fn octo_bus<
        const IOL_PGROUP: u8,
        const IOH_PGROUP: u8,
        const CLK_PGROUP: u8,
        const NCS1_PGROUP: u8,
        const NCS2_PGROUP: u8,
    >(
        mut self,
        sck: Peri<'d, impl SckSrc<T, CLK_PGROUP>>,
        d0: Peri<'d, impl D0Src<T, IOL_PGROUP>>,
        d1: Peri<'d, impl D1Src<T, IOL_PGROUP>>,
        d2: Peri<'d, impl D2Src<T, IOL_PGROUP>>,
        d3: Peri<'d, impl D3Src<T, IOL_PGROUP>>,
        d4: Peri<'d, impl D4Src<T, IOH_PGROUP>>,
        d5: Peri<'d, impl D5Src<T, IOH_PGROUP>>,
        d6: Peri<'d, impl D6Src<T, IOH_PGROUP>>,
        d7: Peri<'d, impl D7Src<T, IOH_PGROUP>>,
        nss1: Peri<'d, impl NSSSrc<T, NCS1_PGROUP>>,
        nss2: Peri<'d, impl NSSSrc<T2, NCS2_PGROUP>>,
    ) -> Self {
        self.sck = new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh));
        self.d0 = new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh));
        self.d1 = new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh));
        self.d2 = new_pin!(d2, AfType::output(OutputType::PushPull, Speed::VeryHigh));
        self.d3 = new_pin!(d3, AfType::output(OutputType::PushPull, Speed::VeryHigh));
        self.d4 = new_pin!(d4, AfType::output(OutputType::PushPull, Speed::VeryHigh));
        self.d5 = new_pin!(d5, AfType::output(OutputType::PushPull, Speed::VeryHigh));
        self.d6 = new_pin!(d6, AfType::output(OutputType::PushPull, Speed::VeryHigh));
        self.d7 = new_pin!(d7, AfType::output(OutputType::PushPull, Speed::VeryHigh));
        self.nss1 = new_pin!(
            nss1,
            AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
        );
        self.nss2 = new_pin!(
            nss2,
            AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
        );
        if !self.widths_set {
            self.width1 = OspiWidth::OCTO;
            self.width2 = OspiWidth::OCTO;
        }
        self.iol_pgroup = IOL_PGROUP;
        self.ioh_pgroup = Some(IOH_PGROUP);
        self.clk_pgroup = CLK_PGROUP;
        self.ncs1_pgroup = NCS1_PGROUP;
        self.ncs2_pgroup = NCS2_PGROUP;
        self
    }

    /// Make the first OSPI instance async.
    pub fn peri1_async<D: OctoDma<T>>(
        mut self,
        dma: Peri<'d, D>,
        irq: impl crate::interrupt::typelevel::Binding<D::Interrupt, crate::dma::InterruptHandler<D>> + 'd,
    ) -> MultiplexedOspiBuilder<'d, T, T2, Async, M2> {
        let dma2 = self.dma2.take();
        self.remap(new_dma!(dma, irq), dma2)
    }

    /// Make the second OSPI instance async.
    pub fn peri2_async<D: OctoDma<T2>>(
        mut self,
        dma: Peri<'d, D>,
        irq: impl crate::interrupt::typelevel::Binding<D::Interrupt, crate::dma::InterruptHandler<D>> + 'd,
    ) -> MultiplexedOspiBuilder<'d, T, T2, M1, Async> {
        let dma1 = self.dma1.take();
        self.remap(dma1, new_dma!(dma, irq))
    }

    /// Build the two multiplexed OSPI instances.
    pub fn build(self) -> (Ospi<'d, T, M1>, Ospi<'d, T2, M2>) {
        assert!(self.sck.is_some(), "multiplexed OSPI requires an SCK pin");
        assert!(self.d0.is_some(), "multiplexed OSPI requires IO0");
        assert!(self.d1.is_some(), "multiplexed OSPI requires IO1");
        assert!(self.d2.is_some(), "multiplexed OSPI requires IO2");
        assert!(self.d3.is_some(), "multiplexed OSPI requires IO3");
        assert!(self.nss1.is_some(), "multiplexed OSPI requires the first NCS pin");
        assert!(self.nss2.is_some(), "multiplexed OSPI requires the second NCS pin");

        Ospi::<T, M1>::new_multiplexed_inner::<T2, M2>(
            self.peri1,
            self.peri2,
            self.sck,
            self.d0,
            self.d1,
            self.d2,
            self.d3,
            self.d4,
            self.d5,
            self.d6,
            self.d7,
            self.nss1,
            self.nss2,
            self.dma1,
            self.dma2,
            self.config1,
            self.config2,
            self.mux_config,
            self.width1,
            self.width2,
            self.iol_pgroup,
            self.ioh_pgroup,
            self.clk_pgroup,
            self.ncs1_pgroup,
            self.ncs2_pgroup,
        )
    }
}

impl<'d, T: Instance> Ospi<'d, T, Async> {
    /// Create new blocking OSPI driver for a single spi external chip
    #[cfg(not(octospim_v1))]
    pub fn new_singlespi<D: OctoDma<T>>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        d1: Peri<'d, impl D1Pin<T>>,
        nss: Peri<'d, impl NSSPin<T>>,
        dma: Peri<'d, D>,
        _irq: impl crate::interrupt::typelevel::Binding<D::Interrupt, crate::dma::InterruptHandler<D>> + 'd,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::input(Pull::None)),
            None,
            None,
            None,
            None,
            None,
            None,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            new_dma!(dma, _irq),
            config,
            OspiWidth::SING,
            false,
        )
    }

    /// Create new blocking OSPI driver for a single spi external chip
    #[cfg(octospim_v1)]
    pub fn new_singlespi<const IOL_PGROUP: u8, const CTRL_PGROUP: u8, D: OctoDma<T>>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckSrc<T, CTRL_PGROUP>>,
        d0: Peri<'d, impl D0Src<T, IOL_PGROUP>>,
        d1: Peri<'d, impl D1Src<T, IOL_PGROUP>>,
        nss: Peri<'d, impl NSSSrc<T, CTRL_PGROUP>>,
        dma: Peri<'d, D>,
        _irq: impl crate::interrupt::typelevel::Binding<D::Interrupt, crate::dma::InterruptHandler<D>> + 'd,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::input(Pull::None)),
            None,
            None,
            None,
            None,
            None,
            None,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            new_dma!(dma, _irq),
            config,
            OspiWidth::SING,
            false,
            IOL_PGROUP,
            None,
            CTRL_PGROUP,
        )
    }

    /// Create new blocking OSPI driver for a dualspi external chip
    #[cfg(not(octospim_v1))]
    pub fn new_dualspi<D: OctoDma<T>>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        d1: Peri<'d, impl D1Pin<T>>,
        nss: Peri<'d, impl NSSPin<T>>,
        dma: Peri<'d, D>,
        _irq: impl crate::interrupt::typelevel::Binding<D::Interrupt, crate::dma::InterruptHandler<D>> + 'd,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            None,
            None,
            None,
            None,
            None,
            None,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            new_dma!(dma, _irq),
            config,
            OspiWidth::DUAL,
            false,
        )
    }

    /// Create new blocking OSPI driver for a dualspi external chip
    #[cfg(octospim_v1)]
    pub fn new_dualspi<const IOL_PGROUP: u8, const CTRL_PGROUP: u8, D: OctoDma<T>>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckSrc<T, CTRL_PGROUP>>,
        d0: Peri<'d, impl D0Src<T, IOL_PGROUP>>,
        d1: Peri<'d, impl D1Src<T, IOL_PGROUP>>,
        nss: Peri<'d, impl NSSSrc<T, CTRL_PGROUP>>,
        dma: Peri<'d, D>,
        _irq: impl crate::interrupt::typelevel::Binding<D::Interrupt, crate::dma::InterruptHandler<D>> + 'd,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            None,
            None,
            None,
            None,
            None,
            None,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            new_dma!(dma, _irq),
            config,
            OspiWidth::DUAL,
            false,
            IOL_PGROUP,
            None,
            CTRL_PGROUP,
        )
    }

    /// Create new blocking OSPI driver for a quadspi external chip
    #[cfg(not(octospim_v1))]
    pub fn new_quadspi<D: OctoDma<T>>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        d1: Peri<'d, impl D1Pin<T>>,
        d2: Peri<'d, impl D2Pin<T>>,
        d3: Peri<'d, impl D3Pin<T>>,
        nss: Peri<'d, impl NSSPin<T>>,
        dma: Peri<'d, D>,
        _irq: impl crate::interrupt::typelevel::Binding<D::Interrupt, crate::dma::InterruptHandler<D>> + 'd,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            None,
            None,
            None,
            None,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            new_dma!(dma, _irq),
            config,
            OspiWidth::QUAD,
            false,
        )
    }

    /// Create new blocking OSPI driver for a quadspi external chip
    #[cfg(octospim_v1)]
    pub fn new_quadspi<const IOL_PGROUP: u8, const CTRL_PGROUP: u8, D: OctoDma<T>>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckSrc<T, CTRL_PGROUP>>,
        d0: Peri<'d, impl D0Src<T, IOL_PGROUP>>,
        d1: Peri<'d, impl D1Src<T, IOL_PGROUP>>,
        d2: Peri<'d, impl D2Src<T, IOL_PGROUP>>,
        d3: Peri<'d, impl D3Src<T, IOL_PGROUP>>,
        nss: Peri<'d, impl NSSSrc<T, CTRL_PGROUP>>,
        dma: Peri<'d, D>,
        _irq: impl crate::interrupt::typelevel::Binding<D::Interrupt, crate::dma::InterruptHandler<D>> + 'd,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            None,
            None,
            None,
            None,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            new_dma!(dma, _irq),
            config,
            OspiWidth::QUAD,
            false,
            IOL_PGROUP,
            None,
            CTRL_PGROUP,
        )
    }

    /// Create new blocking OSPI driver for two quadspi external chips
    #[cfg(not(octospim_v1))]
    pub fn new_dualquadspi<D: OctoDma<T>>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        d0_1: Peri<'d, impl D0Pin<T>>,
        d1_1: Peri<'d, impl D1Pin<T>>,
        d2_1: Peri<'d, impl D2Pin<T>>,
        d3_1: Peri<'d, impl D3Pin<T>>,
        d0_2: Peri<'d, impl D4Pin<T>>,
        d1_2: Peri<'d, impl D5Pin<T>>,
        d2_2: Peri<'d, impl D6Pin<T>>,
        d3_2: Peri<'d, impl D7Pin<T>>,
        nss: Peri<'d, impl NSSPin<T>>,
        dma: Peri<'d, D>,
        _irq: impl crate::interrupt::typelevel::Binding<D::Interrupt, crate::dma::InterruptHandler<D>> + 'd,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0_1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1_1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2_1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3_1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d0_2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1_2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2_2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3_2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            new_dma!(dma, _irq),
            config,
            OspiWidth::QUAD,
            true,
        )
    }

    /// Create new blocking OSPI driver for two quadspi external chips
    #[cfg(octospim_v1)]
    pub fn new_dualquadspi<const IOLSRC1: u8, const CTRL_PGROUP: u8, const IOLSRC2: u8, D: OctoDma<T>>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckSrc<T, CTRL_PGROUP>>,
        d0_1: Peri<'d, impl D0Src<T, IOLSRC1>>,
        d1_1: Peri<'d, impl D1Src<T, IOLSRC1>>,
        d2_1: Peri<'d, impl D2Src<T, IOLSRC1>>,
        d3_1: Peri<'d, impl D3Src<T, IOLSRC1>>,
        d0_2: Peri<'d, impl D0Src<T, IOLSRC2>>,
        d1_2: Peri<'d, impl D1Src<T, IOLSRC2>>,
        d2_2: Peri<'d, impl D2Src<T, IOLSRC2>>,
        d3_2: Peri<'d, impl D3Src<T, IOLSRC2>>,
        nss: Peri<'d, impl NSSSrc<T, CTRL_PGROUP>>,
        dma: Peri<'d, D>,
        _irq: impl crate::interrupt::typelevel::Binding<D::Interrupt, crate::dma::InterruptHandler<D>> + 'd,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0_1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1_1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2_1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3_1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d0_2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1_2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2_2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3_2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            new_dma!(dma, _irq),
            config,
            OspiWidth::QUAD,
            true,
            IOLSRC1,
            Some(IOLSRC2),
            CTRL_PGROUP,
        )
    }

    /// Create new blocking OSPI driver for octospi external chips
    #[cfg(not(octospim_v1))]
    pub fn new_octospi<D: OctoDma<T>>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        d1: Peri<'d, impl D1Pin<T>>,
        d2: Peri<'d, impl D2Pin<T>>,
        d3: Peri<'d, impl D3Pin<T>>,
        d4: Peri<'d, impl D4Pin<T>>,
        d5: Peri<'d, impl D5Pin<T>>,
        d6: Peri<'d, impl D6Pin<T>>,
        d7: Peri<'d, impl D7Pin<T>>,
        nss: Peri<'d, impl NSSPin<T>>,
        dma: Peri<'d, D>,
        _irq: impl crate::interrupt::typelevel::Binding<D::Interrupt, crate::dma::InterruptHandler<D>> + 'd,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d4, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d5, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d6, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d7, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            new_dma!(dma, _irq),
            config,
            OspiWidth::OCTO,
            false,
        )
    }

    /// Create new blocking OSPI driver for octospi external chips
    #[cfg(octospim_v1)]
    pub fn new_octospi<const IOL_PGROUP: u8, const IOH_PGROUP: u8, const CTRL_PGROUP: u8, D: OctoDma<T>>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckSrc<T, CTRL_PGROUP>>,
        d0: Peri<'d, impl D0Src<T, IOL_PGROUP>>,
        d1: Peri<'d, impl D1Src<T, IOL_PGROUP>>,
        d2: Peri<'d, impl D2Src<T, IOL_PGROUP>>,
        d3: Peri<'d, impl D3Src<T, IOL_PGROUP>>,
        d4: Peri<'d, impl D4Src<T, IOH_PGROUP>>,
        d5: Peri<'d, impl D5Src<T, IOH_PGROUP>>,
        d6: Peri<'d, impl D6Src<T, IOH_PGROUP>>,
        d7: Peri<'d, impl D7Src<T, IOH_PGROUP>>,
        nss: Peri<'d, impl NSSSrc<T, CTRL_PGROUP>>,
        dma: Peri<'d, D>,
        _irq: impl crate::interrupt::typelevel::Binding<D::Interrupt, crate::dma::InterruptHandler<D>> + 'd,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d4, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d5, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d6, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d7, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            None,
            new_dma!(dma, _irq),
            config,
            OspiWidth::OCTO,
            false,
            IOL_PGROUP,
            Some(IOH_PGROUP),
            CTRL_PGROUP,
        )
    }

    /// Create new blocking OSPI driver for octospi external chips with DQS support
    #[cfg(not(octospim_v1))]
    pub fn new_octospi_with_dqs<D: OctoDma<T>>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        d1: Peri<'d, impl D1Pin<T>>,
        d2: Peri<'d, impl D2Pin<T>>,
        d3: Peri<'d, impl D3Pin<T>>,
        d4: Peri<'d, impl D4Pin<T>>,
        d5: Peri<'d, impl D5Pin<T>>,
        d6: Peri<'d, impl D6Pin<T>>,
        d7: Peri<'d, impl D7Pin<T>>,
        nss: Peri<'d, impl NSSPin<T>>,
        dqs: Peri<'d, impl DQSPin<T>>,
        dma: Peri<'d, D>,
        _irq: impl crate::interrupt::typelevel::Binding<D::Interrupt, crate::dma::InterruptHandler<D>> + 'd,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d4, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d5, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d6, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d7, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            new_pin!(dqs, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_dma!(dma, _irq),
            config,
            OspiWidth::OCTO,
            false,
        )
    }

    /// Create new blocking OSPI driver for octospi external chips with DQS support
    #[cfg(octospim_v1)]
    pub fn new_octospi_with_dqs<const IOL_PGROUP: u8, const IOH_PGROUP: u8, const CTRL_PGROUP: u8, D: OctoDma<T>>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckSrc<T, CTRL_PGROUP>>,
        d0: Peri<'d, impl D0Src<T, IOL_PGROUP>>,
        d1: Peri<'d, impl D1Src<T, IOL_PGROUP>>,
        d2: Peri<'d, impl D2Src<T, IOL_PGROUP>>,
        d3: Peri<'d, impl D3Src<T, IOL_PGROUP>>,
        d4: Peri<'d, impl D4Src<T, IOH_PGROUP>>,
        d5: Peri<'d, impl D5Src<T, IOH_PGROUP>>,
        d6: Peri<'d, impl D6Src<T, IOH_PGROUP>>,
        d7: Peri<'d, impl D7Src<T, IOH_PGROUP>>,
        nss: Peri<'d, impl NSSSrc<T, CTRL_PGROUP>>,
        dqs: Peri<'d, impl DQSSrc<T, CTRL_PGROUP>>,
        dma: Peri<'d, D>,
        _irq: impl crate::interrupt::typelevel::Binding<D::Interrupt, crate::dma::InterruptHandler<D>> + 'd,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            new_pin!(d0, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d1, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d2, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d3, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d4, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d5, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d6, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(d7, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_pin!(
                nss,
                AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, Pull::Up)
            ),
            new_pin!(dqs, AfType::output(OutputType::PushPull, Speed::VeryHigh)),
            new_dma!(dma, _irq),
            config,
            OspiWidth::OCTO,
            false,
            IOL_PGROUP,
            Some(IOH_PGROUP),
            CTRL_PGROUP,
        )
    }

    /// Blocking read with DMA transfer
    pub fn blocking_read_dma<W: Word>(&mut self, buf: &mut [W], transaction: TransferConfig) -> Result<(), OspiError> {
        if buf.is_empty() {
            return Err(OspiError::EmptyBuffer);
        }

        // Wait for peripheral to be free
        while T::REGS.sr().read().busy() {}

        let transfer_size_bytes = buf.len() * W::size().bytes();
        self.configure_command(&transaction, Some(transfer_size_bytes))?;

        let current_address = T::REGS.ar().read().address();
        let current_instruction = T::REGS.ir().read().instruction();

        // For a indirect read transaction, the transaction begins when the instruction/address is set
        T::REGS.cr().modify(|v| v.set_fmode(vals::FunctionalMode::IndirectRead));
        if T::REGS.ccr().read().admode() == vals::PhaseMode::None {
            T::REGS.ir().write(|v| v.set_instruction(current_instruction));
        } else {
            T::REGS.ar().write(|v| v.set_address(current_address));
        }

        for chunk in buf.chunks_mut(0xFFFF / W::size().bytes()) {
            let transfer = unsafe {
                self.dma
                    .as_mut()
                    .unwrap()
                    .read(T::REGS.dr().as_ptr() as *mut W, chunk, Default::default())
            };

            T::REGS.cr().modify(|w| w.set_dmaen(true));

            transfer.blocking_wait();
        }

        finish_dma(T::REGS);

        Ok(())
    }

    /// Blocking write with DMA transfer
    pub fn blocking_write_dma<W: Word>(&mut self, buf: &[W], transaction: TransferConfig) -> Result<(), OspiError> {
        if buf.is_empty() {
            return Err(OspiError::EmptyBuffer);
        }

        // Wait for peripheral to be free
        while T::REGS.sr().read().busy() {}

        let transfer_size_bytes = buf.len() * W::size().bytes();
        self.configure_command(&transaction, Some(transfer_size_bytes))?;
        T::REGS
            .cr()
            .modify(|v| v.set_fmode(vals::FunctionalMode::IndirectWrite));

        // TODO: implement this using a LinkedList DMA to offload the whole transfer off the CPU.
        for chunk in buf.chunks(0xFFFF / W::size().bytes()) {
            let transfer = unsafe {
                self.dma
                    .as_mut()
                    .unwrap()
                    .write(chunk, T::REGS.dr().as_ptr() as *mut W, Default::default())
            };

            T::REGS.cr().modify(|w| w.set_dmaen(true));

            transfer.blocking_wait();
        }

        finish_dma(T::REGS);

        Ok(())
    }

    /// Asynchronous read from external device
    pub async fn read<W: Word>(&mut self, buf: &mut [W], transaction: TransferConfig) -> Result<(), OspiError> {
        if buf.is_empty() {
            return Err(OspiError::EmptyBuffer);
        }

        // Wait for peripheral to be free
        while T::REGS.sr().read().busy() {}

        let transfer_size_bytes = buf.len() * W::size().bytes();
        self.configure_command(&transaction, Some(transfer_size_bytes))?;

        let current_address = T::REGS.ar().read().address();
        let current_instruction = T::REGS.ir().read().instruction();

        // For a indirect read transaction, the transaction begins when the instruction/address is set
        T::REGS.cr().modify(|v| v.set_fmode(vals::FunctionalMode::IndirectRead));
        if T::REGS.ccr().read().admode() == vals::PhaseMode::None {
            T::REGS.ir().write(|v| v.set_instruction(current_instruction));
        } else {
            T::REGS.ar().write(|v| v.set_address(current_address));
        }

        for chunk in buf.chunks_mut(0xFFFF / W::size().bytes()) {
            let transfer = unsafe {
                self.dma
                    .as_mut()
                    .unwrap()
                    .read(T::REGS.dr().as_ptr() as *mut W, chunk, Default::default())
            };

            T::REGS.cr().modify(|w| w.set_dmaen(true));

            transfer.await;
        }

        finish_dma(T::REGS);

        Ok(())
    }

    /// Asynchronous write to external device
    pub async fn write<W: Word>(&mut self, buf: &[W], transaction: TransferConfig) -> Result<(), OspiError> {
        if buf.is_empty() {
            return Err(OspiError::EmptyBuffer);
        }

        // Wait for peripheral to be free
        while T::REGS.sr().read().busy() {}

        let transfer_size_bytes = buf.len() * W::size().bytes();
        self.configure_command(&transaction, Some(transfer_size_bytes))?;
        T::REGS
            .cr()
            .modify(|v| v.set_fmode(vals::FunctionalMode::IndirectWrite));

        // TODO: implement this using a LinkedList DMA to offload the whole transfer off the CPU.
        for chunk in buf.chunks(0xFFFF / W::size().bytes()) {
            let transfer = unsafe {
                self.dma
                    .as_mut()
                    .unwrap()
                    .write(chunk, T::REGS.dr().as_ptr() as *mut W, Default::default())
            };

            T::REGS.cr().modify(|w| w.set_dmaen(true));

            transfer.await;
        }

        finish_dma(T::REGS);

        Ok(())
    }

    pub async fn autopoll(&mut self, transaction: TransferConfig, config: AutopollConfig) -> Result<(), OspiError> {
        // Wait for peripheral to be free
        while T::REGS.sr().read().busy() {}

        T::REGS.psmar().write(|w| w.set_match_(config.match_value));
        T::REGS.psmkr().write(|w| w.set_mask(config.match_mask));
        T::REGS.pir().write(|w| w.set_interval(config.interval));

        self.configure_command(&transaction, Some(1))?;

        // Clear status flags
        T::REGS.fcr().write(|w| {
            w.set_csmf(true);
            w.set_ctef(true);
        });

        // Enable interrupts and configure auto polling mode
        T::REGS.cr().modify(|w| {
            w.set_smie(true);
            w.set_teie(true);

            w.set_pmm(config.match_mode.into());
            w.set_apms(config.auto_stop);
        });

        let current_address = T::REGS.ar().read().address();
        let current_instruction = T::REGS.ir().read().instruction();

        T::REGS
            .cr()
            .modify(|v| v.set_fmode(vals::FunctionalMode::AutoStatusPolling));

        compiler_fence(Ordering::SeqCst);

        // Auto polling begins when the instruction/address is set
        if T::REGS.ccr().read().admode() == vals::PhaseMode::None {
            T::REGS.ir().write(|v| v.set_instruction(current_instruction));
        } else {
            T::REGS.ar().write(|v| v.set_address(current_address));
        }

        poll_fn(|cx| {
            T::state().waker.register(cx.waker());

            let bits = T::REGS.sr().read();

            if bits.tef() {
                T::REGS.cr().modify(|w| {
                    w.set_smie(false);
                    w.set_teie(false);
                    w.set_fmode(vals::FunctionalMode::IndirectRead);
                });

                Poll::Ready(Err(OspiError::TransferError))
            } else if bits.smf() {
                T::REGS.cr().modify(|w| {
                    w.set_smie(false);
                    w.set_teie(false);
                    w.set_fmode(vals::FunctionalMode::IndirectRead);
                });

                Poll::Ready(Ok(()))
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl<'d, T: Instance, M: PeriMode> Drop for Ospi<'d, T, M> {
    fn drop(&mut self) {
        rcc::disable::<T>();
    }
}

fn finish_dma(regs: Regs) {
    while !regs.sr().read().tcf() {}
    regs.fcr().write(|v| v.set_ctcf(true));

    regs.cr().modify(|w| {
        w.set_dmaen(false);
    });
}

#[cfg(octospim_v1)]
/// OctoSPI I/O manager instance trait.
pub(crate) trait SealedOctospimInstance {
    const OCTOSPIM_REGS: Octospim;
    const OCTOSPI_IDX: u8;
}

/// OctoSPI instance trait.
trait SealedInstance {
    const REGS: Regs;
    fn state() -> &'static State;
}

/// OSPI instance trait.
#[cfg(octospim_v1)]
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + RccPeripheral + SealedOctospimInstance {
    /// Interrupt for OSPI instance.
    type Interrupt: interrupt::typelevel::Interrupt;
}

/// OSPI instance trait.
#[cfg(not(octospim_v1))]
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + RccPeripheral {
    /// Interrupt for OSPI instance.
    type Interrupt: interrupt::typelevel::Interrupt;
}

#[cfg(octospim_v1)]
macro_rules! ospi_signal_src_trait {
    ($signal:ident) => {
        #[doc = concat!(stringify!($signal), " pin trait")]
        pub trait $signal<T: Instance, const CTRL_PGROUP: u8>: crate::gpio::Pin {
            #[cfg(not(afio))]
            #[doc = concat!("Get the AF number needed to use this pin as `", stringify!($signal), "`.")]
            fn af_num(&self) -> u8;

            #[cfg(afio)]
            #[doc = concat!("Configures AFIO_MAPR to use this pin as `", stringify!($signal), "`.")]
            fn afio_remap(&self);
        }
    };
}

#[cfg(octospim_v1)]
macro_rules! ospi_signal_src_trait_impl {
    (crate::ospi::$trait:ident<$src:tt>, $instance:ident, $pin:ident, $af:expr) => {
        #[cfg(afio)]
        impl crate::ospi::$trait<crate::peripherals::$instance, $src> for crate::peripherals::$pin {
            fn afio_remap(&self) {
                // nothing
            }
        }

        #[cfg(not(afio))]
        impl crate::ospi::$trait<crate::peripherals::$instance, $src> for crate::peripherals::$pin {
            fn af_num(&self) -> u8 {
                $af
            }
        }
    };
}

dma_trait!(OctoDma, Instance);

cfg_if::cfg_if! {
    if #[cfg(octospim_v1)] {
        // signal sources when using OCTOSPIM
        ospi_signal_src_trait!(SckSrc);
        ospi_signal_src_trait!(NckSrc);
        ospi_signal_src_trait!(DQSSrc);
        ospi_signal_src_trait!(NSSSrc);
        ospi_signal_src_trait!(D0Src);
        ospi_signal_src_trait!(D1Src);
        ospi_signal_src_trait!(D2Src);
        ospi_signal_src_trait!(D3Src);
        ospi_signal_src_trait!(D4Src);
        ospi_signal_src_trait!(D5Src);
        ospi_signal_src_trait!(D6Src);
        ospi_signal_src_trait!(D7Src);
    } else {
        // pins when NOT using OCTOSPIM
        pin_trait!(SckPin, Instance);
        pin_trait!(NckPin, Instance);
        pin_trait!(DQSPin, Instance);
        pin_trait!(NSSPin, Instance);
        pin_trait!(D0Pin, Instance);
        pin_trait!(D1Pin, Instance);
        pin_trait!(D2Pin, Instance);
        pin_trait!(D3Pin, Instance);
        pin_trait!(D4Pin, Instance);
        pin_trait!(D5Pin, Instance);
        pin_trait!(D6Pin, Instance);
        pin_trait!(D7Pin, Instance);
    }
}

// Hard-coded the octospi index, for OCTOSPIM
#[cfg(octospim_v1)]
impl SealedOctospimInstance for peripherals::OCTOSPI1 {
    const OCTOSPIM_REGS: Octospim = crate::pac::OCTOSPIM;
    const OCTOSPI_IDX: u8 = 1;
}

#[cfg(all(octospim_v1, peri_octospi2))]
impl SealedOctospimInstance for peripherals::OCTOSPI2 {
    const OCTOSPIM_REGS: Octospim = crate::pac::OCTOSPIM;
    const OCTOSPI_IDX: u8 = 2;
}

#[cfg(octospim_v1)]
foreach_peripheral!(
    (octospi, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            const REGS: Regs = crate::pac::$inst;

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = interrupt::typelevel::$inst;
        }
    };
);

#[cfg(not(octospim_v1))]
foreach_peripheral!(
    (octospi, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            const REGS: Regs = crate::pac::$inst;

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = interrupt::typelevel::$inst;
        }
    };
);

impl<'d, T: Instance, M: PeriMode> SetConfig for Ospi<'d, T, M> {
    type Config = Config;
    type ConfigError = ();
    fn set_config(&mut self, config: &Self::Config) -> Result<(), ()> {
        self.set_config(config);
        Ok(())
    }
}

impl<'d, T: Instance, M: PeriMode> GetConfig for Ospi<'d, T, M> {
    type Config = Config;
    fn get_config(&self) -> Self::Config {
        self.get_config()
    }
}

/// Word sizes usable for OSPI.
#[allow(private_bounds)]
pub trait Word: word::Word {}

macro_rules! impl_word {
    ($T:ty) => {
        impl Word for $T {}
    };
}

impl_word!(u8);
impl_word!(u16);
impl_word!(u32);

/// OSPI Interrupt handler
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let sr = T::REGS.sr().read();
        let cr = T::REGS.cr().read();

        if sr.tef() && cr.teie() {
            T::REGS.cr().modify(|w| w.set_teie(false));
        } else if sr.smf() && cr.smie() {
            T::REGS.cr().modify(|w| w.set_smie(false));
        } else {
            return;
        }

        compiler_fence(Ordering::SeqCst);
        T::state().waker.wake();
    }
}

struct State {
    #[allow(unused)]
    waker: AtomicWaker,
}

impl State {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
        }
    }
}
