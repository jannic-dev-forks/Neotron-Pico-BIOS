//! # Neotron Pico BIOS
//!
//! This is the BIOS for the Neotron Pico. It:
//!
//! * initialises the hardware on the Neotron Pico,
//! * loads the Neotron OS (or any other compatible OS), and
//! * implements the Neotron Common BIOS API, to provide hardware abstraction
//!   services for the OS.
//!
//! The BIOS is started by having standard Cortex-M Interrupt Vector Table at
//! address `0x1000_0100`. This IVT is found and jumped to by the RP2040 boot
//! block(`0x1000_0000` to `0x1000_00FF`).

// -----------------------------------------------------------------------------
// Licence Statement
// -----------------------------------------------------------------------------
// Copyright (c) Jonathan 'theJPster' Pallant and the Neotron Developers, 2021
//
// This program is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
// details.
//
// You should have received a copy of the GNU General Public License along with
// this program.  If not, see <https://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#![no_std]
#![no_main]

// -----------------------------------------------------------------------------
// Sub-modules
// -----------------------------------------------------------------------------

pub mod api;
pub mod vga;

// -----------------------------------------------------------------------------
// Imports
// -----------------------------------------------------------------------------

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::rate::*;
use git_version::git_version;
use neotron_common_bios as common;
use panic_probe as _;
use pico::{
	self,
	hal::{
		self,
		pac::{self, interrupt},
	},
};

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

// None

// -----------------------------------------------------------------------------
// Static and Const Data
// -----------------------------------------------------------------------------

/// This is the standard RP2040 bootloader. It must be stored in the first 256
/// bytes of the external SPI Flash chip. It will map the external SPI flash
/// chip to address `0x1000_0000` and jump to an Interrupt Vector Table at
/// address `0x1000_0100` (i.e. immediately after the bootloader).
///
/// See `memory.x` for a definition of the `.boot2` section.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// BIOS version
const GIT_VERSION: &str = git_version!();

/// Create a new Text Console
static TEXT_CONSOLE: vga::TextConsole = vga::TextConsole::new();

// -----------------------------------------------------------------------------
// Functions
// -----------------------------------------------------------------------------

/// Prints to the screen
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        {
            use core::fmt::Write as _;
            write!(&crate::TEXT_CONSOLE, $($arg)*).unwrap();
        }
    };
}

/// Prints to the screen and puts a new-line on the end
#[macro_export]
macro_rules! println {
    () => (print!("\n"));
    ($($arg:tt)*) => {
        {
            use core::fmt::Write as _;
            writeln!(&crate::TEXT_CONSOLE, $($arg)*).unwrap();
        }
    };
}

/// This is the entry-point to the BIOS. It is called by cortex-m-rt once the
/// `.bss` and `.data` sections have been initialised.
#[entry]
fn main() -> ! {
	cortex_m::interrupt::disable();

	info!("Neotron BIOS {} starting...", GIT_VERSION);

	// Grab the singleton containing all the RP2040 peripherals
	let mut pac = pac::Peripherals::take().unwrap();
	// Grab the singleton containing all the generic Cortex-M peripherals
	let _core = pac::CorePeripherals::take().unwrap();

	// Reset the DMA engine. If we don't do this, starting from probe-run
	// (as opposed to a cold-start) is unreliable.
	pac.RESETS.reset.modify(|_r, w| w.dma().set_bit());
	cortex_m::asm::nop();
	pac.RESETS.reset.modify(|_r, w| w.dma().clear_bit());
	while pac.RESETS.reset_done.read().dma().bit_is_clear() {}

	// Needed by the clock setup
	let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

	// Run at 126 MHz SYS_PLL, 48 MHz, USB_PLL

	let xosc = hal::xosc::setup_xosc_blocking(pac.XOSC, pico::XOSC_CRYSTAL_FREQ.Hz())
		.map_err(|_x| false)
		.unwrap();

	// Configure watchdog tick generation to tick over every microsecond
	watchdog.enable_tick_generation((pico::XOSC_CRYSTAL_FREQ / 1_000_000) as u8);

	let mut clocks = hal::clocks::ClocksManager::new(pac.CLOCKS);

	let pll_sys = hal::pll::setup_pll_blocking(
		pac.PLL_SYS,
		xosc.operating_frequency().into(),
		hal::pll::PLLConfig {
			vco_freq: Megahertz(1512),
			refdiv: 1,
			post_div1: 6,
			post_div2: 2,
		},
		&mut clocks,
		&mut pac.RESETS,
	)
	.map_err(|_x| false)
	.unwrap();

	let pll_usb = hal::pll::setup_pll_blocking(
		pac.PLL_USB,
		xosc.operating_frequency().into(),
		hal::pll::common_configs::PLL_USB_48MHZ,
		&mut clocks,
		&mut pac.RESETS,
	)
	.map_err(|_x| false)
	.unwrap();

	clocks
		.init_default(&xosc, &pll_sys, &pll_usb)
		.map_err(|_x| false)
		.unwrap();

	info!("Clocks OK");

	// sio is the *Single-cycle Input/Output* peripheral. It has all our GPIO
	// pins, as well as some mailboxes and other useful things for inter-core
	// communications.
	let mut sio = hal::sio::Sio::new(pac.SIO);

	// Configure and grab all the RP2040 pins the Pico exposes.
	let pins = pico::Pins::new(
		pac.IO_BANK0,
		pac.PADS_BANK0,
		sio.gpio_bank0,
		&mut pac.RESETS,
	);

	// Disable power save mode to force SMPS into low-efficiency, low-noise mode.
	let mut b_power_save = pins.b_power_save.into_push_pull_output();
	b_power_save.set_high().unwrap();

	// Give H-Sync, V-Sync and 12 RGB colour pins to PIO0 to output video
	let _h_sync = pins.gpio0.into_mode::<hal::gpio::FunctionPio0>();
	let _v_sync = pins.gpio1.into_mode::<hal::gpio::FunctionPio0>();
	let _red0 = pins.gpio2.into_mode::<hal::gpio::FunctionPio0>();
	let _red1 = pins.gpio3.into_mode::<hal::gpio::FunctionPio0>();
	let _red2 = pins.gpio4.into_mode::<hal::gpio::FunctionPio0>();
	let _red3 = pins.gpio5.into_mode::<hal::gpio::FunctionPio0>();
	let _green0 = pins.gpio6.into_mode::<hal::gpio::FunctionPio0>();
	let _green1 = pins.gpio7.into_mode::<hal::gpio::FunctionPio0>();
	let _green2 = pins.gpio8.into_mode::<hal::gpio::FunctionPio0>();
	let _green3 = pins.gpio9.into_mode::<hal::gpio::FunctionPio0>();
	let _blue0 = pins.gpio10.into_mode::<hal::gpio::FunctionPio0>();
	let _blue1 = pins.gpio11.into_mode::<hal::gpio::FunctionPio0>();
	let _blue2 = pins.gpio12.into_mode::<hal::gpio::FunctionPio0>();
	let _blue3 = pins.gpio13.into_mode::<hal::gpio::FunctionPio0>();

	info!("Pins OK");

	vga::init(
		pac.PIO0,
		pac.DMA,
		&mut pac.RESETS,
		&mut pac.PPB,
		&mut sio.fifo,
		&mut pac.PSM,
	);

	TEXT_CONSOLE.set_text_buffer(unsafe { &mut vga::CHAR_ARRAY });

	info!("VGA intialised");

	println!(
		"{} booting...",
		&api::BIOS_VERSION[..api::BIOS_VERSION.len() - 1]
	);
	println!("Copyright (C) 2021 The Neotron Developers");
	println!("This program comes with ABSOLUTELY NO WARRANTY.");
	println!("This is free software, and you are welcome to redistribute it");
	println!("under certain conditions.");

	// We assume the OS can initialise its own memory, as we have no idea how
	// much it's using so we can't initialise the memory for it.

	// The first four bytes of OS flash must be the address of the start
	// function. If it looks OK, we call it.

	extern "C" {
		static _start_osram_sym: u32;
		static _end_osram_sym: u32;
		static _start_os_flash_sym: u32;
		static _end_os_flash_sym: u32;
	}

	// Note:(unsafe) - only taking address of external static
	let start_addr = unsafe { &_start_osram_sym as &u32 as *const u32 as usize };
	let end_addr = unsafe { &_end_osram_sym as &u32 as *const u32 as usize };
	let length = end_addr - start_addr;
	println!(
		"OSRAM 0x{:08x}..0x{:08x} = {} bytes",
		start_addr, end_addr, length
	);

	let start_os_flash_address = unsafe { &_start_os_flash_sym as *const u32 as usize };
	let os_entry_address = unsafe { _start_os_flash_sym } as usize;
	let end_os_flash_address = unsafe { &_end_os_flash_sym as *const u32 as usize };

	if (os_entry_address >= start_os_flash_address) && (os_entry_address <= end_os_flash_address) {
		// On this BIOS, the flash split between BIOS and OS is fixed. This value
		// must match the BIOS linker script and the OS linker script.
		let code: &common::OsStartFn = unsafe {
			&*(&_start_os_flash_sym as *const u32
				as *const for<'r> extern "C" fn(&'r neotron_common_bios::Api) -> !)
		};
		code(&api::API_CALLS);
	} else {
		println!(
			"No OS found at 0x{:08x}..0x{:08x} (0x{:08x} is bad)",
			start_os_flash_address, end_os_flash_address, os_entry_address
		);
		loop {
			cortex_m::asm::wfi();
		}
	}
}

/// Called when DMA raises IRQ0; i.e. when a DMA transfer to the pixel FIFO or
/// the timing FIFO has completed.
#[interrupt]
fn DMA_IRQ_0() {
	unsafe {
		vga::irq();
	}
}

// -----------------------------------------------------------------------------
// End of file
// -----------------------------------------------------------------------------
