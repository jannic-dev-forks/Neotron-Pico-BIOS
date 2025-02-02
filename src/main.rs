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
//! block (`0x1000_0000` to `0x1000_00FF`).

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

pub mod vga;

// -----------------------------------------------------------------------------
// Imports
// -----------------------------------------------------------------------------

use common::MemoryRegion;
use core::fmt::Write;
use cortex_m_rt::entry;
use defmt::info;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::rate::*;
use neotron_common_bios as common;
use panic_probe as _;
use rp_pico::{
	self,
	hal::{
		self,
		pac::{self, interrupt},
		Clock,
	},
};

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

// None

// -----------------------------------------------------------------------------
// Static and Const Data
// -----------------------------------------------------------------------------

/// The BIOS version string
static BIOS_VERSION: &str = concat!("Neotron Pico BIOS version ", env!("BIOS_VERSION"), "\0");

/// This is our Operating System. It must be compiled separately.
///
/// The RP2040 requires an OS linked at `0x1002_0000`, which is the OS binary
/// `flash1002`. Use `objdump` as per the README file to make a `flash1002.bin`.
#[link_section = ".flash_os"]
#[used]
pub static OS_IMAGE: [u8; include_bytes!("flash1002.bin").len()] = *include_bytes!("flash1002.bin");

/// The table of API calls we provide the OS
static API_CALLS: common::Api = common::Api {
	api_version_get,
	bios_version_get,
	serial_configure,
	serial_get_info,
	serial_write,
	serial_read,
	time_get,
	time_set,
	configuration_get,
	configuration_set,
	video_is_valid_mode,
	video_set_mode,
	video_get_mode,
	video_get_framebuffer,
	video_set_framebuffer,
	memory_get_region,
	video_mode_needs_vram,
	hid_get_event,
	hid_set_leds,
	video_wait_for_line,
	block_dev_get_info,
	block_write,
	block_read,
	block_verify,
};

extern "C" {
	static mut _flash_os_start: u32;
	static mut _flash_os_len: u32;
	static mut _ram_os_start: u32;
	static mut _ram_os_len: u32;
}

// -----------------------------------------------------------------------------
// Functions
// -----------------------------------------------------------------------------

/// This is the entry-point to the BIOS. It is called by cortex-m-rt once the
/// `.bss` and `.data` sections have been initialised.
#[entry]
fn main() -> ! {
	cortex_m::interrupt::disable();

	// BIOS_VERSION has a trailing `\0` as that is what the BIOS/OS API requires.
	info!("{} starting...", &BIOS_VERSION[0..BIOS_VERSION.len() - 1]);

	// Grab the singleton containing all the RP2040 peripherals
	let mut pp = pac::Peripherals::take().unwrap();
	// Grab the singleton containing all the generic Cortex-M peripherals
	let cp = pac::CorePeripherals::take().unwrap();

	// Reset the DMA engine. If we don't do this, starting from probe-run
	// (as opposed to a cold-start) is unreliable.
	reset_dma_engine(&mut pp);

	// Needed by the clock setup
	let mut watchdog = hal::watchdog::Watchdog::new(pp.WATCHDOG);

	// Run at 126 MHz SYS_PLL, 48 MHz, USB_PLL. This is important, we as clock
	// the PIO at ÷ 5, to give 25.2 MHz (which is close enough to the 25.175
	// MHz standard VGA pixel clock).

	// Step 1. Turn on the crystal.
	let xosc = hal::xosc::setup_xosc_blocking(pp.XOSC, rp_pico::XOSC_CRYSTAL_FREQ.Hz())
		.map_err(|_x| false)
		.unwrap();
	// Step 2. Configure watchdog tick generation to tick over every microsecond.
	watchdog.enable_tick_generation((rp_pico::XOSC_CRYSTAL_FREQ / 1_000_000) as u8);
	// Step 3. Create a clocks manager.
	let mut clocks = hal::clocks::ClocksManager::new(pp.CLOCKS);
	// Step 4. Set up the system PLL. We take Crystal Oscillator (=12 MHz),
	// ×126 (=1512 MHz), ÷6 (=252 MHz), ÷2 (=126 MHz)
	let pll_sys = hal::pll::setup_pll_blocking(
		pp.PLL_SYS,
		xosc.operating_frequency().into(),
		hal::pll::PLLConfig {
			vco_freq: Megahertz(1512),
			refdiv: 1,
			post_div1: 6,
			post_div2: 2,
		},
		&mut clocks,
		&mut pp.RESETS,
	)
	.map_err(|_x| false)
	.unwrap();
	// Step 5. Set up a 48 MHz PLL for the USB system.
	let pll_usb = hal::pll::setup_pll_blocking(
		pp.PLL_USB,
		xosc.operating_frequency().into(),
		hal::pll::common_configs::PLL_USB_48MHZ,
		&mut clocks,
		&mut pp.RESETS,
	)
	.map_err(|_x| false)
	.unwrap();
	// Step 6. Set the system to run from the PLLs we just configured.
	clocks
		.init_default(&xosc, &pll_sys, &pll_usb)
		.map_err(|_x| false)
		.unwrap();

	info!("Clocks OK");

	// sio is the *Single-cycle Input/Output* peripheral. It has all our GPIO
	// pins, as well as some mailboxes and other useful things for inter-core
	// communications.
	let mut sio = hal::sio::Sio::new(pp.SIO);

	// Configure and grab all the RP2040 pins the Pico exposes.
	let pins = rp_pico::Pins::new(pp.IO_BANK0, pp.PADS_BANK0, sio.gpio_bank0, &mut pp.RESETS);

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
		pp.PIO0,
		pp.DMA,
		&mut pp.RESETS,
		&mut pp.PPB,
		&mut sio.fifo,
		&mut pp.PSM,
	);

	// Say hello over VGA (with a bit of a pause)
	let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.system_clock.freq().integer());
	sign_on(&mut delay);

	// Now jump to the OS
	let code: &common::OsStartFn = unsafe { ::core::mem::transmute(&_flash_os_start) };
	code(&API_CALLS);
}

fn sign_on(delay: &mut cortex_m::delay::Delay) {
	static LICENCE_TEXT: &str = "\
        Copyright © Jonathan 'theJPster' Pallant and the Neotron Developers, 2022\n\
        \n\
        This program is free software: you can redistribute it and/or modify\n\
        it under the terms of the GNU General Public License as published by\n\
        the Free Software Foundation, either version 3 of the License, or\n\
        (at your option) any later version.\n\
        \n\
        This program is distributed in the hope that it will be useful,\n\
        but WITHOUT ANY WARRANTY; without even the implied warranty of\n\
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n\
        GNU General Public License for more details.\n\
        \n\
        You should have received a copy of the GNU General Public License\n\
        along with this program.  If not, see https://www.gnu.org/licenses/.\n";

	// Create a new temporary console for some boot-up messages
	let tc = vga::TextConsole::new();
	tc.set_text_buffer(unsafe { &mut vga::GLYPH_ATTR_ARRAY });

	// A crude way to clear the screen
	for _col in 0..vga::MAX_TEXT_ROWS {
		writeln!(&tc).unwrap();
	}

	tc.move_to(0, 0);

	writeln!(&tc, "{}", &BIOS_VERSION[0..BIOS_VERSION.len() - 1]).unwrap();
	write!(&tc, "{}", LICENCE_TEXT).unwrap();

	writeln!(&tc, "Loading Neotron OS...").unwrap();

	// Wait for a bit
	for n in [5, 4, 3, 2, 1].iter() {
		write!(&tc, "{}...", n).unwrap();
		delay.delay_ms(1000);
	}

	// A crude way to clear the screen
	for _col in 0..vga::MAX_TEXT_ROWS {
		writeln!(&tc).unwrap();
	}
	tc.move_to(0, 0);
}

/// Reset the DMA Peripheral.
fn reset_dma_engine(pp: &mut pac::Peripherals) {
	pp.RESETS.reset.modify(|_r, w| w.dma().set_bit());
	cortex_m::asm::nop();
	pp.RESETS.reset.modify(|_r, w| w.dma().clear_bit());
	while pp.RESETS.reset_done.read().dma().bit_is_clear() {}
}

/// Returns the version number of the BIOS API.
pub extern "C" fn api_version_get() -> common::Version {
	common::API_VERSION
}

/// Returns a pointer to a static string slice containing the BIOS Version.
///
/// This string contains the version number and build string of the BIOS.
/// For C compatibility this string is null-terminated and guaranteed to
/// only contain ASCII characters (bytes with a value 127 or lower). We
/// also pass the length (excluding the null) to make it easy to construct
/// a Rust string. It is unspecified as to whether the string is located
/// in Flash ROM or RAM (but it's likely to be Flash ROM).
pub extern "C" fn bios_version_get() -> common::ApiString<'static> {
	common::ApiString::new(BIOS_VERSION)
}

/// Get information about the Serial ports in the system.
///
/// Serial ports are ordered octet-oriented pipes. You can push octets
/// into them using a 'write' call, and pull bytes out of them using a
/// 'read' call. They have options which allow them to be configured at
/// different speeds, or with different transmission settings (parity
/// bits, stop bits, etc) - you set these with a call to
/// `SerialConfigure`. They may physically be a MIDI interface, an RS-232
/// port or a USB-Serial port. There is no sense of 'open' or 'close' -
/// that is an Operating System level design feature. These APIs just
/// reflect the raw hardware, in a similar manner to the registers exposed
/// by a memory-mapped UART peripheral.
pub extern "C" fn serial_get_info(_device: u8) -> common::Option<common::serial::DeviceInfo> {
	common::Option::None
}

/// Set the options for a given serial device. An error is returned if the
/// options are invalid for that serial device.
pub extern "C" fn serial_configure(
	_device: u8,
	_config: common::serial::Config,
) -> common::Result<()> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Write bytes to a serial port. There is no sense of 'opening' or
/// 'closing' the device - serial devices are always open. If the return
/// value is `Ok(n)`, the value `n` may be less than the size of the given
/// buffer. If so, that means not all of the data could be transmitted -
/// only the first `n` bytes were.
pub extern "C" fn serial_write(
	_device: u8,
	_data: common::ApiByteSlice,
	_timeout: common::Option<common::Timeout>,
) -> common::Result<usize> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Read bytes from a serial port. There is no sense of 'opening' or
/// 'closing' the device - serial devices are always open. If the return value
///  is `Ok(n)`, the value `n` may be less than the size of the given buffer.
///  If so, that means not all of the data could be received - only the
///  first `n` bytes were filled in.
pub extern "C" fn serial_read(
	_device: u8,
	_data: common::ApiBuffer,
	_timeout: common::Option<common::Timeout>,
) -> common::Result<usize> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Get the current wall time.
///
/// The Neotron BIOS does not understand time zones, leap-seconds or the
/// Gregorian calendar. It simply stores time as an incrementing number of
/// seconds since some epoch, and the number of milliseconds since that second
/// began. A day is assumed to be exactly 86,400 seconds long. This is a lot
/// like POSIX time, except we have a different epoch
/// - the Neotron epoch is 2000-01-01T00:00:00Z. It is highly recommend that you
/// store UTC in the BIOS and use the OS to handle time-zones.
///
/// If the BIOS does not have a battery-backed clock, or if that battery has
/// failed to keep time, the system starts up assuming it is the epoch.
pub extern "C" fn time_get() -> common::Time {
	// TODO: Read from the MCP7940N
	common::Time { secs: 0, nsecs: 0 }
}

/// Set the current wall time.
///
/// See `time_get` for a description of now the Neotron BIOS should handle
/// time.
///
/// You only need to call this whenever you get a new sense of the current
/// time (e.g. the user has updated the current time, or if you get a GPS
/// fix). The BIOS should push the time out to the battery-backed Real
/// Time Clock, if it has one.
pub extern "C" fn time_set(_time: common::Time) {
	// TODO: Update the MCP7940N RTC
}

/// Get the configuration data block.
///
/// Configuration data is, to the BIOS, just a block of bytes of a given
/// length. How it stores them is up to the BIOS - it could be EEPROM, or
/// battery-backed SRAM.
pub extern "C" fn configuration_get(_buffer: common::ApiBuffer) -> common::Result<usize> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Set the configuration data block.
///
/// See `configuration_get`.
pub extern "C" fn configuration_set(_buffer: common::ApiByteSlice) -> common::Result<()> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Does this Neotron BIOS support this video mode?
pub extern "C" fn video_is_valid_mode(mode: common::video::Mode) -> bool {
	mode == common::video::Mode::new(
		common::video::Timing::T640x480,
		common::video::Format::Text8x16,
	)
}

/// Switch to a new video mode.
///
/// The contents of the screen are undefined after a call to this function.
///
/// If the BIOS does not have enough reserved RAM (or dedicated VRAM) to
/// support this mode, the change will succeed but a subsequent call to
/// `video_get_framebuffer` will return `null`. You must then supply a
/// pointer to a block of size `Mode::frame_size_bytes()` to
/// `video_set_framebuffer` before any video will appear.
pub extern "C" fn video_set_mode(mode: common::video::Mode) -> common::Result<()> {
	if vga::set_video_mode(mode) {
		common::Result::Ok(())
	} else {
		common::Result::Err(common::Error::UnsupportedConfiguration(0))
	}
}

/// Returns the video mode the BIOS is currently in.
///
/// The OS should call this function immediately after start-up and note
/// the value - this is the `default` video mode which can always be
/// serviced without supplying extra RAM.
pub extern "C" fn video_get_mode() -> common::video::Mode {
	vga::get_video_mode()
}

/// Get the framebuffer address.
///
/// We can write through this address to the video framebuffer. The
/// meaning of the data we write, and the size of the region we are
/// allowed to write to, is a function of the current video mode (see
/// `video_get_mode`).
///
/// This function will return `null` if the BIOS isn't able to support the
/// current video mode from its memory reserves. If that happens, you will
/// need to use some OS RAM or Application RAM and provide that as a
/// framebuffer to `video_set_framebuffer`. The BIOS will always be able
/// to provide the 'basic' text buffer experience from reserves, so this
/// function will never return `null` on start-up.
pub extern "C" fn video_get_framebuffer() -> *mut u8 {
	unsafe { vga::GLYPH_ATTR_ARRAY.as_mut_ptr() as *mut u8 }
}

/// Set the framebuffer address.
///
/// Tell the BIOS where it should start fetching pixel or textual data from
/// (depending on the current video mode).
///
/// This value is forgotten after a video mode change and must be re-supplied.
///
/// # Safety
///
/// The pointer must point to enough video memory to handle the current video
/// mode, and any future video mode you set.
pub unsafe extern "C" fn video_set_framebuffer(_buffer: *const u8) -> common::Result<()> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Find out whether the given video mode needs more VRAM than we currently have.
///
/// The answer is no for any currently supported video mode (which is just the four text modes right now).
pub extern "C" fn video_mode_needs_vram(_mode: common::video::Mode) -> bool {
	false
}

/// Find out how large a given region of memory is.
///
/// The first region is the 'main application region' and is defined to always
/// start at address `0x2000_0000` on a standard Cortex-M system. This
/// application region stops just before the BIOS reserved memory, at the top of
/// the internal SRAM. The OS will have been linked to use the first 1 KiB of
/// this region.
///
/// Other regions may be located at other addresses (e.g. external DRAM or
/// PSRAM).
///
/// The OS will always load non-relocatable applications into the bottom of
/// Region 0. It can allocate OS specific structures from any other Region (if
/// any), or from the top of Region 0 (although this reduces the maximum
/// application space available). The OS will prefer lower numbered regions
/// (other than Region 0), so faster memory should be listed first.
///
/// If the region number given is invalid, the function returns `(null, 0)`.
pub extern "C" fn memory_get_region(region: u8) -> common::Result<common::MemoryRegion> {
	match region {
		0 => {
			// Application Region
			common::Result::Ok(MemoryRegion {
				start: unsafe { &mut _ram_os_start as *mut u32 } as *mut u8,
				length: unsafe { &mut _ram_os_len as *const u32 } as usize,
				kind: common::MemoryKind::Ram,
			})
		}
		_ => common::Result::Err(common::Error::InvalidDevice),
	}
}

/// Get the next available HID event, if any.
///
/// This function doesn't block. It will return `Ok(None)` if there is no event ready.
pub extern "C" fn hid_get_event() -> common::Result<common::Option<common::hid::HidEvent>> {
	// TODO: Support some HID events
	common::Result::Ok(common::Option::None)
}

/// Control the keyboard LEDs.
pub extern "C" fn hid_set_leds(_leds: common::hid::KeyboardLeds) -> common::Result<()> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Wait for the next occurence of the specified video scan-line.
///
/// In general we must assume that the video memory is read top-to-bottom
/// as the picture is being drawn on the monitor (e.g. via a VGA video
/// signal). If you modify video memory during this *drawing period*
/// there is a risk that the image on the monitor (however briefly) may
/// contain some parts from before the modification and some parts from
/// after. This can given rise to the *tearing effect* where it looks
/// like the screen has been torn (or ripped) across because there is a
/// discontinuity part-way through the image.
///
/// This function busy-waits until the video drawing has reached a
/// specified scan-line on the video frame.
///
/// There is no error code here. If the line you ask for is beyond the
/// number of visible scan-lines in the current video mode, it waits util
/// the last visible scan-line is complete.
///
/// If you wait for the last visible line until drawing, you stand the
/// best chance of your pixels operations on the video RAM being
/// completed before scan-lines start being sent to the monitor for the
/// next frame.
///
/// You can also use this for a crude `16.7 ms` delay but note that
/// some video modes run at `70 Hz` and so this would then give you a
/// `14.3ms` second delay.
pub extern "C" fn video_wait_for_line(line: u16) {
	let desired_line = line.min(vga::get_num_scan_lines());
	loop {
		let current_line = vga::get_scan_line();
		if current_line == desired_line {
			break;
		}
	}
}

/// Get information about the Block Devices in the system.
///
/// Block Devices are also known as *disk drives*. They can be read from
/// (and often written to) but only in units called *blocks* or *sectors*.
///
/// The BIOS should enumerate removable devices first, followed by fixed
/// devices.
///
/// The set of devices is not expected to change at run-time - removal of
/// media is indicated with a boolean field in the
/// `block_dev::DeviceInfo` structure.
pub extern "C" fn block_dev_get_info(device: u8) -> common::Option<common::block_dev::DeviceInfo> {
	match device {
		0 => {
			common::Option::Some(common::block_dev::DeviceInfo {
				// This is the built-in SD card slot
				name: common::types::ApiString::new("SdCard0"),
				device_type: common::block_dev::DeviceType::SecureDigitalCard,
				// This is the standard for SD cards
				block_size: 512,
				// TODO: scan the card here
				num_blocks: 0,
				// No motorised eject
				ejectable: false,
				// But you can take the card out
				removable: true,
				// Pretend the card is out
				media_present: true,
				// Don't care about this value when card is out
				read_only: false,
			})
		}
		_ => {
			// Nothing else supported by this BIOS
			common::Option::None
		}
	}
}

/// Write one or more sectors to a block device.
///
/// The function will block until all data is written. The array pointed
/// to by `data` must be `num_blocks * block_size` in length, where
/// `block_size` is given by `block_dev_get_info`.
///
/// There are no requirements on the alignment of `data` but if it is
/// aligned, the BIOS may be able to use a higher-performance code path.
pub extern "C" fn block_write(
	_device: u8,
	_block: u64,
	_num_blocks: u8,
	_data: common::ApiByteSlice,
) -> common::Result<()> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Read one or more sectors to a block device.
///
/// The function will block until all data is read. The array pointed
/// to by `data` must be `num_blocks * block_size` in length, where
/// `block_size` is given by `block_dev_get_info`.
///
/// There are no requirements on the alignment of `data` but if it is
/// aligned, the BIOS may be able to use a higher-performance code path.
pub extern "C" fn block_read(
	_device: u8,
	_block: u64,
	_num_blocks: u8,
	_data: common::ApiBuffer,
) -> common::Result<()> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Verify one or more sectors on a block device (that is read them and
/// check they match the given data).
///
/// The function will block until all data is verified. The array pointed
/// to by `data` must be `num_blocks * block_size` in length, where
/// `block_size` is given by `block_dev_get_info`.
///
/// There are no requirements on the alignment of `data` but if it is
/// aligned, the BIOS may be able to use a higher-performance code path.
pub extern "C" fn block_verify(
	_device: u8,
	_block: u64,
	_num_blocks: u8,
	_data: common::ApiByteSlice,
) -> common::Result<()> {
	common::Result::Err(common::Error::Unimplemented)
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
