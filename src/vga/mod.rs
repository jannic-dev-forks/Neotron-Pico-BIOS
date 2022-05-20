//! # VGA Driver for the Neotron Pico
//!
//! VGA output on the Neotron Pico uses 14 GPIO pins and two PIO state machines.
//!
//! It can generate 640x480@60Hz and 640x400@70Hz standard VGA video, with a
//! 25.2 MHz pixel clock. The spec is 25.175 MHz, so we are 0.1% off). The
//! assumption is that the CPU is clocked at 126 MHz, i.e. 5x the pixel
//! clock. All of the PIO code relies on this assumption!
//!
//! Currently only an 80x25 two-colour text-mode is supported. Other modes will be
//! added in the future.

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

// -----------------------------------------------------------------------------
// Sub-modules
// -----------------------------------------------------------------------------

mod font16;
mod font8;

// -----------------------------------------------------------------------------
// Imports
// -----------------------------------------------------------------------------

use core::sync::atomic::{
	AtomicBool, AtomicPtr, AtomicU16, AtomicU32, AtomicU8, AtomicUsize, Ordering,
};
use defmt::debug;
use neotron_common_bios::video::{
	Attr, Glyph, GlyphAttr, TextBackgroundColour, TextForegroundColour,
};
use rp_pico::hal::pio::PIOExt;

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

/// A font
pub struct Font<'a> {
	height: usize,
	data: &'a [u8],
}

/// Holds some data necessary to present a very very basic text console.
///
/// No ANSI support here! The OS handles that and writes direct to our video
/// memory.
///
/// Used by Core 0 to control writes to a shared text-buffer on boot-up.
pub struct TextConsole {
	current_col: AtomicU8,
	current_row: AtomicU8,
	text_buffer: AtomicPtr<GlyphAttr>,
	attr: AtomicU8,
}

/// Describes one scan-line's worth of pixels, including the length word required by the Pixel FIFO.
#[repr(C, align(16))]
struct LineBuffer {
	/// Must be one less than the number of pixel-pairs in `pixels`. This value
	/// is DMA'd to the FIFO, so `repr(C)` is important to ensure it isn't
	/// re-ordered.
	length: u32,
	/// Pixels to be displayed, grouped into pairs (to save FIFO space and reduce DMA bandwidth)
	pixels: [RGBColour; MAX_NUM_PIXEL_PAIRS_PER_LINE * 2],
	/// Set to `true` when the the main loop can fill this buffer with pixels.
	ready_for_drawing: AtomicBool,
	/// Which line number should the main loop draw here.
	line_number: AtomicU16,
}

/// Describes the polarity of a sync pulse.
///
/// Some pulses are positive (active-high), some are negative (active-low).
pub enum SyncPolarity {
	/// An active-high pulse
	Positive,
	/// An active-low pulse
	Negative,
}

/// Holds the four scan-line timing FIFO words we need for one scan-line.
///
/// See `make_timing` for a function which can generate these words. We DMA
/// them into the timing FIFO, so they must sit on a 16-byte boundary.
#[repr(C, align(16))]
struct ScanlineTimingBuffer {
	data: [u32; 4],
}

/// Holds the different kinds of scan-line timing buffers we need for various
/// portions of the screen.
struct TimingBuffer {
	/// We use this when there are visible pixels on screen
	visible_line: ScanlineTimingBuffer,
	/// We use this during the v-sync front-porch and v-sync back-porch
	vblank_porch_buffer: ScanlineTimingBuffer,
	/// We use this during the v-sync sync pulse
	vblank_sync_buffer: ScanlineTimingBuffer,
	/// The last visible scan-line,
	visible_lines_ends_at: u16,
	/// The last scan-line of the front porch
	front_porch_end_at: u16,
	/// The last scan-line of the sync pulse
	sync_pulse_ends_at: u16,
	/// The last scan-line of the back-porch (and the frame)
	back_porch_ends_at: u16,
}

/// Represents a 12-bit colour value.
///
/// Each channel has four-bits, and they are packed in `GBR` format. This is
/// so the PIO can shift them out right-first, and we have RED0 assigned to
/// the lowest GPIO pin.
#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct RGBColour(u16);

/// Represents two `RGBColour` pixels packed together.
///
/// The `first` pixel is packed in the lower 16-bits. This is because the PIO
/// shifts-right.
#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct RGBPair(u32);

// -----------------------------------------------------------------------------
// Static and Const Data
// -----------------------------------------------------------------------------

/// How many pixels per scan-line.
///
/// Adjust the pixel PIO program to run at the right speed to the screen is
/// filled. For example, if this is only 320 but you are aiming at 640x480,
/// make the pixel PIO take twice as long per pixel.
const MAX_NUM_PIXELS_PER_LINE: usize = 640;

/// Maximum number of lines on screen.
const MAX_NUM_LINES: usize = 480;

/// How many pixel pairs we send out.
///
/// Each pixel is two 12-bit values packed into one 32-bit word(an `RGBPair`).
/// This is to make more efficient use of DMA and FIFO resources.
const MAX_NUM_PIXEL_PAIRS_PER_LINE: usize = MAX_NUM_PIXELS_PER_LINE / 2;

/// The highest number of columns in any text mode.
pub const MAX_TEXT_COLS: usize = MAX_NUM_PIXELS_PER_LINE / 8;

/// The highest number of rows in any text mode.
pub const MAX_TEXT_ROWS: usize = MAX_NUM_LINES as usize / 8;

/// Current number of visible columns.
///
/// Must be `<= MAX_TEXT_COLS`
pub static NUM_TEXT_COLS: AtomicUsize = AtomicUsize::new(80);

/// Current number of visible rows.
///
/// Must be `<= MAX_TEXT_ROWS`
pub static NUM_TEXT_ROWS: AtomicUsize = AtomicUsize::new(25);

/// Used to signal when Core 1 has started
static CORE1_START_FLAG: AtomicBool = AtomicBool::new(false);

/// Stores our timing data which we DMA into the timing PIO State Machine
static mut TIMING_BUFFER: TimingBuffer = TimingBuffer::make_640x480();

/// Stores which mode we are in
static mut VIDEO_MODE: crate::common::video::Mode = crate::common::video::Mode::new(
	crate::common::video::Timing::T640x480,
	crate::common::video::Format::Text8x16,
);

/// Tracks which scan-line we are currently on (for timing purposes => it goes 0..`TIMING_BUFFER.back_porch_ends_at`)
static CURRENT_TIMING_LINE: AtomicU16 = AtomicU16::new(0);

/// Tracks which scan-line we are currently rendering (for pixel purposes => it goes 0..NUM_LINES)
static CURRENT_PLAYOUT_LINE: AtomicU16 = AtomicU16::new(0);

/// Somewhere to stash the DMA controller object, so the IRQ can find it
static mut DMA_PERIPH: Option<super::pac::DMA> = None;

/// DMA channel for the timing FIFO
const TIMING_DMA_CHAN: usize = 0;

/// DMA channel for the pixel FIFO
const PIXEL_DMA_CHAN: usize = 1;

/// One scan-line's worth of 12-bit pixels, used for the even scan-lines (0, 2, 4 ... NUM_LINES-2).
///
/// Gets read by DMA, which pushes them into the pixel state machine's FIFO.
///
/// Gets written to by `render_scanline()` running on Core 1.
static mut PIXEL_DATA_BUFFER_EVEN: LineBuffer = LineBuffer {
	length: 0,
	pixels: [RGBColour(0); MAX_NUM_PIXEL_PAIRS_PER_LINE * 2],
	ready_for_drawing: AtomicBool::new(false),
	line_number: AtomicU16::new(0),
};

/// One scan-line's worth of 12-bit pixels, used for the odd scan-lines (1, 3, 5 ... NUM_LINES-1).
///
/// Gets read by DMA, which pushes them into the pixel state machine's FIFO.
///
/// Gets written to by `render_scanline()` running on Core 1.
static mut PIXEL_DATA_BUFFER_ODD: LineBuffer = LineBuffer {
	length: 0,
	pixels: [RGBColour(0); MAX_NUM_PIXEL_PAIRS_PER_LINE * 2],
	ready_for_drawing: AtomicBool::new(false),
	line_number: AtomicU16::new(0),
};

/// A count of how many lines failed to render in time
pub static CANT_PLAY_COUNT: AtomicU32 = AtomicU32::new(0);
pub static RENDER_WAITS_COUNT: AtomicU32 = AtomicU32::new(0);

/// Holds the 256-entry palette for indexed colour modes.
static mut VIDEO_PALETTE: [RGBColour; 256] = [
	// Index 0: #000000
	RGBColour::new(0x00, 0x00, 0x00),
	// Index 1: #800000
	RGBColour::new(0x80, 0x00, 0x00),
	// Index 2: #008000
	RGBColour::new(0x00, 0x80, 0x00),
	// Index 3: #808000
	RGBColour::new(0x80, 0x80, 0x00),
	// Index 4: #000080
	RGBColour::new(0x00, 0x00, 0x80),
	// Index 5: #800080
	RGBColour::new(0x80, 0x00, 0x80),
	// Index 6: #008080
	RGBColour::new(0x00, 0x80, 0x80),
	// Index 7: #c0c0c00
	RGBColour::new(0x0c, 0x0c, 0x00),
	// Index 8: #808080
	RGBColour::new(0x80, 0x80, 0x80),
	// Index 9: #ff00000
	RGBColour::new(0xf0, 0x00, 0x00),
	// Index 10: #00ff0000
	RGBColour::new(0xff, 0x00, 0x00),
	// Index 11: #ffff000
	RGBColour::new(0xff, 0xf0, 0x00),
	// Index 12: #0000ff
	RGBColour::new(0x00, 0x00, 0xff),
	// Index 13: #ff00ff
	RGBColour::new(0xff, 0x00, 0xff),
	// Index 14: #00ffff
	RGBColour::new(0x00, 0xff, 0xff),
	// Index 15: #ffffff
	RGBColour::new(0xff, 0xff, 0xff),
	// Index 16: #000000
	RGBColour::new(0x00, 0x00, 0x00),
	// Index 17: #00005f
	RGBColour::new(0x00, 0x00, 0x5f),
	// Index 18: #000087
	RGBColour::new(0x00, 0x00, 0x87),
	// Index 19: #0000af
	RGBColour::new(0x00, 0x00, 0xaf),
	// Index 20: #0000d7
	RGBColour::new(0x00, 0x00, 0xd7),
	// Index 21: #0000ff
	RGBColour::new(0x00, 0x00, 0xff),
	// Index 22: #005f00
	RGBColour::new(0x00, 0x5f, 0x00),
	// Index 23: #005f5f
	RGBColour::new(0x00, 0x5f, 0x5f),
	// Index 24: #005f87
	RGBColour::new(0x00, 0x5f, 0x87),
	// Index 25: #005faf
	RGBColour::new(0x00, 0x5f, 0xaf),
	// Index 26: #005fd7
	RGBColour::new(0x00, 0x5f, 0xd7),
	// Index 27: #005fff
	RGBColour::new(0x00, 0x5f, 0xff),
	// Index 28: #008700
	RGBColour::new(0x00, 0x87, 0x00),
	// Index 29: #00875f
	RGBColour::new(0x00, 0x87, 0x5f),
	// Index 30: #008787
	RGBColour::new(0x00, 0x87, 0x87),
	// Index 31: #0087af
	RGBColour::new(0x00, 0x87, 0xaf),
	// Index 32: #0087d7
	RGBColour::new(0x00, 0x87, 0xd7),
	// Index 33: #0087ff
	RGBColour::new(0x00, 0x87, 0xff),
	// Index 34: #00af00
	RGBColour::new(0x00, 0xaf, 0x00),
	// Index 35: #00af5f
	RGBColour::new(0x00, 0xaf, 0x5f),
	// Index 36: #00af87
	RGBColour::new(0x00, 0xaf, 0x87),
	// Index 37: #00afaf
	RGBColour::new(0x00, 0xaf, 0xaf),
	// Index 38: #00afd7
	RGBColour::new(0x00, 0xaf, 0xd7),
	// Index 39: #00afff
	RGBColour::new(0x00, 0xaf, 0xff),
	// Index 40: #00d700
	RGBColour::new(0x00, 0xd7, 0x00),
	// Index 41: #00d75f
	RGBColour::new(0x00, 0xd7, 0x5f),
	// Index 42: #00d787
	RGBColour::new(0x00, 0xd7, 0x87),
	// Index 43: #00d7af
	RGBColour::new(0x00, 0xd7, 0xaf),
	// Index 44: #00d7d7
	RGBColour::new(0x00, 0xd7, 0xd7),
	// Index 45: #00d7ff
	RGBColour::new(0x00, 0xd7, 0xff),
	// Index 46: #00ff00
	RGBColour::new(0x00, 0xff, 0x00),
	// Index 47: #00ff5f
	RGBColour::new(0x00, 0xff, 0x5f),
	// Index 48: #00ff87
	RGBColour::new(0x00, 0xff, 0x87),
	// Index 49: #00ffaf
	RGBColour::new(0x00, 0xff, 0xaf),
	// Index 50: #00ffd7
	RGBColour::new(0x00, 0xff, 0xd7),
	// Index 51: #00ffff
	RGBColour::new(0x00, 0xff, 0xff),
	// Index 52: #5f0000
	RGBColour::new(0x5f, 0x00, 0x00),
	// Index 53: #5f005f
	RGBColour::new(0x5f, 0x00, 0x5f),
	// Index 54: #5f0087
	RGBColour::new(0x5f, 0x00, 0x87),
	// Index 55: #5f00af
	RGBColour::new(0x5f, 0x00, 0xaf),
	// Index 56: #5f00d7
	RGBColour::new(0x5f, 0x00, 0xd7),
	// Index 57: #5f00ff
	RGBColour::new(0x5f, 0x00, 0xff),
	// Index 58: #5f5f00
	RGBColour::new(0x5f, 0x5f, 0x00),
	// Index 59: #5f5f5f
	RGBColour::new(0x5f, 0x5f, 0x5f),
	// Index 60: #5f5f87
	RGBColour::new(0x5f, 0x5f, 0x87),
	// Index 61: #5f5faf
	RGBColour::new(0x5f, 0x5f, 0xaf),
	// Index 62: #5f5fd7
	RGBColour::new(0x5f, 0x5f, 0xd7),
	// Index 63: #5f5fff
	RGBColour::new(0x5f, 0x5f, 0xff),
	// Index 64: #5f8700
	RGBColour::new(0x5f, 0x87, 0x00),
	// Index 65: #5f875f
	RGBColour::new(0x5f, 0x87, 0x5f),
	// Index 66: #5f8787
	RGBColour::new(0x5f, 0x87, 0x87),
	// Index 67: #5f87af
	RGBColour::new(0x5f, 0x87, 0xaf),
	// Index 68: #5f87d7
	RGBColour::new(0x5f, 0x87, 0xd7),
	// Index 69: #5f87ff
	RGBColour::new(0x5f, 0x87, 0xff),
	// Index 70: #5faf00
	RGBColour::new(0x5f, 0xaf, 0x00),
	// Index 71: #5faf5f
	RGBColour::new(0x5f, 0xaf, 0x5f),
	// Index 72: #5faf87
	RGBColour::new(0x5f, 0xaf, 0x87),
	// Index 73: #5fafaf
	RGBColour::new(0x5f, 0xaf, 0xaf),
	// Index 74: #5fafd7
	RGBColour::new(0x5f, 0xaf, 0xd7),
	// Index 75: #5fafff
	RGBColour::new(0x5f, 0xaf, 0xff),
	// Index 76: #5fd700
	RGBColour::new(0x5f, 0xd7, 0x00),
	// Index 77: #5fd75f
	RGBColour::new(0x5f, 0xd7, 0x5f),
	// Index 78: #5fd787
	RGBColour::new(0x5f, 0xd7, 0x87),
	// Index 79: #5fd7af
	RGBColour::new(0x5f, 0xd7, 0xaf),
	// Index 80: #5fd7d7
	RGBColour::new(0x5f, 0xd7, 0xd7),
	// Index 81: #5fd7ff
	RGBColour::new(0x5f, 0xd7, 0xff),
	// Index 82: #5fff00
	RGBColour::new(0x5f, 0xff, 0x00),
	// Index 83: #5fff5f
	RGBColour::new(0x5f, 0xff, 0x5f),
	// Index 84: #5fff87
	RGBColour::new(0x5f, 0xff, 0x87),
	// Index 85: #5fffaf
	RGBColour::new(0x5f, 0xff, 0xaf),
	// Index 86: #5fffd7
	RGBColour::new(0x5f, 0xff, 0xd7),
	// Index 87: #5fffff
	RGBColour::new(0x5f, 0xff, 0xff),
	// Index 88: #870000
	RGBColour::new(0x87, 0x00, 0x00),
	// Index 89: #87005f
	RGBColour::new(0x87, 0x00, 0x5f),
	// Index 90: #870087
	RGBColour::new(0x87, 0x00, 0x87),
	// Index 91: #8700af
	RGBColour::new(0x87, 0x00, 0xaf),
	// Index 92: #8700d7
	RGBColour::new(0x87, 0x00, 0xd7),
	// Index 93: #8700ff
	RGBColour::new(0x87, 0x00, 0xff),
	// Index 94: #875f00
	RGBColour::new(0x87, 0x5f, 0x00),
	// Index 95: #875f5f
	RGBColour::new(0x87, 0x5f, 0x5f),
	// Index 96: #875f87
	RGBColour::new(0x87, 0x5f, 0x87),
	// Index 97: #875faf
	RGBColour::new(0x87, 0x5f, 0xaf),
	// Index 98: #875fd7
	RGBColour::new(0x87, 0x5f, 0xd7),
	// Index 99: #875fff
	RGBColour::new(0x87, 0x5f, 0xff),
	// Index 100: #878700
	RGBColour::new(0x87, 0x87, 0x00),
	// Index 101: #87875f
	RGBColour::new(0x87, 0x87, 0x5f),
	// Index 102: #878787
	RGBColour::new(0x87, 0x87, 0x87),
	// Index 103: #8787af
	RGBColour::new(0x87, 0x87, 0xaf),
	// Index 104: #8787d7
	RGBColour::new(0x87, 0x87, 0xd7),
	// Index 105: #8787ff
	RGBColour::new(0x87, 0x87, 0xff),
	// Index 106: #87af00
	RGBColour::new(0x87, 0xaf, 0x00),
	// Index 107: #87af5f
	RGBColour::new(0x87, 0xaf, 0x5f),
	// Index 108: #87af87
	RGBColour::new(0x87, 0xaf, 0x87),
	// Index 109: #87afaf
	RGBColour::new(0x87, 0xaf, 0xaf),
	// Index 110: #87afd7
	RGBColour::new(0x87, 0xaf, 0xd7),
	// Index 111: #87afff
	RGBColour::new(0x87, 0xaf, 0xff),
	// Index 112: #87d700
	RGBColour::new(0x87, 0xd7, 0x00),
	// Index 113: #87d75f
	RGBColour::new(0x87, 0xd7, 0x5f),
	// Index 114: #87d787
	RGBColour::new(0x87, 0xd7, 0x87),
	// Index 115: #87d7af
	RGBColour::new(0x87, 0xd7, 0xaf),
	// Index 116: #87d7d7
	RGBColour::new(0x87, 0xd7, 0xd7),
	// Index 117: #87d7ff
	RGBColour::new(0x87, 0xd7, 0xff),
	// Index 118: #87ff00
	RGBColour::new(0x87, 0xff, 0x00),
	// Index 119: #87ff5f
	RGBColour::new(0x87, 0xff, 0x5f),
	// Index 120: #87ff87
	RGBColour::new(0x87, 0xff, 0x87),
	// Index 121: #87ffaf
	RGBColour::new(0x87, 0xff, 0xaf),
	// Index 122: #87ffd7
	RGBColour::new(0x87, 0xff, 0xd7),
	// Index 123: #87ffff
	RGBColour::new(0x87, 0xff, 0xff),
	// Index 124: #af0000
	RGBColour::new(0xaf, 0x00, 0x00),
	// Index 125: #af005f
	RGBColour::new(0xaf, 0x00, 0x5f),
	// Index 126: #af0087
	RGBColour::new(0xaf, 0x00, 0x87),
	// Index 127: #af00af
	RGBColour::new(0xaf, 0x00, 0xaf),
	// Index 128: #af00d7
	RGBColour::new(0xaf, 0x00, 0xd7),
	// Index 129: #af00ff
	RGBColour::new(0xaf, 0x00, 0xff),
	// Index 130: #af5f00
	RGBColour::new(0xaf, 0x5f, 0x00),
	// Index 131: #af5f5f
	RGBColour::new(0xaf, 0x5f, 0x5f),
	// Index 132: #af5f87
	RGBColour::new(0xaf, 0x5f, 0x87),
	// Index 133: #af5faf
	RGBColour::new(0xaf, 0x5f, 0xaf),
	// Index 134: #af5fd7
	RGBColour::new(0xaf, 0x5f, 0xd7),
	// Index 135: #af5fff
	RGBColour::new(0xaf, 0x5f, 0xff),
	// Index 136: #af8700
	RGBColour::new(0xaf, 0x87, 0x00),
	// Index 137: #af875f
	RGBColour::new(0xaf, 0x87, 0x5f),
	// Index 138: #af8787
	RGBColour::new(0xaf, 0x87, 0x87),
	// Index 139: #af87af
	RGBColour::new(0xaf, 0x87, 0xaf),
	// Index 140: #af87d7
	RGBColour::new(0xaf, 0x87, 0xd7),
	// Index 141: #af87ff
	RGBColour::new(0xaf, 0x87, 0xff),
	// Index 142: #afaf00
	RGBColour::new(0xaf, 0xaf, 0x00),
	// Index 143: #afaf5f
	RGBColour::new(0xaf, 0xaf, 0x5f),
	// Index 144: #afaf87
	RGBColour::new(0xaf, 0xaf, 0x87),
	// Index 145: #afafaf
	RGBColour::new(0xaf, 0xaf, 0xaf),
	// Index 146: #afafd7
	RGBColour::new(0xaf, 0xaf, 0xd7),
	// Index 147: #afafff
	RGBColour::new(0xaf, 0xaf, 0xff),
	// Index 148: #afd700
	RGBColour::new(0xaf, 0xd7, 0x00),
	// Index 149: #afd75f
	RGBColour::new(0xaf, 0xd7, 0x5f),
	// Index 150: #afd787
	RGBColour::new(0xaf, 0xd7, 0x87),
	// Index 151: #afd7af
	RGBColour::new(0xaf, 0xd7, 0xaf),
	// Index 152: #afd7d7
	RGBColour::new(0xaf, 0xd7, 0xd7),
	// Index 153: #afd7ff
	RGBColour::new(0xaf, 0xd7, 0xff),
	// Index 154: #afff00
	RGBColour::new(0xaf, 0xff, 0x00),
	// Index 155: #afff5f
	RGBColour::new(0xaf, 0xff, 0x5f),
	// Index 156: #afff87
	RGBColour::new(0xaf, 0xff, 0x87),
	// Index 157: #afffaf
	RGBColour::new(0xaf, 0xff, 0xaf),
	// Index 158: #afffd7
	RGBColour::new(0xaf, 0xff, 0xd7),
	// Index 159: #afffff
	RGBColour::new(0xaf, 0xff, 0xff),
	// Index 160: #d70000
	RGBColour::new(0xd7, 0x00, 0x00),
	// Index 161: #d7005f
	RGBColour::new(0xd7, 0x00, 0x5f),
	// Index 162: #d70087
	RGBColour::new(0xd7, 0x00, 0x87),
	// Index 163: #d700af
	RGBColour::new(0xd7, 0x00, 0xaf),
	// Index 164: #d700d7
	RGBColour::new(0xd7, 0x00, 0xd7),
	// Index 165: #d700ff
	RGBColour::new(0xd7, 0x00, 0xff),
	// Index 166: #d75f00
	RGBColour::new(0xd7, 0x5f, 0x00),
	// Index 167: #d75f5f
	RGBColour::new(0xd7, 0x5f, 0x5f),
	// Index 168: #d75f87
	RGBColour::new(0xd7, 0x5f, 0x87),
	// Index 169: #d75faf
	RGBColour::new(0xd7, 0x5f, 0xaf),
	// Index 170: #d75fd7
	RGBColour::new(0xd7, 0x5f, 0xd7),
	// Index 171: #d75fff
	RGBColour::new(0xd7, 0x5f, 0xff),
	// Index 172: #d78700
	RGBColour::new(0xd7, 0x87, 0x00),
	// Index 173: #d7875f
	RGBColour::new(0xd7, 0x87, 0x5f),
	// Index 174: #d78787
	RGBColour::new(0xd7, 0x87, 0x87),
	// Index 175: #d787af
	RGBColour::new(0xd7, 0x87, 0xaf),
	// Index 176: #d787d7
	RGBColour::new(0xd7, 0x87, 0xd7),
	// Index 177: #d787ff
	RGBColour::new(0xd7, 0x87, 0xff),
	// Index 178: #d7af00
	RGBColour::new(0xd7, 0xaf, 0x00),
	// Index 179: #d7af5f
	RGBColour::new(0xd7, 0xaf, 0x5f),
	// Index 180: #d7af87
	RGBColour::new(0xd7, 0xaf, 0x87),
	// Index 181: #d7afaf
	RGBColour::new(0xd7, 0xaf, 0xaf),
	// Index 182: #d7afd7
	RGBColour::new(0xd7, 0xaf, 0xd7),
	// Index 183: #d7afff
	RGBColour::new(0xd7, 0xaf, 0xff),
	// Index 184: #d7d700
	RGBColour::new(0xd7, 0xd7, 0x00),
	// Index 185: #d7d75f
	RGBColour::new(0xd7, 0xd7, 0x5f),
	// Index 186: #d7d787
	RGBColour::new(0xd7, 0xd7, 0x87),
	// Index 187: #d7d7af
	RGBColour::new(0xd7, 0xd7, 0xaf),
	// Index 188: #d7d7d7
	RGBColour::new(0xd7, 0xd7, 0xd7),
	// Index 189: #d7d7ff
	RGBColour::new(0xd7, 0xd7, 0xff),
	// Index 190: #d7ff00
	RGBColour::new(0xd7, 0xff, 0x00),
	// Index 191: #d7ff5f
	RGBColour::new(0xd7, 0xff, 0x5f),
	// Index 192: #d7ff87
	RGBColour::new(0xd7, 0xff, 0x87),
	// Index 193: #d7ffaf
	RGBColour::new(0xd7, 0xff, 0xaf),
	// Index 194: #d7ffd7
	RGBColour::new(0xd7, 0xff, 0xd7),
	// Index 195: #d7ffff
	RGBColour::new(0xd7, 0xff, 0xff),
	// Index 196: #ff0000
	RGBColour::new(0xff, 0x00, 0x00),
	// Index 197: #ff005f
	RGBColour::new(0xff, 0x00, 0x5f),
	// Index 198: #ff0087
	RGBColour::new(0xff, 0x00, 0x87),
	// Index 199: #ff00af
	RGBColour::new(0xff, 0x00, 0xaf),
	// Index 200: #ff00d7
	RGBColour::new(0xff, 0x00, 0xd7),
	// Index 201: #ff00ff
	RGBColour::new(0xff, 0x00, 0xff),
	// Index 202: #ff5f00
	RGBColour::new(0xff, 0x5f, 0x00),
	// Index 203: #ff5f5f
	RGBColour::new(0xff, 0x5f, 0x5f),
	// Index 204: #ff5f87
	RGBColour::new(0xff, 0x5f, 0x87),
	// Index 205: #ff5faf
	RGBColour::new(0xff, 0x5f, 0xaf),
	// Index 206: #ff5fd7
	RGBColour::new(0xff, 0x5f, 0xd7),
	// Index 207: #ff5fff
	RGBColour::new(0xff, 0x5f, 0xff),
	// Index 208: #ff8700
	RGBColour::new(0xff, 0x87, 0x00),
	// Index 209: #ff875f
	RGBColour::new(0xff, 0x87, 0x5f),
	// Index 210: #ff8787
	RGBColour::new(0xff, 0x87, 0x87),
	// Index 211: #ff87af
	RGBColour::new(0xff, 0x87, 0xaf),
	// Index 212: #ff87d7
	RGBColour::new(0xff, 0x87, 0xd7),
	// Index 213: #ff87ff
	RGBColour::new(0xff, 0x87, 0xff),
	// Index 214: #ffaf00
	RGBColour::new(0xff, 0xaf, 0x00),
	// Index 215: #ffaf5f
	RGBColour::new(0xff, 0xaf, 0x5f),
	// Index 216: #ffaf87
	RGBColour::new(0xff, 0xaf, 0x87),
	// Index 217: #ffafaf
	RGBColour::new(0xff, 0xaf, 0xaf),
	// Index 218: #ffafd7
	RGBColour::new(0xff, 0xaf, 0xd7),
	// Index 219: #ffafff
	RGBColour::new(0xff, 0xaf, 0xff),
	// Index 220: #ffd700
	RGBColour::new(0xff, 0xd7, 0x00),
	// Index 221: #ffd75f
	RGBColour::new(0xff, 0xd7, 0x5f),
	// Index 222: #ffd787
	RGBColour::new(0xff, 0xd7, 0x87),
	// Index 223: #ffd7af
	RGBColour::new(0xff, 0xd7, 0xaf),
	// Index 224: #ffd7d7
	RGBColour::new(0xff, 0xd7, 0xd7),
	// Index 225: #ffd7ff
	RGBColour::new(0xff, 0xd7, 0xff),
	// Index 226: #ffff00
	RGBColour::new(0xff, 0xff, 0x00),
	// Index 227: #ffff5f
	RGBColour::new(0xff, 0xff, 0x5f),
	// Index 228: #ffff87
	RGBColour::new(0xff, 0xff, 0x87),
	// Index 229: #ffffaf
	RGBColour::new(0xff, 0xff, 0xaf),
	// Index 230: #ffffd7
	RGBColour::new(0xff, 0xff, 0xd7),
	// Index 231: #ffffff
	RGBColour::new(0xff, 0xff, 0xff),
	// Index 232: #080808
	RGBColour::new(0x08, 0x08, 0x08),
	// Index 233: #121212
	RGBColour::new(0x12, 0x12, 0x12),
	// Index 234: #1c1c1c
	RGBColour::new(0x1c, 0x1c, 0x1c),
	// Index 235: #262626
	RGBColour::new(0x26, 0x26, 0x26),
	// Index 236: #303030
	RGBColour::new(0x30, 0x30, 0x30),
	// Index 237: #3a3a3a
	RGBColour::new(0x3a, 0x3a, 0x3a),
	// Index 238: #444444
	RGBColour::new(0x44, 0x44, 0x44),
	// Index 239: #4e4e4e
	RGBColour::new(0x4e, 0x4e, 0x4e),
	// Index 240: #585858
	RGBColour::new(0x58, 0x58, 0x58),
	// Index 241: #606060
	RGBColour::new(0x60, 0x60, 0x60),
	// Index 242: #666666
	RGBColour::new(0x66, 0x66, 0x66),
	// Index 243: #767676
	RGBColour::new(0x76, 0x76, 0x76),
	// Index 244: #808080
	RGBColour::new(0x80, 0x80, 0x80),
	// Index 245: #8a8a8a
	RGBColour::new(0x8a, 0x8a, 0x8a),
	// Index 246: #949494
	RGBColour::new(0x94, 0x94, 0x94),
	// Index 247: #9e9e9e
	RGBColour::new(0x9e, 0x9e, 0x9e),
	// Index 248: #a8a8a8
	RGBColour::new(0xa8, 0xa8, 0xa8),
	// Index 249: #b2b2b2
	RGBColour::new(0xb2, 0xb2, 0xb2),
	// Index 250: #bcbcbc
	RGBColour::new(0xbc, 0xbc, 0xbc),
	// Index 251: #c6c6c6
	RGBColour::new(0xc6, 0xc6, 0xc6),
	// Index 252: #d0d0d0
	RGBColour::new(0xd0, 0xd0, 0xd0),
	// Index 253: #dadada
	RGBColour::new(0xda, 0xda, 0xda),
	// Index 254: #e4e4e4
	RGBColour::new(0xe4, 0xe4, 0xe4),
	// Index 255: #eeeeee
	RGBColour::new(0xee, 0xee, 0xee),
];

/// This is our text buffer.
///
/// This is arranged as `NUM_TEXT_ROWS` rows of `NUM_TEXT_COLS` columns. Each
/// item is an index into `font16::FONT_DATA` plus an 8-bit attribute.
///
/// Written to by Core 0, and read from by `RenderEngine` running on Core 1.
pub static mut GLYPH_ATTR_ARRAY: [GlyphAttr; MAX_TEXT_COLS * MAX_TEXT_ROWS] =
	[GlyphAttr(0); MAX_TEXT_COLS * MAX_TEXT_ROWS];

/// Core 1 entry function.
///
/// This is a naked function I have pre-compiled to thumb-2 instructions. I
/// could use inline assembler, but then I'd have to make you install
/// arm-none-eabi-as or arm-none-eabi-gcc, or wait until `llvm_asm!` is
/// stablised.
///
/// We want this function to load the three parameters (which we placed in
/// Core 1's stack) into registers. We then jump to the third of these, which
/// effectively makes a function call with the first two values as function
/// arguments.
static CORE1_ENTRY_FUNCTION: [u16; 2] = [
	0xbd03, // pop {r0, r1, pc}
	0x46c0, // nop - pad this out to 32-bits long
];

/// A set of useful constants representing common RGB colours.
pub mod colours {
	/// The colour white
	pub const WHITE: super::RGBColour = super::RGBColour(0xFFF);

	/// The colour black
	pub const BLACK: super::RGBColour = super::RGBColour(0x000);

	/// The colour blue
	pub const BLUE: super::RGBColour = super::RGBColour(0xF00);

	/// The colour green
	pub const GREEN: super::RGBColour = super::RGBColour(0x0F0);

	/// The colour red
	pub const RED: super::RGBColour = super::RGBColour(0x00F);
}

// -----------------------------------------------------------------------------
// Functions
// -----------------------------------------------------------------------------

/// Initialise all the static data and peripherals we need for our video display.
///
/// We need to keep `pio` and `dma` to run the video. We need `resets` to set
/// things up, so we only borrow that.
pub fn init(
	pio: super::pac::PIO0,
	dma: super::pac::DMA,
	resets: &mut super::pac::RESETS,
	ppb: &mut crate::pac::PPB,
	fifo: &mut rp_pico::hal::sio::SioFifo,
	psm: &mut crate::pac::PSM,
) {
	// Grab PIO0 and the state machines it contains
	let (mut pio, sm0, sm1, _sm2, _sm3) = pio.split(resets);

	// This program runs the timing loop. We post timing data (i.e. the length
	// of each period, along with what the H-Sync and V-Sync pins should do)
	// and it sets the GPIO pins and busy-waits the appropriate amount of
	// time. It also takes an extra 'instruction' which we can use to trigger
	// the appropriate interrupts.
	//
	// Post <value:32> where value: <clock_cycles:14> <hsync:1> <vsync:1>
	// <instruction:16>
	//
	// The SM will execute the instruction (typically either a NOP or an IRQ),
	// set the H-Sync and V-Sync pins as desired, then wait the given number
	// of clock cycles.
	//
	// Note: autopull should be set to 32-bits, OSR is set to shift right.
	let timing_program = pio_proc::pio_asm!(
		".wrap_target"
		// Step 1. Push next 2 bits of OSR into `pins`, to set H-Sync and V-Sync
		"out pins, 2"
		// Step 2. Push last 14 bits of OSR into X for the timing loop.
		"out x, 14"
		// Step 3. Execute bottom 16-bits of OSR as an instruction. This take two cycles.
		"out exec, 16"
		// Spin until X is zero
		"loop0:"
			"jmp x-- loop0"
		".wrap"
	);

	// This is the video pixels program. It waits for an IRQ
	// (posted by the timing loop) then pulls pixel data from the FIFO. We post
	// the number of pixels for that line, then the pixel data.
	//
	// Post <num_pixels> <pixel1> <pixel2> ... <pixelN>; each <pixelX> maps to
	// the RGB output pins. On a Neotron Pico, there are 12 (4 Red, 4 Green and
	// 4 Blue) - so we set autopull to 12, and each value should be 12-bits long.
	//
	// Currently the FIFO supplies only the pixels, not the length value. When
	// we read the length from the FIFO as well, all hell breaks loose.
	//
	// Note autopull should be set to 32-bits, OSR is set to shift right.
	let pixel_program = pio_proc::pio_asm!(
		".wrap_target"
		// Wait for timing state machine to start visible line
		"wait 1 irq 0"
		// Read the line length (in pixel-pairs)
		"out x, 32"
		"loop1:"
			// Write out first pixel - takes 10 clocks per pixel
			"out pins, 16 [9]"
			// Write out second pixel - takes 10 clocks per pixel (allowing one clock for the jump)
			"out pins, 16 [8]"
			// Repeat until all pixel pairs sent
			"jmp x-- loop1"
		// Clear all pins after visible section
		"mov pins null"
		".wrap"
	);

	// These two state machines run thus:
	//
	// | Clock | Timing PIOSM  | Pixel PIOSM      |
	// |:------|:--------------|:-----------------|
	// | 1     | out pins, 2   | wait 1 irq 0     |
	// | 2     | out x, 14     | wait 1 irq 0     |
	// | 3     | out exec, 16  | wait 1 irq 0     |
	// | 4     | <exec irq>    | wait 1 irq 0     |
	// | 5     | jmp x-- loop0 | wait 1 irq 0     |
	// | 6     | jmp x-- loop0 | out x, 32        |
	// | 7     | jmp x-- loop0 | out pins, 16 [9] |
	// | 8     | jmp x-- loop0 | ..               |
	// | 9     | jmp x-- loop0 | ..               |
	// | 10    | jmp x-- loop0 | ..               |
	// | 11    | jmp x-- loop0 | ..               |
	// | 12    | jmp x-- loop0 | ..               |
	// | 13    | jmp x-- loop0 | ..               |
	// | 14    | jmp x-- loop0 | ..               |
	// | 15    | jmp x-- loop0 | ..               |
	// | 16    | jmp x-- loop0 | ..               |
	// | 17    | jmp x-- loop0 | out pins, 16 [8] |
	// | 18    | jmp x-- loop0 | ..               |
	// | 19    | jmp x-- loop0 | ..               |
	// | 20    | jmp x-- loop0 | ..               |
	// | 21    | jmp x-- loop0 | ..               |
	// | 22    | jmp x-- loop0 | ..               |
	// | 23    | jmp x-- loop0 | ..               |
	// | 23    | jmp x-- loop0 | ..               |
	// | 24    | jmp x-- loop0 | ..               |
	// | 25    | jmp x-- loop0 | jmp x-- loop1    |
	// | 26    | jmp x-- loop0 | out pins, 16 [9] |
	//
	// Note: Credit to
	// https://gregchadwick.co.uk/blog/playing-with-the-pico-pt5/ who had a
	// very similar idea to me, but wrote it up far better than I ever could.

	let timing_installed = pio.install(&timing_program.program).unwrap();
	let (mut timing_sm, _, timing_fifo) =
		rp_pico::hal::pio::PIOBuilder::from_program(timing_installed)
			.buffers(rp_pico::hal::pio::Buffers::OnlyTx)
			.out_pins(0, 2) // H-Sync is GPIO0, V-Sync is GPIO1
			.autopull(true)
			.out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
			.pull_threshold(32)
			.build(sm0);
	timing_sm.set_pindirs([
		(0, rp_pico::hal::pio::PinDir::Output),
		(1, rp_pico::hal::pio::PinDir::Output),
	]);

	// Important notes!
	//
	// You must not set a clock_divider (other than 1.0) on the pixel state
	// machine. You might want the pixels to be twice as wide (or mode), but
	// enabling a clock divider adds a lot of jitter (i.e. the start each
	// each line differs by some number of 126 MHz clock cycles).

	let pixels_installed = pio.install(&pixel_program.program).unwrap();
	let (mut pixel_sm, _, pixel_fifo) =
		rp_pico::hal::pio::PIOBuilder::from_program(pixels_installed)
			.buffers(rp_pico::hal::pio::Buffers::OnlyTx)
			.out_pins(2, 12) // Red0 is GPIO2, Blue3 is GPIO13
			.autopull(true)
			.out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
			.pull_threshold(32) // We read all 32-bits in each FIFO word
			.build(sm1);
	pixel_sm.set_pindirs((2..=13).map(|x| (x, rp_pico::hal::pio::PinDir::Output)));

	// Read from the timing buffer and write to the timing FIFO. We get an
	// IRQ when the transfer is complete (i.e. when line has been fully
	// loaded).
	dma.ch[TIMING_DMA_CHAN].ch_ctrl_trig.write(|w| {
		w.data_size().size_word();
		w.incr_read().set_bit();
		w.incr_write().clear_bit();
		unsafe { w.treq_sel().bits(timing_fifo.dreq_value()) };
		unsafe { w.chain_to().bits(TIMING_DMA_CHAN as u8) };
		unsafe { w.ring_size().bits(0) };
		w.ring_sel().clear_bit();
		w.bswap().clear_bit();
		w.irq_quiet().clear_bit();
		w.en().set_bit();
		w.sniff_en().clear_bit();
		w
	});
	dma.ch[TIMING_DMA_CHAN]
		.ch_read_addr
		.write(|w| unsafe { w.bits(TIMING_BUFFER.visible_line.data.as_ptr() as usize as u32) });
	dma.ch[TIMING_DMA_CHAN]
		.ch_write_addr
		.write(|w| unsafe { w.bits(timing_fifo.fifo_address() as usize as u32) });
	dma.ch[TIMING_DMA_CHAN]
		.ch_trans_count
		.write(|w| unsafe { w.bits(TIMING_BUFFER.visible_line.data.len() as u32) });

	// Read from the pixel buffer (even first) and write to the pixel FIFO
	dma.ch[PIXEL_DMA_CHAN].ch_ctrl_trig.write(|w| {
		w.data_size().size_word();
		w.incr_read().set_bit();
		w.incr_write().clear_bit();
		unsafe { w.treq_sel().bits(pixel_fifo.dreq_value()) };
		unsafe { w.chain_to().bits(PIXEL_DMA_CHAN as u8) };
		unsafe { w.ring_size().bits(0) };
		w.ring_sel().clear_bit();
		w.bswap().clear_bit();
		w.irq_quiet().clear_bit();
		w.en().set_bit();
		w.sniff_en().clear_bit();
		w
	});

	// This is where the data is coming from (the scan-line buffer)
	dma.ch[PIXEL_DMA_CHAN]
		.ch_read_addr
		.write(|w| unsafe { w.bits(PIXEL_DATA_BUFFER_EVEN.as_ptr()) });
	// This is where the data is going (the PIO FIFO)
	dma.ch[PIXEL_DMA_CHAN]
		.ch_write_addr
		.write(|w| unsafe { w.bits(pixel_fifo.fifo_address() as usize as u32) });
	// This is the count of words to send, which is the size of the array plus one for the length field.
	dma.ch[PIXEL_DMA_CHAN]
		.ch_trans_count
		.write(|w| unsafe { w.bits(MAX_NUM_PIXEL_PAIRS_PER_LINE as u32 + 1) });
	// Enable the DMA interrupts
	dma.inte0.write(|w| unsafe {
		w.inte0()
			.bits((1 << PIXEL_DMA_CHAN) | (1 << TIMING_DMA_CHAN))
	});

	// Set up the buffers
	unsafe {
		// These are the length value given to the PIO FSMs. They must be one
		// less than the number of pixels we actually have, because of how the
		// PIO FSM loop works.
		PIXEL_DATA_BUFFER_EVEN.length = (MAX_NUM_PIXEL_PAIRS_PER_LINE as u32) - 1;
		PIXEL_DATA_BUFFER_ODD.length = (MAX_NUM_PIXEL_PAIRS_PER_LINE as u32) - 1;
		// Mark both buffers as ready for playing out, so the DMA gets going
		// before Core 1 starts rendering. You get a couple of garbage lines at
		// the start, but that's OK.
		PIXEL_DATA_BUFFER_EVEN.mark_rendering_done();
		PIXEL_DATA_BUFFER_ODD.mark_rendering_done();
	}

	// Enable the DMA
	dma.multi_chan_trigger
		.write(|w| unsafe { w.bits((1 << PIXEL_DMA_CHAN) | (1 << TIMING_DMA_CHAN)) });

	debug!("DMA enabled");

	unsafe {
		// Hand off the DMA peripheral to the interrupt
		DMA_PERIPH = Some(dma);

		// Enable the interrupts (DMA_PERIPH has to be set first)
		cortex_m::interrupt::enable();
		crate::pac::NVIC::unpend(crate::pac::Interrupt::DMA_IRQ_0);
		crate::pac::NVIC::unmask(crate::pac::Interrupt::DMA_IRQ_0);
	}

	debug!("IRQs enabled");

	debug!("DMA set-up complete");

	// We drop our state-machine and PIO objects here - this means the video
	// cannot be reconfigured at a later time, but they do keep on running
	// as-is.

	let core1_stack: &'static mut [usize] = unsafe {
		extern "C" {
			static mut _core1_stack_bottom: usize;
			static mut _core1_stack_len: usize;
		}
		core::slice::from_raw_parts_mut(
			&mut _core1_stack_bottom as *mut _,
			&mut _core1_stack_len as *const _ as usize / 4,
		)
	};

	debug!(
		"Core 1 stack: {:08x}, {} bytes",
		core1_stack.as_ptr(),
		core1_stack.len()
	);

	multicore_launch_core1_with_stack(core1_main, core1_stack, ppb, fifo, psm);

	debug!("Core 1 running");

	timing_sm.start();
	pixel_sm.start();

	debug!("State Machines running");
}

/// The bootrom code will call this function on core1 to perform any set-up, before the
/// entry function is called.
extern "C" fn core1_wrapper(entry_func: extern "C" fn() -> u32, _stack_base: *mut u32) -> u32 {
	entry_func()
}

/// Starts core 1 running the given function, with the given stack.
fn multicore_launch_core1_with_stack(
	main_func: unsafe extern "C" fn() -> u32,
	stack: &mut [usize],
	ppb: &mut crate::pac::PPB,
	fifo: &mut rp_pico::hal::sio::SioFifo,
	psm: &mut crate::pac::PSM,
) {
	debug!("Resetting CPU1...");

	psm.frce_off.modify(|_, w| w.proc1().set_bit());
	while !psm.frce_off.read().proc1().bit_is_set() {
		cortex_m::asm::nop();
	}
	psm.frce_off.modify(|_, w| w.proc1().clear_bit());

	debug!("Setting up stack...");

	// Gets popped into `r0` by CORE1_ENTRY_FUNCTION. This is the `main`
	// function we want to run. It appears in the call to `core1_wrapper` as
	// the first argument.
	stack[stack.len() - 3] = main_func as *const () as usize;
	// Gets popped into `r1` by CORE1_ENTRY_FUNCTION. This is the top of stack
	// for Core 1. It appears in the call to `core1_wrapper` as the second
	// argument.
	stack[stack.len() - 2] = stack.as_ptr() as *const _ as usize;
	// Gets popped into `pc` by CORE1_ENTRY_FUNCTION. This is the function
	// `CORE1_ENTRY_FUNCTION` will jump to, passing the above two values as
	// arguments.
	stack[stack.len() - 1] = core1_wrapper as *const () as usize;
	// Point into the top of the stack (so there are three values pushed onto
	// it, i.e. at/above it)
	let stack_ptr = unsafe { stack.as_mut_ptr().add(stack.len() - 3) };

	debug!("Stack ptr is 0x{:x}", stack_ptr);
	debug!("Stack bottom is 0x{:x}", stack.as_ptr());
	debug!("Stack top is 0x{:x}", &stack[stack.len() - 4..stack.len()]);

	// This is the launch sequence we send to core1, to get it to leave the
	// boot ROM and run our code.
	let cmd_sequence: [u32; 6] = [
		0,
		0,
		1,
		ppb.vtor.read().bits() as usize as u32,
		stack_ptr as usize as u32,
		// Have to add 1 to convert from an array pointer to a thumb instruction pointer
		(CORE1_ENTRY_FUNCTION.as_ptr() as usize as u32) + 1,
	];

	let enabled = crate::pac::NVIC::is_enabled(crate::pac::Interrupt::SIO_IRQ_PROC0);
	crate::pac::NVIC::mask(crate::pac::Interrupt::SIO_IRQ_PROC0);

	'outer: loop {
		for cmd in cmd_sequence.iter() {
			debug!("Sending command {:x}...", *cmd);

			// we drain before sending a 0
			if *cmd == 0 {
				debug!("Draining FIFO...");
				fifo.drain();
				// core 1 may be waiting for fifo space
				cortex_m::asm::sev();
			}
			debug!("Pushing to FIFO...");
			fifo.write_blocking(*cmd);

			debug!("Getting response from FIFO...");
			let response = loop {
				if let Some(x) = fifo.read() {
					break x;
				} else {
					debug!("ST is {:x}", fifo.status());
				}
			};

			// move to next state on correct response otherwise start over
			debug!("Got {:x}", response);
			if *cmd != response {
				continue 'outer;
			}
		}
		break;
	}

	if enabled {
		unsafe { crate::pac::NVIC::unmask(crate::pac::Interrupt::SIO_IRQ_PROC0) };
	}

	debug!("Waiting for Core 1 to start...");
	while !CORE1_START_FLAG.load(Ordering::Relaxed) {
		cortex_m::asm::nop();
	}
	debug!("Core 1 started!!");
}

/// Gets the current video mode
pub fn get_video_mode() -> crate::common::video::Mode {
	unsafe { VIDEO_MODE }
}

/// Sets the current video mode
pub fn set_video_mode(mode: crate::common::video::Mode) -> bool {
	cortex_m::interrupt::disable();
	let mode_ok = match (
		mode.timing(),
		mode.format(),
		mode.is_horiz_2x(),
		mode.is_vert_2x(),
	) {
		(
			crate::common::video::Timing::T640x480,
			crate::common::video::Format::Text8x16 | crate::common::video::Format::Text8x8,
			false,
			false,
		) => {
			unsafe {
				VIDEO_MODE = mode;
				TIMING_BUFFER = TimingBuffer::make_640x480();
			}
			true
		}
		(
			crate::common::video::Timing::T640x400,
			crate::common::video::Format::Text8x16 | crate::common::video::Format::Text8x8,
			false,
			false,
		) => {
			unsafe {
				VIDEO_MODE = mode;
				TIMING_BUFFER = TimingBuffer::make_640x400();
			}
			true
		}
		_ => false,
	};
	if mode_ok {
		NUM_TEXT_COLS.store(mode.text_width().unwrap_or(0) as usize, Ordering::Relaxed);
		NUM_TEXT_ROWS.store(mode.text_height().unwrap_or(0) as usize, Ordering::Relaxed);
	}
	unsafe {
		cortex_m::interrupt::enable();
	}
	mode_ok
}

/// Get the current scan line.
pub fn get_scan_line() -> u16 {
	CURRENT_PLAYOUT_LINE.load(Ordering::Relaxed)
}

/// Get how many visible lines there currently are
pub fn get_num_scan_lines() -> u16 {
	let mode = get_video_mode();
	mode.vertical_lines()
}

/// This function runs the video processing loop on Core 1.
///
/// It keeps the odd/even scan-line buffers updated, as per the contents of
/// the text buffer.
///
/// # Safety
///
/// Only run this function on Core 1.
#[link_section = ".data"]
unsafe extern "C" fn core1_main() -> u32 {
	CORE1_START_FLAG.store(true, Ordering::Relaxed);

	loop {
		let mut waited: u32 = 0;
		waited += render_scanline(&mut PIXEL_DATA_BUFFER_ODD);
		waited += render_scanline(&mut PIXEL_DATA_BUFFER_EVEN);
		RENDER_WAITS_COUNT.store(
			RENDER_WAITS_COUNT.load(Ordering::Relaxed) + waited / 2,
			Ordering::Relaxed,
		);
	}
}

/// Call this function whenever the DMA reports that it has completed a transfer.
///
/// We use this as a prompt to either start a transfer or more Timing words,
/// or a transfer or more pixel words.
///
/// # Safety
///
/// Only call this from the DMA IRQ handler.
#[link_section = ".data"]
#[inline(always)]
pub unsafe fn irq() {
	let dma: &mut super::pac::DMA = match DMA_PERIPH.as_mut() {
		Some(dma) => dma,
		None => {
			return;
		}
	};
	let status = dma.ints0.read().bits();

	// Check if this is a DMA interrupt for the sync DMA channel
	let timing_dma_chan_irq = (status & (1 << TIMING_DMA_CHAN)) != 0;

	// Check if this is a DMA interrupt for the line DMA channel
	let pixel_dma_chan_irq = (status & (1 << PIXEL_DMA_CHAN)) != 0;

	if timing_dma_chan_irq {
		// clear timing_dma_chan bit in DMA interrupt bitfield
		dma.ints0.write(|w| w.bits(1 << TIMING_DMA_CHAN));

		let old_timing_line = CURRENT_TIMING_LINE.load(Ordering::Relaxed);
		let next_timing_line = if old_timing_line == TIMING_BUFFER.back_porch_ends_at {
			// Wrap around
			0
		} else {
			// Keep going
			old_timing_line + 1
		};
		CURRENT_TIMING_LINE.store(next_timing_line, Ordering::Relaxed);

		let buffer = if next_timing_line <= TIMING_BUFFER.visible_lines_ends_at {
			// Visible lines
			&TIMING_BUFFER.visible_line
		} else if next_timing_line <= TIMING_BUFFER.front_porch_end_at {
			// VGA front porch before VGA sync pulse
			&TIMING_BUFFER.vblank_porch_buffer
		} else if next_timing_line <= TIMING_BUFFER.sync_pulse_ends_at {
			// Sync pulse
			&TIMING_BUFFER.vblank_sync_buffer
		} else {
			// VGA back porch following VGA sync pulse
			&TIMING_BUFFER.vblank_porch_buffer
		};
		dma.ch[TIMING_DMA_CHAN]
			.ch_al3_read_addr_trig
			.write(|w| w.bits(buffer as *const _ as usize as u32))
	}

	if pixel_dma_chan_irq {
		dma.ints0.write(|w| w.bits(1 << PIXEL_DMA_CHAN));

		// A pixel DMA transfer is now complete. This only fires on visible
		// lines. We now need to queue the next DMA transfer.

		let last_playout_line = CURRENT_PLAYOUT_LINE.load(Ordering::Relaxed);

		let next_playout_line = if last_playout_line < TIMING_BUFFER.visible_lines_ends_at {
			last_playout_line + 1
		} else {
			0
		};

		let next_draw_line = if next_playout_line < TIMING_BUFFER.visible_lines_ends_at {
			next_playout_line + 1
		} else {
			0
		};

		static TOTAL_LINES: AtomicU32 = AtomicU32::new(0);
		TOTAL_LINES.store(TOTAL_LINES.load(Ordering::Relaxed) + 1, Ordering::Relaxed);

		// Set the DMA load address according to which line we are on. We use
		// the 'trigger' alias to restart the DMA at the same time as we write
		// the new read address. The DMA had stopped because the previous line
		// was transferred completely. The DMA will continue as and when the pixel
		// PIO FIFO needs more data.
		if (last_playout_line & 1) == 0 {
			// Is the one we're about to play out fully rendered?
			if !PIXEL_DATA_BUFFER_ODD.is_rendering_done() {
				// Can't playout line that's still being rendered
				CANT_PLAY_COUNT.store(
					CANT_PLAY_COUNT.load(Ordering::Relaxed) + 1,
					Ordering::Relaxed,
				);
			}
			// Queue the odd buffer for playout
			dma.ch[PIXEL_DMA_CHAN]
				.ch_al3_read_addr_trig
				.write(|w| w.bits(PIXEL_DATA_BUFFER_ODD.as_ptr()));
			// Just played an even line, so the even buffer is now ready for more rendering.
			PIXEL_DATA_BUFFER_EVEN.set_ready(next_draw_line);
		} else {
			// Is the one we're about to play out fully rendered?
			if !PIXEL_DATA_BUFFER_EVEN.is_rendering_done() {
				// Can't playout line that's still being rendered
				CANT_PLAY_COUNT.store(
					CANT_PLAY_COUNT.load(Ordering::Relaxed) + 1,
					Ordering::Relaxed,
				);
			}
			// Queue the even buffer for playout
			dma.ch[PIXEL_DMA_CHAN]
				.ch_al3_read_addr_trig
				.write(|w| w.bits(PIXEL_DATA_BUFFER_EVEN.as_ptr()));
			// Just played an odd line, so the odd buffer is now ready for more rendering.
			PIXEL_DATA_BUFFER_ODD.set_ready(next_draw_line);
		}

		CURRENT_PLAYOUT_LINE.store(next_playout_line, Ordering::Relaxed);
	}
}

/// Performs the VGA rendering.
#[link_section = ".data"]
fn render_scanline(scan_line_buffer: &mut LineBuffer) -> u32 {
	let font = match unsafe { VIDEO_MODE.format() } {
		crate::common::video::Format::Text8x16 => &font16::FONT,
		crate::common::video::Format::Text8x8 => &font8::FONT,
		_ => {
			return 0;
		}
	};

	let num_rows = NUM_TEXT_ROWS.load(Ordering::Relaxed);
	let num_cols = NUM_TEXT_COLS.load(Ordering::Relaxed);

	let mut count: u32 = 0;
	while !scan_line_buffer.is_ready_for_rendering() {
		// Wait for this buffer to be ready for us
		count = count.wrapping_add(1);
	}

	// Which line do we want?
	let current_line_num = scan_line_buffer.line_number.load(Ordering::SeqCst);

	// Convert our position in scan-lines to a text row, and a line within each glyph on that row
	let text_row = current_line_num as usize / font.height;
	let font_row = current_line_num as usize % font.height;

	if text_row < num_rows {
		// Note (unsafe): accessing a static mut, but we do it via a const ptr.
		let row_start: *const GlyphAttr =
			unsafe { GLYPH_ATTR_ARRAY.as_ptr().add(text_row * num_cols) };

		// Get a pointer into our scan-line buffer
		let scan_line_buffer_ptr = scan_line_buffer.pixels.as_mut_ptr();

		// Every font look-up we are about to do for this row will
		// involve offsetting by the row within each glyph. As this
		// is the same for every glyph on this row, we calculate a
		// new pointer once, in advance, and save ourselves an
		// addition each time around the loop.
		let font_ptr = unsafe { font.data.as_ptr().add(font_row) };

		match num_cols {
			80 => render80cols(row_start, font_ptr, font.height, scan_line_buffer_ptr),
			40 => render40cols(row_start, font_ptr, font.height, scan_line_buffer_ptr),
			_ => {
				// Do nothing
			}
		}
	}

	scan_line_buffer.mark_rendering_done();

	count
}

/// Render one line of 80-column text mode
///
/// We bring this out into a function as making the for loop have a fixed range
/// greatly speeds up the generated code.
fn render80cols(
	row_start: *const GlyphAttr,
	font_ptr: *const u8,
	font_height: usize,
	scan_line_buffer_ptr: *mut RGBColour,
) {
	let mut pixel_offset = 0;

	// Convert from characters to coloured pixels, using the font as a look-up table.
	for col in 0..80 {
		// Get the 16-bit glyph/attribute pair
		let glyphattr = unsafe { core::ptr::read(row_start.add(col)) };
		// Grab just the attribute
		let attr = glyphattr.attr();
		// Note (unsafe): We use pointer arithmetic here because we
		// can't afford a bounds-check on an array. This is safe
		// because the fg attribute is a maximum of 15, and the
		// palette is 256 entries long.
		let fg_idx = attr.fg();
		let fg_rgb = unsafe { core::ptr::read(VIDEO_PALETTE.as_ptr().add(fg_idx.0 as usize)) };
		// Note (unsafe): We use pointer arithmetic here because we
		// can't afford a bounds-check on an array. This is safe
		// because the bg attribute is a maximum of 7, and the
		// palette is 256 entries long.
		let bg_idx = attr.bg();
		let bg_rgb = unsafe { core::ptr::read(VIDEO_PALETTE.as_ptr().add(bg_idx.0 as usize)) };
		// Note (unsafe): We use pointer arithmetic here because we
		// can't afford a bounds-check on an array. This is safe
		// because the font is `256 * width` bytes long and we can't
		// index more than `255 * width` bytes into it.
		let glyph_index = (glyphattr.glyph().0 as usize) * font_height;
		let mono_pixels = unsafe { core::ptr::read(font_ptr.add(glyph_index)) };

		let pixel = if (mono_pixels & 0x80) != 0 {
			fg_rgb
		} else {
			bg_rgb
		};
		unsafe {
			core::ptr::write(scan_line_buffer_ptr.offset(pixel_offset), pixel);
		}

		let pixel = if (mono_pixels & 0x40) != 0 {
			fg_rgb
		} else {
			bg_rgb
		};
		unsafe {
			core::ptr::write(scan_line_buffer_ptr.offset(pixel_offset + 1), pixel);
		}

		let pixel = if (mono_pixels & 0x20) != 0 {
			fg_rgb
		} else {
			bg_rgb
		};
		unsafe {
			core::ptr::write(scan_line_buffer_ptr.offset(pixel_offset + 2), pixel);
		}

		let pixel = if (mono_pixels & 0x10) != 0 {
			fg_rgb
		} else {
			bg_rgb
		};
		unsafe {
			core::ptr::write(scan_line_buffer_ptr.offset(pixel_offset + 3), pixel);
		}

		let pixel = if (mono_pixels & 0x08) != 0 {
			fg_rgb
		} else {
			bg_rgb
		};
		unsafe {
			core::ptr::write(scan_line_buffer_ptr.offset(pixel_offset + 4), pixel);
		}

		let pixel = if (mono_pixels & 0x04) != 0 {
			fg_rgb
		} else {
			bg_rgb
		};
		unsafe {
			core::ptr::write(scan_line_buffer_ptr.offset(pixel_offset + 5), pixel);
		}

		let pixel = if (mono_pixels & 0x02) != 0 {
			fg_rgb
		} else {
			bg_rgb
		};
		unsafe {
			core::ptr::write(scan_line_buffer_ptr.offset(pixel_offset + 6), pixel);
		}

		let pixel = if (mono_pixels & 0x01) != 0 {
			fg_rgb
		} else {
			bg_rgb
		};
		unsafe {
			core::ptr::write(scan_line_buffer_ptr.offset(pixel_offset + 7), pixel);
		}

		pixel_offset += 8;
	}
}

/// Render one line of 80-column text mode
///
/// We bring this out into a function as making the for loop have a fixed range
/// greatly speeds up the generated code.
fn render40cols(
	row_start: *const GlyphAttr,
	font_ptr: *const u8,
	font_height: usize,
	scan_line_buffer_ptr: *mut RGBColour,
) {
	let mut pixel_offset = 0;

	// Convert from characters to coloured pixels, using the font as a look-up table.
	for col in 0..40 {
		// Get the 16-bit glyph/attribute pair
		let glyphattr = unsafe { core::ptr::read(row_start.add(col)) };
		// Grab just the attribute
		let attr = glyphattr.attr();
		// Note (unsafe): We use pointer arithmetic here because we
		// can't afford a bounds-check on an array. This is safe
		// because the fg attribute is a maximum of 15, and the
		// palette is 256 entries long.
		let fg_idx = attr.fg();
		let fg_rgb = unsafe { core::ptr::read(VIDEO_PALETTE.as_ptr().add(fg_idx.0 as usize)) };
		// Note (unsafe): We use pointer arithmetic here because we
		// can't afford a bounds-check on an array. This is safe
		// because the bg attribute is a maximum of 7, and the
		// palette is 256 entries long.
		let bg_idx = attr.bg();
		let bg_rgb = unsafe { core::ptr::read(VIDEO_PALETTE.as_ptr().add(bg_idx.0 as usize)) };
		// Note (unsafe): We use pointer arithmetic here because we
		// can't afford a bounds-check on an array. This is safe
		// because the font is `256 * width` bytes long and we can't
		// index more than `255 * width` bytes into it.
		let glyph_index = (glyphattr.glyph().0 as usize) * font_height;
		let mono_pixels = unsafe { core::ptr::read(font_ptr.add(glyph_index)) };

		let pixel = if (mono_pixels & 0x80) != 0 {
			fg_rgb
		} else {
			bg_rgb
		};
		unsafe {
			core::ptr::write(scan_line_buffer_ptr.offset(pixel_offset), pixel);
		}

		let pixel = if (mono_pixels & 0x40) != 0 {
			fg_rgb
		} else {
			bg_rgb
		};
		unsafe {
			core::ptr::write(scan_line_buffer_ptr.offset(pixel_offset + 1), pixel);
		}

		let pixel = if (mono_pixels & 0x20) != 0 {
			fg_rgb
		} else {
			bg_rgb
		};
		unsafe {
			core::ptr::write(scan_line_buffer_ptr.offset(pixel_offset + 2), pixel);
		}

		let pixel = if (mono_pixels & 0x10) != 0 {
			fg_rgb
		} else {
			bg_rgb
		};
		unsafe {
			core::ptr::write(scan_line_buffer_ptr.offset(pixel_offset + 3), pixel);
		}

		let pixel = if (mono_pixels & 0x08) != 0 {
			fg_rgb
		} else {
			bg_rgb
		};
		unsafe {
			core::ptr::write(scan_line_buffer_ptr.offset(pixel_offset + 4), pixel);
		}

		let pixel = if (mono_pixels & 0x04) != 0 {
			fg_rgb
		} else {
			bg_rgb
		};
		unsafe {
			core::ptr::write(scan_line_buffer_ptr.offset(pixel_offset + 5), pixel);
		}

		let pixel = if (mono_pixels & 0x02) != 0 {
			fg_rgb
		} else {
			bg_rgb
		};
		unsafe {
			core::ptr::write(scan_line_buffer_ptr.offset(pixel_offset + 6), pixel);
		}

		let pixel = if (mono_pixels & 0x01) != 0 {
			fg_rgb
		} else {
			bg_rgb
		};
		unsafe {
			core::ptr::write(scan_line_buffer_ptr.offset(pixel_offset + 7), pixel);
		}

		pixel_offset += 8;
	}
}

impl TextConsole {
	/// Create a TextConsole.
	///
	/// Has no buffer associated with it
	pub const fn new() -> TextConsole {
		TextConsole {
			current_row: AtomicU8::new(0),
			current_col: AtomicU8::new(0),
			text_buffer: AtomicPtr::new(core::ptr::null_mut()),
			attr: AtomicU8::new(
				Attr::new(TextForegroundColour(15), TextBackgroundColour(0), false).as_u8(),
			),
		}
	}

	/// Update the text buffer we are using.
	///
	/// Will reset the cursor. The screen is not cleared.
	pub fn set_text_buffer(
		&self,
		text_buffer: &'static mut [GlyphAttr; MAX_TEXT_ROWS * MAX_TEXT_COLS],
	) {
		self.text_buffer
			.store(text_buffer.as_mut_ptr(), Ordering::Relaxed)
	}

	/// Set the attribute value used for writing out text.
	pub fn set_attribute(&self, attr: Attr) {
		self.attr.store(attr.as_u8(), Ordering::Relaxed);
	}

	/// Moves the text cursor to the specified row and column.
	///
	/// If a value is out of bounds, the cursor is not moved in that axis.
	pub fn move_to(&self, row: u8, col: u8) {
		if (row as usize) < NUM_TEXT_ROWS.load(Ordering::Relaxed) {
			self.current_row.store(row, Ordering::Relaxed);
		}
		if (col as usize) < NUM_TEXT_COLS.load(Ordering::Relaxed) {
			self.current_col.store(col, Ordering::Relaxed);
		}
	}

	/// Convert a Unicode Scalar Value to a font glyph.
	///
	/// Zero-width and modifier Unicode Scalar Values (e.g. `U+0301 COMBINING,
	/// ACCENT`) are not supported. Normalise your Unicode before calling
	/// this function.
	pub fn map_char_to_glyph(input: char) -> Option<Glyph> {
		// Only support 7-bit US-ASCII in the BIOS console.
		if input as u32 <= 127 {
			Some(Glyph(input as u8))
		} else {
			None
		}
	}

	/// Put a single glyph at a specified point on screen.
	///
	/// The glyph is an index into the the current font.
	fn write_at(
		&self,
		glyphattr: GlyphAttr,
		buffer: *mut GlyphAttr,
		row: u8,
		col: u8,
		num_cols: usize,
	) {
		let offset = (col as usize) + (num_cols * (row as usize));
		// Note (safety): This is safe as we bound `col` and `row`
		unsafe { buffer.add(offset).write_volatile(glyphattr) };
	}
}

unsafe impl Sync for TextConsole {}

impl core::fmt::Write for &TextConsole {
	/// Allows us to call `writeln!(some_text_console, "hello")`
	fn write_str(&mut self, s: &str) -> core::fmt::Result {
		// Load from global state
		let mut row = self.current_row.load(Ordering::Relaxed);
		let mut col = self.current_col.load(Ordering::Relaxed);
		let num_cols = NUM_TEXT_COLS.load(Ordering::Relaxed);
		let num_rows = NUM_TEXT_ROWS.load(Ordering::Relaxed);
		let attr = Attr(self.attr.load(Ordering::Relaxed));
		let buffer = self.text_buffer.load(Ordering::Relaxed);

		if !buffer.is_null() {
			for ch in s.chars() {
				match ch {
					'\n' => {
						// New Line (with implicit carriage return, like UNIX)
						row = row + 1;
						col = 0;
					}
					'\r' => {
						// Carriage Return
						col = 0;
					}
					_ => {
						let glyph = TextConsole::map_char_to_glyph(ch).unwrap_or(Glyph(b'?'));
						let glyphattr = GlyphAttr::new(glyph, attr);
						self.write_at(glyphattr, buffer, row, col, num_cols);
						col = col + 1;
					}
				}
				if col == (num_cols as u8) {
					col = 0;
					row += 1;
				}
				if row == (num_rows as u8) {
					// Stay on last line
					row = (num_rows - 1) as u8;
					// Scroll everything
					unsafe {
						core::ptr::copy(
							buffer.add(num_cols as usize),
							buffer,
							num_cols * (num_rows - 1),
						)
					};
					// Wipe the last line
					for blank_col in 0..num_cols {
						let offset = (blank_col as usize) + (num_cols * (row as usize));
						unsafe {
							buffer
								.add(offset)
								.write_volatile(GlyphAttr::new(Glyph(b' '), Attr(0)))
						};
					}
				}
			}

			// Push back to global state
			self.current_row.store(row as u8, Ordering::Relaxed);
			self.current_col.store(col as u8, Ordering::Relaxed);
		}

		Ok(())
	}
}

impl LineBuffer {
	/// Convert the line buffer to a 32-bit address that the DMA engine understands.
	fn as_ptr(&self) -> u32 {
		self as *const _ as usize as u32
	}

	/// Mark that this buffer is ready to be rendered into.
	fn set_ready(&self, line_number: u16) {
		self.line_number.store(line_number, Ordering::SeqCst);
		self.ready_for_drawing.store(true, Ordering::SeqCst);
	}

	/// Mark that this buffer has been rendered into and is ready for playout.
	fn mark_rendering_done(&self) {
		self.ready_for_drawing.store(false, Ordering::SeqCst);
	}

	/// Report whether DMA is using this line
	fn is_ready_for_rendering(&self) -> bool {
		self.ready_for_drawing.load(Ordering::Relaxed)
	}

	/// Report whether rendering has finished
	fn is_rendering_done(&self) -> bool {
		!self.is_ready_for_rendering()
	}
}

impl SyncPolarity {
	const fn enabled(&self) -> bool {
		match self {
			SyncPolarity::Positive => true,
			SyncPolarity::Negative => false,
		}
	}

	const fn disabled(&self) -> bool {
		match self {
			SyncPolarity::Positive => false,
			SyncPolarity::Negative => true,
		}
	}
}

impl ScanlineTimingBuffer {
	/// Create a timing buffer for each scan-line in the V-Sync visible portion.
	///
	/// The timings are in the order (front-porch, sync, back-porch, visible) and are in pixel clocks.
	const fn new_v_visible(
		hsync: SyncPolarity,
		vsync: SyncPolarity,
		timings: (u32, u32, u32, u32),
	) -> ScanlineTimingBuffer {
		ScanlineTimingBuffer {
			data: [
				// Front porch (as per the spec)
				Self::make_timing(timings.0 * 10, hsync.disabled(), vsync.disabled(), false),
				// Sync pulse (as per the spec)
				Self::make_timing(timings.1 * 10, hsync.enabled(), vsync.disabled(), false),
				// Back porch. Adjusted by a few clocks to account for interrupt +
				// PIO SM start latency.
				Self::make_timing(
					(timings.2 * 10) - 5,
					hsync.disabled(),
					vsync.disabled(),
					false,
				),
				// Visible portion. It also triggers the IRQ to start pixels
				// moving. Adjusted to compensate for changes made to previous
				// period to ensure scan-line remains at correct length.
				Self::make_timing(
					(timings.3 * 10) + 5,
					hsync.disabled(),
					vsync.disabled(),
					true,
				),
			],
		}
	}

	/// Create a timing buffer for each scan-line in the V-Sync front-porch and back-porch
	const fn new_v_porch(
		hsync: SyncPolarity,
		vsync: SyncPolarity,
		timings: (u32, u32, u32, u32),
	) -> ScanlineTimingBuffer {
		ScanlineTimingBuffer {
			data: [
				// Front porch (as per the spec)
				Self::make_timing(timings.0 * 10, hsync.disabled(), vsync.disabled(), false),
				// Sync pulse (as per the spec)
				Self::make_timing(timings.1 * 10, hsync.enabled(), vsync.disabled(), false),
				// Back porch.
				Self::make_timing(timings.2 * 10, hsync.disabled(), vsync.disabled(), false),
				// Visible portion.
				Self::make_timing(timings.3 * 10, hsync.disabled(), vsync.disabled(), false),
			],
		}
	}

	/// Create a timing buffer for each scan-line in the V-Sync pulse
	const fn new_v_pulse(
		hsync: SyncPolarity,
		vsync: SyncPolarity,
		timings: (u32, u32, u32, u32),
	) -> ScanlineTimingBuffer {
		ScanlineTimingBuffer {
			data: [
				// Front porch (as per the spec)
				Self::make_timing(timings.0 * 10, hsync.disabled(), vsync.enabled(), false),
				// Sync pulse (as per the spec)
				Self::make_timing(timings.1 * 10, hsync.enabled(), vsync.enabled(), false),
				// Back porch.
				Self::make_timing(timings.2 * 10, hsync.disabled(), vsync.enabled(), false),
				// Visible portion.
				Self::make_timing(timings.3 * 10, hsync.disabled(), vsync.enabled(), false),
			],
		}
	}

	/// Generate a 32-bit value we can send to the Timing FIFO.
	///
	/// * `period` - The length of this portion of the scan-line, in system clock ticks
	/// * `hsync` - true if the H-Sync pin should be high during this period, else false
	/// * `vsync` - true if the H-Sync pin should be high during this period, else false
	/// * `raise_irq` - true the timing statemachine should raise an IRQ at the start of this period
	///
	/// Returns a 32-bit value you can post to the Timing FIFO.
	const fn make_timing(period: u32, hsync: bool, vsync: bool, raise_irq: bool) -> u32 {
		let command = if raise_irq {
			// This command sets IRQ 0. It is the same as:
			//
			// ```
			// pio::InstructionOperands::IRQ {
			// 	clear: false,
			// 	wait: false,
			// 	index: 0,
			// 	relative: false,
			// }.encode()
			// ```
			//
			// Unfortunately encoding this isn't a const-fn, so we cheat:
			0xc000
		} else {
			// This command is a no-op (it moves Y into Y)
			//
			// ```
			// pio::InstructionOperands::MOV {
			// 	destination: pio::MovDestination::Y,
			// 	op: pio::MovOperation::None,
			// 	source: pio::MovSource::Y,
			// }.encode()
			// ```
			//
			// Unfortunately encoding this isn't a const-fn, so we cheat:
			0xa042
		};
		let mut value: u32 = 0;
		if hsync {
			value |= 1 << 0;
		}
		if vsync {
			value |= 1 << 1;
		}
		value |= (period - 6) << 2;
		value | command << 16
	}
}

impl TimingBuffer {
	/// Make a timing buffer suitable for 640 x 400 @ 70 Hz
	pub const fn make_640x400() -> TimingBuffer {
		TimingBuffer {
			visible_line: ScanlineTimingBuffer::new_v_visible(
				SyncPolarity::Negative,
				SyncPolarity::Positive,
				(16, 96, 48, 640),
			),
			vblank_porch_buffer: ScanlineTimingBuffer::new_v_porch(
				SyncPolarity::Negative,
				SyncPolarity::Positive,
				(16, 96, 48, 640),
			),
			vblank_sync_buffer: ScanlineTimingBuffer::new_v_pulse(
				SyncPolarity::Negative,
				SyncPolarity::Positive,
				(16, 96, 48, 640),
			),
			visible_lines_ends_at: 399,
			front_porch_end_at: 399 + 12,
			sync_pulse_ends_at: 399 + 12 + 2,
			back_porch_ends_at: 399 + 12 + 2 + 35,
		}
	}

	/// Make a timing buffer suitable for 640 x 480 @ 60 Hz
	pub const fn make_640x480() -> TimingBuffer {
		TimingBuffer {
			visible_line: ScanlineTimingBuffer::new_v_visible(
				SyncPolarity::Negative,
				SyncPolarity::Negative,
				(16, 96, 48, 640),
			),
			vblank_porch_buffer: ScanlineTimingBuffer::new_v_porch(
				SyncPolarity::Negative,
				SyncPolarity::Negative,
				(16, 96, 48, 640),
			),
			vblank_sync_buffer: ScanlineTimingBuffer::new_v_pulse(
				SyncPolarity::Negative,
				SyncPolarity::Negative,
				(16, 96, 48, 640),
			),
			visible_lines_ends_at: 479,
			front_porch_end_at: 479 + 10,
			sync_pulse_ends_at: 479 + 10 + 2,
			back_porch_ends_at: 479 + 10 + 2 + 33,
		}
	}
}

impl RGBColour {
	/// Make a 12-bit RGB Colour from 8-bit red, green and blue values.
	pub const fn new(red: u8, green: u8, blue: u8) -> RGBColour {
		let red = (red >> 4) as u16;
		let green = (green >> 4) as u16;
		let blue = (blue >> 4) as u16;
		RGBColour((blue << 8) | (green << 4) | red)
	}
}

impl RGBPair {
	pub const fn new(first: RGBColour, second: RGBColour) -> RGBPair {
		let first: u32 = first.0 as u32;
		let second: u32 = second.0 as u32;
		RGBPair((second << 16) | first)
	}
}

// -----------------------------------------------------------------------------
// End of file
// -----------------------------------------------------------------------------
