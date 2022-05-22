//! # VGA Driver for the Neotron Pico
//!
//! VGA output on the Neotron Pico uses 14 GPIO pins and two PIO state machines.
//!
//! It can generate 640x480@60Hz and 640x400@70Hz standard VGA video, with a
//! 25.2 MHz pixel clock. The spec is 25.175 MHz, so we are 0.1% off). The
//! assumption is that the CPU is clocked at 252 MHz, i.e. 10x the pixel
//! clock. All of the PIO code relies on this assumption!

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

/// A font.
///
/// A font is always 8 pixels wide, and a power of 2 high. It always has 256
/// glyphs in it.
pub struct Font<'a> {
	/// The height of the font is `1 << height_shift`. Makes division faster,
	/// but it means you can't have a 14px high font.
	height_shift: u8,
	/// The font pixels. They are arranged by row, so all the row 0s come first, then all the row 1s, etc.
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
	pixels: [RGBPair; MAX_NUM_PIXEL_PAIRS_PER_LINE],
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

/// Caches the conversion of two mono pixels into an RGB pixel pair, coloured
/// with the desired foreground and background colours.
struct TextColourLookup {
	entries: [RGBPair; 512],
}

/// Represents a 12-bit colour value.
///
/// Each channel has four-bits, and they are packed in `GBR` format. This is
/// so the PIO can shift them out right-first, and we have RED0 assigned to
/// the lowest GPIO pin.
#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Eq, Default)]
pub struct RGBColour(u16);

/// Represents two `RGBColour` pixels packed together.
///
/// The `first` pixel is packed in the lower 16-bits. This is because the PIO
/// shifts-right.
#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Eq, Default)]
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
pub static NUM_TEXT_ROWS: AtomicUsize = AtomicUsize::new(30);

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
	pixels: [RGBPair(0); MAX_NUM_PIXEL_PAIRS_PER_LINE],
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
	pixels: [RGBPair(0); MAX_NUM_PIXEL_PAIRS_PER_LINE],
	ready_for_drawing: AtomicBool::new(false),
	line_number: AtomicU16::new(0),
};

/// A count of how many lines were queued for DMA before rendering complete.
/// This may not result in a glitch, if the rendering completes just ahead of
/// the beam.
pub static CLASHED_COUNT: AtomicU32 = AtomicU32::new(0);

/// A record of how many clock cycles were spent in the rendering code on Core 1.
pub static RENDER_TIME: AtomicU32 = AtomicU32::new(0);

/// Holds the colour look-up table for text mode.
///
/// The input is a 9-bit vlaue comprised of the 4-bit foreground colour index,
/// the 3-bit background colour index, and a two mono pixels. The output is a
/// 32-bit RGB Colour Pair, containing two RGB pixels.
///
/// ```
/// +-----+-----+-----+-----+-----+-----+-----+-----+-----+
/// | FG3 | FG2 | FG1 | FG0 | BG2 | BG1 | BG0 | PX1 | PX0 |
/// +-----+-----+-----+-----+-----+-----+-----+-----+-----+
/// ```
static mut TEXT_COLOUR_LOOKUP: TextColourLookup = TextColourLookup::blank();

/// Holds the 256-entry palette for indexed colour modes.
static mut VIDEO_PALETTE: [RGBColour; 256] = [
	// Index 000: 0x000 (Black)
	RGBColour::new4(0x0, 0x0, 0x0),
	// Index 001: 0x800 (Dark Red)
	RGBColour::new4(0x8, 0x0, 0x0),
	// Index 002: 0x080 (Dark Green)
	RGBColour::new4(0x0, 0x8, 0x0),
	// Index 003: 0x880 (Orange)
	RGBColour::new4(0x8, 0x8, 0x0),
	// Index 004: 0x008 (Blue)
	RGBColour::new4(0x0, 0x0, 0x8),
	// Index 005: 0x808 (Dark Magenta)
	RGBColour::new4(0x8, 0x0, 0x8),
	// Index 006: 0x088 (Dark Cyan)
	RGBColour::new4(0x0, 0x8, 0x8),
	// Index 007: 0xcc0 (Yellow)
	RGBColour::new4(0xc, 0xc, 0x0),
	// Index 008: 0x888 (Grey)
	RGBColour::new4(0x8, 0x8, 0x8),
	// Index 009: 0xf00 (Bright Red)
	RGBColour::new4(0xf, 0x0, 0x0),
	// Index 010: 0x0f0 (Bright Green)
	RGBColour::new4(0x0, 0xf, 0x0),
	// Index 011: 0xff0 (Bright Yellow)
	RGBColour::new4(0xf, 0xf, 0x0),
	// Index 012: 0x00f (Bright Blue)
	RGBColour::new4(0x0, 0x0, 0xf),
	// Index 013: 0xf0f (Bright Magenta)
	RGBColour::new4(0xf, 0x0, 0xf),
	// Index 014: 0x0ff (Bright Cyan)
	RGBColour::new4(0x0, 0xf, 0xf),
	// Index 015: 0xfff (White)
	RGBColour::new4(0xf, 0xf, 0xf),
	// Index 016: 0x003
	RGBColour::new4(0x0, 0x0, 0x3),
	// Index 017: 0x006
	RGBColour::new4(0x0, 0x0, 0x6),
	// Index 018: 0x00c
	RGBColour::new4(0x0, 0x0, 0xc),
	// Index 019: 0x020
	RGBColour::new4(0x0, 0x2, 0x0),
	// Index 020: 0x023
	RGBColour::new4(0x0, 0x2, 0x3),
	// Index 021: 0x026
	RGBColour::new4(0x0, 0x2, 0x6),
	// Index 022: 0x028
	RGBColour::new4(0x0, 0x2, 0x8),
	// Index 023: 0x02c
	RGBColour::new4(0x0, 0x2, 0xc),
	// Index 024: 0x02f
	RGBColour::new4(0x0, 0x2, 0xf),
	// Index 025: 0x040
	RGBColour::new4(0x0, 0x4, 0x0),
	// Index 026: 0x043
	RGBColour::new4(0x0, 0x4, 0x3),
	// Index 027: 0x046
	RGBColour::new4(0x0, 0x4, 0x6),
	// Index 028: 0x048
	RGBColour::new4(0x0, 0x4, 0x8),
	// Index 029: 0x04c
	RGBColour::new4(0x0, 0x4, 0xc),
	// Index 030: 0x04f
	RGBColour::new4(0x0, 0x4, 0xf),
	// Index 031: 0x083
	RGBColour::new4(0x0, 0x8, 0x3),
	// Index 032: 0x086
	RGBColour::new4(0x0, 0x8, 0x6),
	// Index 033: 0x08c
	RGBColour::new4(0x0, 0x8, 0xc),
	// Index 034: 0x08f
	RGBColour::new4(0x0, 0x8, 0xf),
	// Index 035: 0x0a0
	RGBColour::new4(0x0, 0xa, 0x0),
	// Index 036: 0x0a3
	RGBColour::new4(0x0, 0xa, 0x3),
	// Index 037: 0x0a6
	RGBColour::new4(0x0, 0xa, 0x6),
	// Index 038: 0x0a8
	RGBColour::new4(0x0, 0xa, 0x8),
	// Index 039: 0x0ac
	RGBColour::new4(0x0, 0xa, 0xc),
	// Index 040: 0x0af
	RGBColour::new4(0x0, 0xa, 0xf),
	// Index 041: 0x0e0
	RGBColour::new4(0x0, 0xe, 0x0),
	// Index 042: 0x0e3
	RGBColour::new4(0x0, 0xe, 0x3),
	// Index 043: 0x0e6
	RGBColour::new4(0x0, 0xe, 0x6),
	// Index 044: 0x0e8
	RGBColour::new4(0x0, 0xe, 0x8),
	// Index 045: 0x0ec
	RGBColour::new4(0x0, 0xe, 0xc),
	// Index 046: 0x0ef
	RGBColour::new4(0x0, 0xe, 0xf),
	// Index 047: 0x0f3
	RGBColour::new4(0x0, 0xf, 0x3),
	// Index 048: 0x0f6
	RGBColour::new4(0x0, 0xf, 0x6),
	// Index 049: 0x0f8
	RGBColour::new4(0x0, 0xf, 0x8),
	// Index 050: 0x0fc
	RGBColour::new4(0x0, 0xf, 0xc),
	// Index 051: 0x300
	RGBColour::new4(0x3, 0x0, 0x0),
	// Index 052: 0x303
	RGBColour::new4(0x3, 0x0, 0x3),
	// Index 053: 0x306
	RGBColour::new4(0x3, 0x0, 0x6),
	// Index 054: 0x308
	RGBColour::new4(0x3, 0x0, 0x8),
	// Index 055: 0x30c
	RGBColour::new4(0x3, 0x0, 0xc),
	// Index 056: 0x30f
	RGBColour::new4(0x3, 0x0, 0xf),
	// Index 057: 0x320
	RGBColour::new4(0x3, 0x2, 0x0),
	// Index 058: 0x323
	RGBColour::new4(0x3, 0x2, 0x3),
	// Index 059: 0x326
	RGBColour::new4(0x3, 0x2, 0x6),
	// Index 060: 0x328
	RGBColour::new4(0x3, 0x2, 0x8),
	// Index 061: 0x32c
	RGBColour::new4(0x3, 0x2, 0xc),
	// Index 062: 0x32f
	RGBColour::new4(0x3, 0x2, 0xf),
	// Index 063: 0x340
	RGBColour::new4(0x3, 0x4, 0x0),
	// Index 064: 0x343
	RGBColour::new4(0x3, 0x4, 0x3),
	// Index 065: 0x346
	RGBColour::new4(0x3, 0x4, 0x6),
	// Index 066: 0x348
	RGBColour::new4(0x3, 0x4, 0x8),
	// Index 067: 0x34c
	RGBColour::new4(0x3, 0x4, 0xc),
	// Index 068: 0x34f
	RGBColour::new4(0x3, 0x4, 0xf),
	// Index 069: 0x380
	RGBColour::new4(0x3, 0x8, 0x0),
	// Index 070: 0x383
	RGBColour::new4(0x3, 0x8, 0x3),
	// Index 071: 0x386
	RGBColour::new4(0x3, 0x8, 0x6),
	// Index 072: 0x388
	RGBColour::new4(0x3, 0x8, 0x8),
	// Index 073: 0x38c
	RGBColour::new4(0x3, 0x8, 0xc),
	// Index 074: 0x38f
	RGBColour::new4(0x3, 0x8, 0xf),
	// Index 075: 0x3a0
	RGBColour::new4(0x3, 0xa, 0x0),
	// Index 076: 0x3a3
	RGBColour::new4(0x3, 0xa, 0x3),
	// Index 077: 0x3a6
	RGBColour::new4(0x3, 0xa, 0x6),
	// Index 078: 0x3a8
	RGBColour::new4(0x3, 0xa, 0x8),
	// Index 079: 0x3ac
	RGBColour::new4(0x3, 0xa, 0xc),
	// Index 080: 0x3af
	RGBColour::new4(0x3, 0xa, 0xf),
	// Index 081: 0x3e0
	RGBColour::new4(0x3, 0xe, 0x0),
	// Index 082: 0x3e3
	RGBColour::new4(0x3, 0xe, 0x3),
	// Index 083: 0x3e6
	RGBColour::new4(0x3, 0xe, 0x6),
	// Index 084: 0x3e8
	RGBColour::new4(0x3, 0xe, 0x8),
	// Index 085: 0x3ec
	RGBColour::new4(0x3, 0xe, 0xc),
	// Index 086: 0x3ef
	RGBColour::new4(0x3, 0xe, 0xf),
	// Index 087: 0x3f0
	RGBColour::new4(0x3, 0xf, 0x0),
	// Index 088: 0x3f3
	RGBColour::new4(0x3, 0xf, 0x3),
	// Index 089: 0x3f6
	RGBColour::new4(0x3, 0xf, 0x6),
	// Index 090: 0x3f8
	RGBColour::new4(0x3, 0xf, 0x8),
	// Index 091: 0x3fc
	RGBColour::new4(0x3, 0xf, 0xc),
	// Index 092: 0x3ff
	RGBColour::new4(0x3, 0xf, 0xf),
	// Index 093: 0x600
	RGBColour::new4(0x6, 0x0, 0x0),
	// Index 094: 0x603
	RGBColour::new4(0x6, 0x0, 0x3),
	// Index 095: 0x606
	RGBColour::new4(0x6, 0x0, 0x6),
	// Index 096: 0x608
	RGBColour::new4(0x6, 0x0, 0x8),
	// Index 097: 0x60c
	RGBColour::new4(0x6, 0x0, 0xc),
	// Index 098: 0x60f
	RGBColour::new4(0x6, 0x0, 0xf),
	// Index 099: 0x620
	RGBColour::new4(0x6, 0x2, 0x0),
	// Index 100: 0x623
	RGBColour::new4(0x6, 0x2, 0x3),
	// Index 101: 0x626
	RGBColour::new4(0x6, 0x2, 0x6),
	// Index 102: 0x628
	RGBColour::new4(0x6, 0x2, 0x8),
	// Index 103: 0x62c
	RGBColour::new4(0x6, 0x2, 0xc),
	// Index 104: 0x62f
	RGBColour::new4(0x6, 0x2, 0xf),
	// Index 105: 0x640
	RGBColour::new4(0x6, 0x4, 0x0),
	// Index 106: 0x643
	RGBColour::new4(0x6, 0x4, 0x3),
	// Index 107: 0x646
	RGBColour::new4(0x6, 0x4, 0x6),
	// Index 108: 0x648
	RGBColour::new4(0x6, 0x4, 0x8),
	// Index 109: 0x64c
	RGBColour::new4(0x6, 0x4, 0xc),
	// Index 110: 0x64f
	RGBColour::new4(0x6, 0x4, 0xf),
	// Index 111: 0x680
	RGBColour::new4(0x6, 0x8, 0x0),
	// Index 112: 0x683
	RGBColour::new4(0x6, 0x8, 0x3),
	// Index 113: 0x686
	RGBColour::new4(0x6, 0x8, 0x6),
	// Index 114: 0x688
	RGBColour::new4(0x6, 0x8, 0x8),
	// Index 115: 0x68c
	RGBColour::new4(0x6, 0x8, 0xc),
	// Index 116: 0x68f
	RGBColour::new4(0x6, 0x8, 0xf),
	// Index 117: 0x6a0
	RGBColour::new4(0x6, 0xa, 0x0),
	// Index 118: 0x6a3
	RGBColour::new4(0x6, 0xa, 0x3),
	// Index 119: 0x6a6
	RGBColour::new4(0x6, 0xa, 0x6),
	// Index 120: 0x6a8
	RGBColour::new4(0x6, 0xa, 0x8),
	// Index 121: 0x6ac
	RGBColour::new4(0x6, 0xa, 0xc),
	// Index 122: 0x6af
	RGBColour::new4(0x6, 0xa, 0xf),
	// Index 123: 0x6e0
	RGBColour::new4(0x6, 0xe, 0x0),
	// Index 124: 0x6e3
	RGBColour::new4(0x6, 0xe, 0x3),
	// Index 125: 0x6e6
	RGBColour::new4(0x6, 0xe, 0x6),
	// Index 126: 0x6e8
	RGBColour::new4(0x6, 0xe, 0x8),
	// Index 127: 0x6ec
	RGBColour::new4(0x6, 0xe, 0xc),
	// Index 128: 0x6ef
	RGBColour::new4(0x6, 0xe, 0xf),
	// Index 129: 0x6f0
	RGBColour::new4(0x6, 0xf, 0x0),
	// Index 130: 0x6f3
	RGBColour::new4(0x6, 0xf, 0x3),
	// Index 131: 0x6f6
	RGBColour::new4(0x6, 0xf, 0x6),
	// Index 132: 0x6f8
	RGBColour::new4(0x6, 0xf, 0x8),
	// Index 133: 0x6fc
	RGBColour::new4(0x6, 0xf, 0xc),
	// Index 134: 0x6ff
	RGBColour::new4(0x6, 0xf, 0xf),
	// Index 135: 0x803
	RGBColour::new4(0x8, 0x0, 0x3),
	// Index 136: 0x806
	RGBColour::new4(0x8, 0x0, 0x6),
	// Index 137: 0x80c
	RGBColour::new4(0x8, 0x0, 0xc),
	// Index 138: 0x80f
	RGBColour::new4(0x8, 0x0, 0xf),
	// Index 139: 0x820
	RGBColour::new4(0x8, 0x2, 0x0),
	// Index 140: 0x823
	RGBColour::new4(0x8, 0x2, 0x3),
	// Index 141: 0x826
	RGBColour::new4(0x8, 0x2, 0x6),
	// Index 142: 0x828
	RGBColour::new4(0x8, 0x2, 0x8),
	// Index 143: 0x82c
	RGBColour::new4(0x8, 0x2, 0xc),
	// Index 144: 0x82f
	RGBColour::new4(0x8, 0x2, 0xf),
	// Index 145: 0x840
	RGBColour::new4(0x8, 0x4, 0x0),
	// Index 146: 0x843
	RGBColour::new4(0x8, 0x4, 0x3),
	// Index 147: 0x846
	RGBColour::new4(0x8, 0x4, 0x6),
	// Index 148: 0x848
	RGBColour::new4(0x8, 0x4, 0x8),
	// Index 149: 0x84c
	RGBColour::new4(0x8, 0x4, 0xc),
	// Index 150: 0x84f
	RGBColour::new4(0x8, 0x4, 0xf),
	// Index 151: 0x883
	RGBColour::new4(0x8, 0x8, 0x3),
	// Index 152: 0x886
	RGBColour::new4(0x8, 0x8, 0x6),
	// Index 153: 0x88c
	RGBColour::new4(0x8, 0x8, 0xc),
	// Index 154: 0x88f
	RGBColour::new4(0x8, 0x8, 0xf),
	// Index 155: 0x8a0
	RGBColour::new4(0x8, 0xa, 0x0),
	// Index 156: 0x8a3
	RGBColour::new4(0x8, 0xa, 0x3),
	// Index 157: 0x8a6
	RGBColour::new4(0x8, 0xa, 0x6),
	// Index 158: 0x8a8
	RGBColour::new4(0x8, 0xa, 0x8),
	// Index 159: 0x8ac
	RGBColour::new4(0x8, 0xa, 0xc),
	// Index 160: 0x8af
	RGBColour::new4(0x8, 0xa, 0xf),
	// Index 161: 0x8e0
	RGBColour::new4(0x8, 0xe, 0x0),
	// Index 162: 0x8e3
	RGBColour::new4(0x8, 0xe, 0x3),
	// Index 163: 0x8e6
	RGBColour::new4(0x8, 0xe, 0x6),
	// Index 164: 0x8e8
	RGBColour::new4(0x8, 0xe, 0x8),
	// Index 165: 0x8ec
	RGBColour::new4(0x8, 0xe, 0xc),
	// Index 166: 0x8ef
	RGBColour::new4(0x8, 0xe, 0xf),
	// Index 167: 0x8f0
	RGBColour::new4(0x8, 0xf, 0x0),
	// Index 168: 0x8f3
	RGBColour::new4(0x8, 0xf, 0x3),
	// Index 169: 0x8f6
	RGBColour::new4(0x8, 0xf, 0x6),
	// Index 170: 0x8f8
	RGBColour::new4(0x8, 0xf, 0x8),
	// Index 171: 0x8fc
	RGBColour::new4(0x8, 0xf, 0xc),
	// Index 172: 0x8ff
	RGBColour::new4(0x8, 0xf, 0xf),
	// Index 173: 0xc00
	RGBColour::new4(0xc, 0x0, 0x0),
	// Index 174: 0xc03
	RGBColour::new4(0xc, 0x0, 0x3),
	// Index 175: 0xc06
	RGBColour::new4(0xc, 0x0, 0x6),
	// Index 176: 0xc08
	RGBColour::new4(0xc, 0x0, 0x8),
	// Index 177: 0xc0c
	RGBColour::new4(0xc, 0x0, 0xc),
	// Index 178: 0xc0f
	RGBColour::new4(0xc, 0x0, 0xf),
	// Index 179: 0xc20
	RGBColour::new4(0xc, 0x2, 0x0),
	// Index 180: 0xc23
	RGBColour::new4(0xc, 0x2, 0x3),
	// Index 181: 0xc26
	RGBColour::new4(0xc, 0x2, 0x6),
	// Index 182: 0xc28
	RGBColour::new4(0xc, 0x2, 0x8),
	// Index 183: 0xc2c
	RGBColour::new4(0xc, 0x2, 0xc),
	// Index 184: 0xc2f
	RGBColour::new4(0xc, 0x2, 0xf),
	// Index 185: 0xc40
	RGBColour::new4(0xc, 0x4, 0x0),
	// Index 186: 0xc43
	RGBColour::new4(0xc, 0x4, 0x3),
	// Index 187: 0xc46
	RGBColour::new4(0xc, 0x4, 0x6),
	// Index 188: 0xc48
	RGBColour::new4(0xc, 0x4, 0x8),
	// Index 189: 0xc4c
	RGBColour::new4(0xc, 0x4, 0xc),
	// Index 190: 0xc4f
	RGBColour::new4(0xc, 0x4, 0xf),
	// Index 191: 0xc80
	RGBColour::new4(0xc, 0x8, 0x0),
	// Index 192: 0xc83
	RGBColour::new4(0xc, 0x8, 0x3),
	// Index 193: 0xc86
	RGBColour::new4(0xc, 0x8, 0x6),
	// Index 194: 0xc88
	RGBColour::new4(0xc, 0x8, 0x8),
	// Index 195: 0xc8c
	RGBColour::new4(0xc, 0x8, 0xc),
	// Index 196: 0xc8f
	RGBColour::new4(0xc, 0x8, 0xf),
	// Index 197: 0xca0
	RGBColour::new4(0xc, 0xa, 0x0),
	// Index 198: 0xca3
	RGBColour::new4(0xc, 0xa, 0x3),
	// Index 199: 0xca6
	RGBColour::new4(0xc, 0xa, 0x6),
	// Index 200: 0xca8
	RGBColour::new4(0xc, 0xa, 0x8),
	// Index 201: 0xcac
	RGBColour::new4(0xc, 0xa, 0xc),
	// Index 202: 0xcaf
	RGBColour::new4(0xc, 0xa, 0xf),
	// Index 203: 0xce0
	RGBColour::new4(0xc, 0xe, 0x0),
	// Index 204: 0xce3
	RGBColour::new4(0xc, 0xe, 0x3),
	// Index 205: 0xce6
	RGBColour::new4(0xc, 0xe, 0x6),
	// Index 206: 0xce8
	RGBColour::new4(0xc, 0xe, 0x8),
	// Index 207: 0xcec
	RGBColour::new4(0xc, 0xe, 0xc),
	// Index 208: 0xcef
	RGBColour::new4(0xc, 0xe, 0xf),
	// Index 209: 0xcf0
	RGBColour::new4(0xc, 0xf, 0x0),
	// Index 210: 0xcf3
	RGBColour::new4(0xc, 0xf, 0x3),
	// Index 211: 0xcf6
	RGBColour::new4(0xc, 0xf, 0x6),
	// Index 212: 0xcf8
	RGBColour::new4(0xc, 0xf, 0x8),
	// Index 213: 0xcfc
	RGBColour::new4(0xc, 0xf, 0xc),
	// Index 214: 0xcff
	RGBColour::new4(0xc, 0xf, 0xf),
	// Index 215: 0xf03
	RGBColour::new4(0xf, 0x0, 0x3),
	// Index 216: 0xf06
	RGBColour::new4(0xf, 0x0, 0x6),
	// Index 217: 0xf08
	RGBColour::new4(0xf, 0x0, 0x8),
	// Index 218: 0xf0c
	RGBColour::new4(0xf, 0x0, 0xc),
	// Index 219: 0xf20
	RGBColour::new4(0xf, 0x2, 0x0),
	// Index 220: 0xf23
	RGBColour::new4(0xf, 0x2, 0x3),
	// Index 221: 0xf26
	RGBColour::new4(0xf, 0x2, 0x6),
	// Index 222: 0xf28
	RGBColour::new4(0xf, 0x2, 0x8),
	// Index 223: 0xf2c
	RGBColour::new4(0xf, 0x2, 0xc),
	// Index 224: 0xf2f
	RGBColour::new4(0xf, 0x2, 0xf),
	// Index 225: 0xf40
	RGBColour::new4(0xf, 0x4, 0x0),
	// Index 226: 0xf43
	RGBColour::new4(0xf, 0x4, 0x3),
	// Index 227: 0xf46
	RGBColour::new4(0xf, 0x4, 0x6),
	// Index 228: 0xf48
	RGBColour::new4(0xf, 0x4, 0x8),
	// Index 229: 0xf4c
	RGBColour::new4(0xf, 0x4, 0xc),
	// Index 230: 0xf4f
	RGBColour::new4(0xf, 0x4, 0xf),
	// Index 231: 0xf80
	RGBColour::new4(0xf, 0x8, 0x0),
	// Index 232: 0xf83
	RGBColour::new4(0xf, 0x8, 0x3),
	// Index 233: 0xf86
	RGBColour::new4(0xf, 0x8, 0x6),
	// Index 234: 0xf88
	RGBColour::new4(0xf, 0x8, 0x8),
	// Index 235: 0xf8c
	RGBColour::new4(0xf, 0x8, 0xc),
	// Index 236: 0xf8f
	RGBColour::new4(0xf, 0x8, 0xf),
	// Index 237: 0xfa0
	RGBColour::new4(0xf, 0xa, 0x0),
	// Index 238: 0xfa3
	RGBColour::new4(0xf, 0xa, 0x3),
	// Index 239: 0xfa6
	RGBColour::new4(0xf, 0xa, 0x6),
	// Index 240: 0xfa8
	RGBColour::new4(0xf, 0xa, 0x8),
	// Index 241: 0xfac
	RGBColour::new4(0xf, 0xa, 0xc),
	// Index 242: 0xfaf
	RGBColour::new4(0xf, 0xa, 0xf),
	// Index 243: 0xfe0
	RGBColour::new4(0xf, 0xe, 0x0),
	// Index 244: 0xfe3
	RGBColour::new4(0xf, 0xe, 0x3),
	// Index 245: 0xfe6
	RGBColour::new4(0xf, 0xe, 0x6),
	// Index 246: 0xfe8
	RGBColour::new4(0xf, 0xe, 0x8),
	// Index 247: 0xfec
	RGBColour::new4(0xf, 0xe, 0xc),
	// Index 248: 0xfef
	RGBColour::new4(0xf, 0xe, 0xf),
	// Index 249: 0xff3
	RGBColour::new4(0xf, 0xf, 0x3),
	// Index 250: 0xff6
	RGBColour::new4(0xf, 0xf, 0x6),
	// Index 251: 0xff8
	RGBColour::new4(0xf, 0xf, 0x8),
	// Index 252: 0xffc
	RGBColour::new4(0xf, 0xf, 0xc),
	// Index 253: 0xbbb
	RGBColour::new4(0xb, 0xb, 0xb),
	// Index 254: 0x333
	RGBColour::new4(0x3, 0x3, 0x3),
	// Index 255: 0x777
	RGBColour::new4(0x7, 0x7, 0x7),
];

/// This is our text buffer.
///
/// This is arranged as `NUM_TEXT_ROWS` rows of `NUM_TEXT_COLS` columns.
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
	// | 24    | jmp x-- loop0 | ..               |
	// | 25    | jmp x-- loop0 | ..               |
	// | 26    | jmp x-- loop0 | jmp x-- loop1    |
	// | 27    | jmp x-- loop0 | out pins, 16 [9] |
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

	// Note (safety): No-one else is looking at the `TEXT_COLOUR_LOOKUP` table
	// at this point, and access to `VIDEO_PALETTE` is read-only.
	unsafe {
		TEXT_COLOUR_LOOKUP.init(&VIDEO_PALETTE);
	}

	debug!("Text colour lookup filled");

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
		core::mem::size_of_val(core1_stack)
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
unsafe extern "C" fn core1_main() -> u32 {
	CORE1_START_FLAG.store(true, Ordering::Relaxed);

	loop {
		let mut waited: u32 = 0;
		waited += render_scanline(&mut PIXEL_DATA_BUFFER_ODD);
		waited += render_scanline(&mut PIXEL_DATA_BUFFER_EVEN);
		RENDER_TIME.store(
			RENDER_TIME.load(Ordering::Relaxed) + (waited / 2),
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
				CLASHED_COUNT.store(CLASHED_COUNT.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
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
				CLASHED_COUNT.store(CLASHED_COUNT.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
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

	// Wake up both CPUs with an event. This IRQ might not be on the rendering
	// core.
	cortex_m::asm::sev();
}

/// Performs the VGA rendering.
///
/// This routine runs at 2364 clocks/line in 80x30 text mode.
fn render_scanline(scan_line_buffer: &mut LineBuffer) -> u32 {
	// Wait for this buffer to be ready for us
	while !scan_line_buffer.is_ready_for_rendering() {
		cortex_m::asm::wfe();
	}

	let syst = unsafe { &*cortex_m::peripheral::SYST::ptr() };
	unsafe {
		syst.csr.modify(|v| v & !1);
		syst.csr.modify(|v| v | 4);
		syst.rvr.write(0xffffff);
		syst.cvr.write(0);
		syst.csr.modify(|v| v | 1);
	}

	let font = match unsafe { VIDEO_MODE.format() } {
		crate::common::video::Format::Text8x16 => &font16::FONT,
		crate::common::video::Format::Text8x8 => &font8::FONT,
		_ => {
			return 0;
		}
	};

	let num_rows = NUM_TEXT_ROWS.load(Ordering::Relaxed);
	let num_cols = NUM_TEXT_COLS.load(Ordering::Relaxed);

	// Which line do we want?
	let current_line_num = scan_line_buffer.line_number.load(Ordering::SeqCst);

	// Convert our position in scan-lines to a text row, and a line within each glyph on that row
	let text_row = current_line_num as usize >> font.height_shift;
	let font_row = current_line_num as usize & ((1 << font.height_shift) - 1);

	if text_row >= num_rows {
		return 0;
	}

	// Note (unsafe): accessing a static mut, but we do it via a const ptr.
	let row_start: *const GlyphAttr = unsafe { GLYPH_ATTR_ARRAY.as_ptr().add(text_row * num_cols) };

	// Get a pointer into our scan-line buffer
	let scan_line_buffer_ptr = scan_line_buffer.pixels.as_mut_ptr();

	// Every font look-up we are about to do for this row will
	// involve offsetting by the row within each glyph. As this
	// is the same for every glyph on this row, we calculate a
	// new pointer once, in advance, and save ourselves an
	// addition each time around the loop.
	let font_ptr = unsafe { font.data.as_ptr().add(font_row * 256) };

	match num_cols {
		80 => render_scanline_text::<80>(row_start, font_ptr, scan_line_buffer_ptr),
		40 => render_scanline_text::<40>(row_start, font_ptr, scan_line_buffer_ptr),
		_ => {
			// Do nothing
		}
	}

	scan_line_buffer.mark_rendering_done();

	0xffffff - syst.cvr.read()
}

/// Render one line of N-column text mode
///
/// We bring this out into a function as making the for loop have a fixed range
/// appears to greatly speed up the generated code.
fn render_scanline_text<const N: usize>(
	row_start: *const GlyphAttr,
	font_ptr: *const u8,
	scan_line_buffer_ptr: *mut RGBPair,
) {
	let mut pair_offset = 0;

	// Convert from characters to coloured pixels, using the font as a look-up table.
	for col in 0..N {
		// Get the 16-bit glyph/attribute pair
		let glyphattr = unsafe { core::ptr::read(row_start.add(col)) };
		// Grab just the attribute
		let attr = glyphattr.attr();
		// Where in the font do we need to look up. Note that the `font_ptr`
		// is already offset for the line (out of 8, or out of 16) that we're
		// looking at.
		let glyph_index = glyphattr.glyph().0 as usize;

		// Note (unsafe): We use pointer arithmetic here because we can't
		// afford a bounds-check on an array. This is safe because the font
		// is `256 * width` bytes long and we can't index more than `255 *
		// width` bytes into it. We also touch a bunch of static muts, but a
		// race hazard merely results in a graphical glitch for 1/60th of a
		// second, so it doesn't matter.
		unsafe {
			// Grab 0bXXXXXXXX where X=1 means foreground, and X=0 means background
			let mono_pixels = core::ptr::read(font_ptr.add(glyph_index));
			// 0bXX------
			let pair = TEXT_COLOUR_LOOKUP.lookup(attr, mono_pixels >> 6);
			core::ptr::write(scan_line_buffer_ptr.offset(pair_offset), pair);
			// 0b--XX----
			let pair = TEXT_COLOUR_LOOKUP.lookup(attr, mono_pixels >> 4);
			core::ptr::write(scan_line_buffer_ptr.offset(pair_offset + 1), pair);
			// 0b----XX--
			let pair = TEXT_COLOUR_LOOKUP.lookup(attr, mono_pixels >> 2);
			core::ptr::write(scan_line_buffer_ptr.offset(pair_offset + 2), pair);
			// 0b------XX
			let pair = TEXT_COLOUR_LOOKUP.lookup(attr, mono_pixels);
			core::ptr::write(scan_line_buffer_ptr.offset(pair_offset + 3), pair);
		}

		pair_offset += 4;
	}
}

impl<'a> Font<'a> {
	/// This function performs a glyph look-up based on the Font being Code Page 850.
	fn convert_char(input: char) -> Option<Glyph> {
		if input as u32 <= 127 {
			Some(Glyph(input as u8))
		} else {
			match input {
				'\u{00A0}' => Some(Glyph(255)), // NBSP
				'\u{00A1}' => Some(Glyph(173)), // ¡
				'\u{00A2}' => Some(Glyph(189)), // ¢
				'\u{00A3}' => Some(Glyph(156)), // £
				'\u{00A4}' => Some(Glyph(207)), // ¤
				'\u{00A5}' => Some(Glyph(190)), // ¥
				'\u{00A6}' => Some(Glyph(221)), // ¦
				'\u{00A7}' => Some(Glyph(245)), // §
				'\u{00A8}' => Some(Glyph(249)), // ¨
				'\u{00A9}' => Some(Glyph(184)), // ©
				'\u{00AA}' => Some(Glyph(166)), // ª
				'\u{00AB}' => Some(Glyph(174)), // «
				'\u{00AC}' => Some(Glyph(170)), // ¬
				'\u{00AD}' => Some(Glyph(240)), // SHY
				'\u{00AE}' => Some(Glyph(169)), // ®
				'\u{00AF}' => Some(Glyph(238)), // ¯
				'\u{00B0}' => Some(Glyph(248)), // °
				'\u{00B1}' => Some(Glyph(241)), // ±
				'\u{00B2}' => Some(Glyph(253)), // ²
				'\u{00B3}' => Some(Glyph(252)), // ³
				'\u{00B4}' => Some(Glyph(239)), // ´
				'\u{00B5}' => Some(Glyph(230)), // µ
				'\u{00B6}' => Some(Glyph(244)), // ¶
				'\u{00B7}' => Some(Glyph(250)), // ·
				'\u{00B8}' => Some(Glyph(247)), // ¸
				'\u{00B9}' => Some(Glyph(251)), // ¹
				'\u{00BA}' => Some(Glyph(167)), // º
				'\u{00BB}' => Some(Glyph(175)), // »
				'\u{00BC}' => Some(Glyph(172)), // ¼
				'\u{00BD}' => Some(Glyph(171)), // ½
				'\u{00BE}' => Some(Glyph(243)), // ¾
				'\u{00BF}' => Some(Glyph(168)), // ¿
				'\u{00C0}' => Some(Glyph(183)), // À
				'\u{00C1}' => Some(Glyph(181)), // Á
				'\u{00C2}' => Some(Glyph(182)), // Â
				'\u{00C3}' => Some(Glyph(199)), // Ã
				'\u{00C4}' => Some(Glyph(142)), // Ä
				'\u{00C5}' => Some(Glyph(143)), // Å
				'\u{00C6}' => Some(Glyph(146)), // Æ
				'\u{00C7}' => Some(Glyph(128)), // Ç
				'\u{00C8}' => Some(Glyph(212)), // È
				'\u{00C9}' => Some(Glyph(144)), // É
				'\u{00CA}' => Some(Glyph(210)), // Ê
				'\u{00CB}' => Some(Glyph(211)), // Ë
				'\u{00CC}' => Some(Glyph(222)), // Ì
				'\u{00CD}' => Some(Glyph(214)), // Í
				'\u{00CE}' => Some(Glyph(215)), // Î
				'\u{00CF}' => Some(Glyph(216)), // Ï
				'\u{00D0}' => Some(Glyph(209)), // Ð
				'\u{00D1}' => Some(Glyph(165)), // Ñ
				'\u{00D2}' => Some(Glyph(227)), // Ò
				'\u{00D3}' => Some(Glyph(224)), // Ó
				'\u{00D4}' => Some(Glyph(226)), // Ô
				'\u{00D5}' => Some(Glyph(229)), // Õ
				'\u{00D6}' => Some(Glyph(153)), // Ö
				'\u{00D7}' => Some(Glyph(158)), // ×
				'\u{00D8}' => Some(Glyph(157)), // Ø
				'\u{00D9}' => Some(Glyph(235)), // Ù
				'\u{00DA}' => Some(Glyph(233)), // Ú
				'\u{00DB}' => Some(Glyph(234)), // Û
				'\u{00DC}' => Some(Glyph(154)), // Ü
				'\u{00DD}' => Some(Glyph(237)), // Ý
				'\u{00DE}' => Some(Glyph(232)), // Þ
				'\u{00DF}' => Some(Glyph(225)), // ß
				'\u{00E0}' => Some(Glyph(133)), // à
				'\u{00E1}' => Some(Glyph(160)), // á
				'\u{00E2}' => Some(Glyph(131)), // â
				'\u{00E3}' => Some(Glyph(198)), // ã
				'\u{00E4}' => Some(Glyph(132)), // ä
				'\u{00E5}' => Some(Glyph(134)), // å
				'\u{00E6}' => Some(Glyph(145)), // æ
				'\u{00E7}' => Some(Glyph(135)), // ç
				'\u{00E8}' => Some(Glyph(138)), // è
				'\u{00E9}' => Some(Glyph(130)), // é
				'\u{00EA}' => Some(Glyph(136)), // ê
				'\u{00EB}' => Some(Glyph(137)), // ë
				'\u{00EC}' => Some(Glyph(141)), // ì
				'\u{00ED}' => Some(Glyph(161)), // í
				'\u{00EE}' => Some(Glyph(140)), // î
				'\u{00EF}' => Some(Glyph(139)), // ï
				'\u{00F0}' => Some(Glyph(208)), // ð
				'\u{00F1}' => Some(Glyph(164)), // ñ
				'\u{00F2}' => Some(Glyph(149)), // ò
				'\u{00F3}' => Some(Glyph(162)), // ó
				'\u{00F4}' => Some(Glyph(147)), // ô
				'\u{00F5}' => Some(Glyph(228)), // õ
				'\u{00F6}' => Some(Glyph(148)), // ö
				'\u{00F7}' => Some(Glyph(246)), // ÷
				'\u{00F8}' => Some(Glyph(155)), // ø
				'\u{00F9}' => Some(Glyph(151)), // ù
				'\u{00FA}' => Some(Glyph(163)), // ú
				'\u{00FB}' => Some(Glyph(150)), // û
				'\u{00FC}' => Some(Glyph(129)), // ü
				'\u{00FD}' => Some(Glyph(236)), // ý
				'\u{00FE}' => Some(Glyph(231)), // þ
				'\u{00FF}' => Some(Glyph(152)), // ÿ
				'\u{0131}' => Some(Glyph(213)), // ı
				'\u{0192}' => Some(Glyph(159)), // ƒ
				'\u{2017}' => Some(Glyph(242)), // ‗
				'\u{2500}' => Some(Glyph(196)), // ─
				'\u{2502}' => Some(Glyph(179)), // │
				'\u{250C}' => Some(Glyph(218)), // ┌
				'\u{2510}' => Some(Glyph(191)), // ┐
				'\u{2514}' => Some(Glyph(192)), // └
				'\u{2518}' => Some(Glyph(217)), // ┘
				'\u{251C}' => Some(Glyph(195)), // ├
				'\u{2524}' => Some(Glyph(180)), // ┤
				'\u{252C}' => Some(Glyph(194)), // ┬
				'\u{2534}' => Some(Glyph(193)), // ┴
				'\u{253C}' => Some(Glyph(197)), // ┼
				'\u{2550}' => Some(Glyph(205)), // ═
				'\u{2551}' => Some(Glyph(186)), // ║
				'\u{2554}' => Some(Glyph(201)), // ╔
				'\u{2557}' => Some(Glyph(187)), // ╗
				'\u{255A}' => Some(Glyph(200)), // ╚
				'\u{255D}' => Some(Glyph(188)), // ╝
				'\u{2560}' => Some(Glyph(204)), // ╠
				'\u{2563}' => Some(Glyph(185)), // ╣
				'\u{2566}' => Some(Glyph(203)), // ╦
				'\u{2569}' => Some(Glyph(202)), // ╩
				'\u{256C}' => Some(Glyph(206)), // ╬
				'\u{2580}' => Some(Glyph(223)), // ▀
				'\u{2584}' => Some(Glyph(220)), // ▄
				'\u{2588}' => Some(Glyph(219)), // █
				'\u{2591}' => Some(Glyph(176)), // ░
				'\u{2592}' => Some(Glyph(177)), // ▒
				'\u{2593}' => Some(Glyph(178)), // ▓
				'\u{25A0}' => Some(Glyph(254)), // ■
				_ => None,
			}
		}
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
				Attr::new(
					TextForegroundColour::WHITE,
					TextBackgroundColour::BLACK,
					false,
				)
				.as_u8(),
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
						row += 1;
						col = 0;
					}
					'\r' => {
						// Carriage Return
						col = 0;
					}
					_ => {
						let glyph = Font::convert_char(ch).unwrap_or(Glyph(b'?'));
						let glyphattr = GlyphAttr::new(glyph, attr);
						self.write_at(glyphattr, buffer, row, col, num_cols);
						col += 1;
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

impl TextColourLookup {
	/// Make a blank lookup table at start-up.
	const fn blank() -> TextColourLookup {
		TextColourLookup {
			entries: [RGBPair(0); 512],
		}
	}

	/// Populate the look-up table with data from the given palette.
	fn init(&mut self, palette: &[RGBColour]) {
		for (fg, fg_colour) in palette.iter().take(16).enumerate() {
			for (bg, bg_colour) in palette.iter().take(8).enumerate() {
				let attr = Attr::new(
					unsafe { TextForegroundColour::new_unchecked(fg as u8) },
					unsafe { TextBackgroundColour::new_unchecked(bg as u8) },
					false,
				);
				for pixels in 0..=3 {
					let index: usize = (((attr.0 & 0x7F) as usize) << 2) | (pixels & 0x03) as usize;
					let pair = RGBPair::new(
						if pixels & 0x02 == 0x02 {
							*fg_colour
						} else {
							*bg_colour
						},
						if pixels & 0x01 == 0x01 {
							*fg_colour
						} else {
							*bg_colour
						},
					);
					self.entries[index] = pair;
				}
			}
		}
	}

	/// Grab a pixel pair from the look-up table, given a text-mode `Attr`.
	///
	/// Only looks at the bottom two bits of `pixels`.
	#[inline]
	fn lookup(&self, attr: Attr, pixels: u8) -> RGBPair {
		let index: usize = (((attr.0 & 0x7F) as usize) << 2) | (pixels & 0x03) as usize;
		unsafe { core::ptr::read(self.entries.as_ptr().add(index)) }
	}
}

impl RGBColour {
	/// Make a 12-bit RGB Colour from 8-bit red, green and blue values.
	pub const fn new8(red: u8, green: u8, blue: u8) -> RGBColour {
		let red = (red >> 4) as u16;
		let green = (green >> 4) as u16;
		let blue = (blue >> 4) as u16;
		RGBColour((blue << 8) | (green << 4) | red)
	}

	/// Make a 12-bit RGB Colour from 3-bit red, green and blue values.
	pub const fn new4(red: u8, green: u8, blue: u8) -> RGBColour {
		let red = (red & 0x0F) as u16;
		let green = (green & 0x0F) as u16;
		let blue = (blue & 0x0F) as u16;
		RGBColour((blue << 8) | (green << 4) | red)
	}
}

impl RGBPair {
	/// Make a new RGB Pair from two RGB Colours.
	pub const fn new(first: RGBColour, second: RGBColour) -> RGBPair {
		let first: u32 = first.0 as u32;
		let second: u32 = second.0 as u32;
		RGBPair((second << 16) | first)
	}
}

// -----------------------------------------------------------------------------
// End of file
// -----------------------------------------------------------------------------
