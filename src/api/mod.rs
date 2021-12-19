//! # BIOS API Implementations for the Neotron Pico BIOS

use super::common;

/// The table of API calls we provide the OS
pub static API_CALLS: common::Api = common::Api {
	api_version_get,
	bios_version_get,
	serial_configure,
	serial_get_info,
	serial_write,
	time_get,
	time_set,
	video_memory_info_get,
	configuration_get,
	configuration_set,
};

/// The BIOS version string
pub static BIOS_VERSION: &str = concat!(
	"Neotron Pico BIOS, version ",
	env!("CARGO_PKG_VERSION"),
	"\0"
);

/// Get the API version this crate implements
pub extern "C" fn api_version_get() -> u32 {
	common::API_VERSION
}

/// Get this BIOS version as a string.
pub extern "C" fn bios_version_get() -> common::ApiString<'static> {
	BIOS_VERSION.into()
}

/// Re-configure the UART. This is unimplemented right now.
pub extern "C" fn serial_configure(
	_device: u8,
	_serial_config: common::serial::Config,
) -> common::Result<()> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Get infomation about the UARTs available in ths system.
///
/// We don't have support for any right now.
pub extern "C" fn serial_get_info(_device: u8) -> common::Option<common::serial::DeviceInfo> {
	common::Option::None
}

/// Write some text to a UART.
///
/// Except we don't have one.
pub extern "C" fn serial_write(
	_device: u8,
	_data: common::ApiByteSlice,
	_timeout: common::Option<common::Timeout>,
) -> common::Result<usize> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Get the current wall time.
pub extern "C" fn time_get() -> common::Time {
	// TODO - implement an RTC
	common::Time {
		frames_since_second: 0,
		seconds_since_epoch: 0,
	}
}

/// Set the current wall time.
pub extern "C" fn time_set(_new_time: common::Time) {
	// TODO: Write the new time to the RTC (which is only accurate to the second)
}

/// Gets information about the memory-mapped text buffer.
pub extern "C" fn video_memory_info_get(
	address: &mut *mut u8,
	width_cols: &mut u8,
	height_rows: &mut u8,
) {
	*address = unsafe { super::vga::CHAR_ARRAY.as_mut_ptr() as *mut u8 };
	*width_cols = 80;
	*height_rows = 25;
}

/// Loads config data from the TM4C EEPROM
pub extern "C" fn configuration_get(mut buffer: common::ApiBuffer) -> common::Result<usize> {
	// Canned config for now
	// This is awful. Do not do this.
	let buffer = buffer.as_mut_slice();
	if buffer.len() >= 6 {
		buffer[0] = 0x01;
		buffer[1] = 0x01;
		buffer[2] = 0x00;
		buffer[3] = 0xc2;
		buffer[4] = 0x01;
		buffer[5] = 0x00;
		common::Result::Ok(6)
	} else {
		common::Result::Err(common::Error::DeviceError(0))
	}
}

/// Saves config data to the TM4C EEPROM
pub extern "C" fn configuration_set(_buffer: common::ApiByteSlice) -> common::Result<()> {
	// Lie and say we stored it OK
	common::Result::Ok(())
}

// End of file
