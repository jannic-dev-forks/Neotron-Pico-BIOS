/**
 * Linker script for the Raspberry Pi Pico.
 *
 * Part of the Neotron Pico BIOS.
 *
 * This file licenced as CC0.
 */

MEMORY {
    /*
     * This is bootloader for the RP2040. It must live at the start of the
       external flash chip.
     */
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    /*
     * The Pico has 2048 KiB of external Flash Memory. We allow ourselves 128
     * KiB for the BIOS, leaving the rest
     * for the OS and any user applications.
     */
    FLASH : ORIGIN = 0x10000100, LENGTH = 0x1FF00
    /*
     * This is the remainder of the 2048 KiB flash chip on the Pico.
     */
    FLASH_OS: ORIGIN = 0x10020000, LENGTH = 0x1E0000
    /*
     * The RP2040 has 256 KiB of SRAM striped across four banks (for high
     * performance), plus a fifth bank containing another 8 KiB of RAM. We
     * want to be at the top of the high-performance RAM, and we give ourselves
     * 16 KiB to work with (for stack, text buffer, pixel buffers, etc.).
     */
    RAM   : ORIGIN = 0x2003C000, LENGTH = 0x4000
    /*
     * This is where the OS and any applications are loaded.
     */
    OSRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x3C000
}

/*
 * This is where the call stack is located. It is of the full descending
 * type (i.e. it grows downwards towards 0x2000_0000).
 *
 * We start it at the top of the application RAM region.
 */
_stack_start = ORIGIN(RAM);

/*
 * Export some symbols to tell the BIOS where it might find the OS.
 */
_start_os_flash_sym = ORIGIN(FLASH) + LENGTH(FLASH);
_end_os_flash_sym = ORIGIN(FLASH) + 256K;
_start_osram_sym = ORIGIN(OSRAM);
_end_osram_sym = ORIGIN(OSRAM) + LENGTH(OSRAM);

SECTIONS {
    /* ### RP2040 Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;

/* End of file */
