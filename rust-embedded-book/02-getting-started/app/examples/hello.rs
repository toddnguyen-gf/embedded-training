//! Prints "Hello, world!" on the host console using semihosting

#![no_main]
#![no_std]

use core::fmt::Write;

use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hio, hprintln};

#[entry]
fn main() -> ! {
    // hprintln!("Hello, world!").unwrap();
    let mut stdout = hio::hstdout().unwrap();
    writeln!(stdout, "Hello, world!").unwrap();

    // exit QEMU
    // NOTE do not run this on hardware; it can corrupt OpenOCD state
    // debug::exit(debug::EXIT_SUCCESS);

    loop {}
}
