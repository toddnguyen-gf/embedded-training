#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

#[entry]
fn main() -> ! {
    // OK if `main` is executed only once
    let x: &'static mut bool = cortex_m::singleton!(: bool = false).unwrap();

    hprintln!("{:?}", x).unwrap();

    loop {}
}
