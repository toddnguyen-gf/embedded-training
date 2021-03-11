#![no_std]
#![no_main]

use panic_halt as _;

use heapless::consts::*;
use heapless::Vec;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

#[entry]
fn main() -> ! {
    let mut xs: Vec<_, U8> = Vec::new();

    xs.push(42).unwrap();
    hprintln!("{:?}", xs).unwrap();
    assert_eq!(xs.pop(), Some(42));
    hprintln!("{:?}", xs).unwrap();

    loop {}
}
