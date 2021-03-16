#![deny(unsafe_code)]
#![no_main]
#![no_std]

use aux5::{entry, Delay, DelayMs, LedArray, OutputSwitch};
use volatile::Volatile;

#[entry]
fn main() -> ! {
    let (mut delay, mut leds): (Delay, LedArray) = aux5::init();
    let mut half_period = 500_u16;
    let v_half_period = Volatile::new(&mut half_period);

    // infinite loop; just so we don't leave this stack frame
    loop {
        leds[0].on().ok();
        delay.delay_ms(v_half_period.read());

        leds[0].off().ok();
        delay.delay_ms(v_half_period.read());
    }
}
