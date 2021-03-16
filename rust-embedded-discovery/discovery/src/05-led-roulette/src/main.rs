#![deny(unsafe_code)]
#![no_main]
#![no_std]

use aux5::{entry, Delay, DelayMs, LedArray, OutputSwitch};

#[entry]
fn main() -> ! {
    let (mut delay, mut leds): (Delay, LedArray) = aux5::init();

    let on_delay_ms = 100_u16;
    let max_lights = 8_usize;

    // infinite loop; just so we don't leave this stack frame
    loop {
        for i in 0..max_lights {
            leds[i].on().ok();
            delay.delay_ms(on_delay_ms);

            let new_index = (i + 1) % max_lights;
            leds[new_index].on().ok();
            delay.delay_ms(on_delay_ms / 2);

            leds[i].off().ok();
        }
    }
}
