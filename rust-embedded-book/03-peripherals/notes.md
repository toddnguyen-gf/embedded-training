# Singletons in Rust

https://docs.rust-embedded.org/book/peripherals/singletons.html

## Defining Singleton ourselves

```rust
struct Peripherals {
    serial: Option<SerialPort>,
}
impl Peripherals {
    fn take_serial(&mut self) -> SerialPort {
        let p = replace(&mut self.serial, None);
        p.unwrap()
    }
}
static mut PERIPHERALS: Peripherals = Peripherals {
    serial: Some(SerialPort),
};

fn main() {
    let serial_1: SerialPort = unsafe { PERIPHERALS.take_serial() };
    // This panics!
    // let serial_2 = unsafe { PERIPHERALS.take_serial() };
}
```

- If we try to call `take_serial()` more than once, our code will panic!
- Once we have the `SerialPort` structure it contains, we no longer need to use `unsafe` NOR the `PERIPHERALS` structure

## cortex_m Library Support

```rust
#[macro_use(singleton)]
extern crate cortex_m;

fn main() {
    // OK if `main` is executed only once
    let x: &'static mut bool =
        singleton!(: bool = false).unwrap();
}
```

## cortex-m-rtic Library Support

```rust
// cortex-m-rtic v0.5.x
#[rtic::app(device = lm3s6965, peripherals = true)]
const APP: () = {
    #[init]
    fn init(cx: init::Context) {
        static mut X: u32 = 0;

        // Cortex-M peripherals
        let core: cortex_m::Peripherals = cx.core;

        // Device specific peripherals
        let device: lm3s6965::Peripherals = cx.device;
    }
}
```

---

## But why?

1. Singletons = only one way to obtain a `SerialPort`
1. To call any functions, we must have ownership or reference to a `SerialPort` structure

These two factors put together means that it is only possible to access the hardware if we have appropriately satisfied the borrow checker, meaning that at no point do we have multiple mutable references to the same hardware!
