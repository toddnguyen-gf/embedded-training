# Chapter 06 - Concurrency

Concurrency happens whenever different parts of your program might execute at different times or out of order. In an embedded context, this includes:

- various forms of multithreading, where your microprocessor regularly swaps between parts of your program,
- interrupt handlers, which run whenever the associated interrupt happens,
- and in some systems, multiple-core microprocessors, where each core can be independently running a different part of your program at the same time.

## Global Mutable Data

Our interrupt handlers might be called at any time and must know how to access whatever shared memory we are using. At the lowest level, this means we must have **statically allocated mutable memory**, which both the interrupt handler and the main code can refer to.

Problem with `static mut`!

```rust
static mut COUNTER: u32 = 0;

#[entry]
fn main() -> ! {
    set_timer_1hz();
    let mut last_state = false;
    loop {
        let state = read_signal_level();
        if state && !last_state {
            // DANGER - Not actually safe! Could cause data races.
            unsafe { COUNTER += 1 };
        }
        last_state = state;
    }
}

#[interrupt]
fn timer() {
    // Might be ignored after the interrupt returns
    unsafe { COUNTER = 0; }
}
```

- Increment on `COUNTER` is not guaranteed to be atomic!
- If the interrupt fired after the load but before the store, the reset back to 0 would be ignored after the interrupt returns - and we would count twice as many transitions for that period.

## Critical Sections

- A simple approach is to use critical sections, a context where interrupts are disabled.
- By wrapping the access to `COUNTER` in main in a critical section, we can be sure the timer interrupt will not fire until we're finished incrementing `COUNTER`

```rust
static mut COUNTER: u32 = 0;

#[entry]
fn main() -> ! {
    set_timer_1hz();
    let mut last_state = false;
    loop {
        let state = read_signal_level();
        if state && !last_state {
            // New critical section ensures synchronised access to COUNTER
            cortex_m::interrupt::free(|_| {
                unsafe { COUNTER += 1 };
            });
        }
        last_state = state;
    }
}

#[interrupt]
fn timer() {
    unsafe { COUNTER = 0; }
}
```

- This is also the same as disabling interrupts, running some code, and then re-enabling interrupts.
- Since each critical section temporarily pauses interrupt processing, there is an associated cost of some extra code size and higher interrupt latency and jitter
- It's worth noting that while a critical section guarantees no interrupts will fire, it does NOT provide an exclusivity guarantee on _multi-core_ systems!

## Atomic Access

- Compare and Swap (CAS) instructions give an alternative to the heavy-handed disabling of all interrupts: we can attempt the increment, it will succeed most of the time, but if it was interrupted it will automatically retry the entire increment operation. These atomic operations are safe even across multiple cores.

```rust
use core::sync::atomic::{AtomicUsize, Ordering};

static COUNTER: AtomicUsize = AtomicUsize::new(0);

#[entry]
fn main() -> ! {
    set_timer_1hz();
    let mut last_state = false;
    loop {
        let state = read_signal_level();
        if state && !last_state {
            // Use `fetch_add` to atomically add 1 to COUNTER
            COUNTER.fetch_add(1, Ordering::Relaxed);
        }
        last_state = state;
    }
}

#[interrupt]
fn timer() {
    // Use `store` to write 0 directly to COUNTER
    COUNTER.store(0, Ordering::Relaxed)
}
```

- `COUNTER` is a safe static variable
- However this may not be available on all platforms
- A note on `Ordering`: this affects how the compiler and hardware may reorder instructions, and also has consequences on cache visibility. Assuming that the target is a single core platform `Relaxed` is sufficient and the most efficient choice in this particular case.

# Abstractions, Send and Sync

- We can abstract our counter into a safe interface which can be safely used anywhere else in our code. For this example, we'll use the critical-section counter, but you could do something very similar with atomics.

```rust
use core::cell::UnsafeCell;
use cortex_m::interrupt;

// Our counter is just a wrapper around UnsafeCell<u32>, which is the heart
// of interior mutability in Rust. By using interior mutability, we can have
// COUNTER be `static` instead of `static mut`, but still able to mutate
// its counter value.
struct CSCounter(UnsafeCell<u32>);

const CS_COUNTER_INIT: CSCounter = CSCounter(UnsafeCell::new(0));

impl CSCounter {
    pub fn reset(&self, _cs: &interrupt::CriticalSection) {
        // By requiring a CriticalSection be passed in, we know we must
        // be operating inside a CriticalSection, and so can confidently
        // use this unsafe block (required to call UnsafeCell::get).
        unsafe { *self.0.get() = 0 };
    }

    pub fn increment(&self, _cs: &interrupt::CriticalSection) {
        unsafe { *self.0.get() += 1 };
    }
}

// Required to allow static CSCounter. See explanation below.
unsafe impl Sync for CSCounter {}

// COUNTER is no longer `mut` as it uses interior mutability;
// therefore it also no longer requires unsafe blocks to access.
static COUNTER: CSCounter = CS_COUNTER_INIT;

#[entry]
fn main() -> ! {
    set_timer_1hz();
    let mut last_state = false;
    loop {
        let state = read_signal_level();
        if state && !last_state {
            // No unsafe here!
            interrupt::free(|cs| COUNTER.increment(cs));
        }
        last_state = state;
    }
}

#[interrupt]
fn timer() {
    // We do need to enter a critical section here just to obtain a valid
    // cs token, even though we know no other interrupt could pre-empt
    // this one.
    interrupt::free(|cs| COUNTER.reset(cs));

    // We could use unsafe code to generate a fake CriticalSection if we
    // really wanted to, avoiding the overhead:
    // let cs = unsafe { interrupt::CriticalSection::new() };
}
```

- `UnsafeCell<u32>` is the heart of interior mutability in Rust
- This design requires that the application pass a `CriticalSection` token in: these tokens are only safely generated by `interrupt::free`, so by requiring one be passed in, we ensure we are operating inside a critical section, _without_ having to actually do the lock ourselves.
- Brings our topic to `Send` and `Sync`.
  - A type is `Send` when it can safely be moved to another thread
  - `Sync` when it can be safely shared between multiple threads.
  - In an embedded context, we consider interrupts to be executing in a separate thread to the application code, so variables accessed by both an interrupt and the main code **must be Sync**.
  - `static` variables must be `Sync` since they can be accessed by multiple threads

## Mutexes

- Mutex constructs ensure exclusive access to a variable, such as our counter.
- A thread can attempt to lock (or acquire) the mutex, and either:
  - succeeds immediately
  - blocks waiting for the lock to be acquired
  - returns an error that the mutex could not be locked.
- In Rust, we would usually implement the unlock using the `Drop` trait to ensure it is always released when the mutex goes out of scope.
- It is **not** normally acceptable for the interrupt handler to block
- To avoid this behaviour entirely, we could implement a mutex which requires a critical section to lock, just like our counter example. So long as the critical section must last as long as the lock, we can be sure we have exclusive access to the wrapped variable without even needing to track the lock/unlock state of the mutex.
- This is done for us in the `cortex_m` crate!

```rust
use core::cell::Cell;
use cortex_m::interrupt::Mutex;

static COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

#[entry]
fn main() -> ! {
    set_timer_1hz();
    let mut last_state = false;
    loop {
        let state = read_signal_level();
        if state && !last_state {
            interrupt::free(|cs|
                COUNTER.borrow(cs).set(COUNTER.borrow(cs).get() + 1));
        }
        last_state = state;
    }
}

#[interrupt]
fn timer() {
    // We still need to enter a critical section here to satisfy the Mutex.
    interrupt::free(|cs| COUNTER.borrow(cs).set(0));
}
```

- We're now using `Cell`, which along with its sibling `RefCell` is used to provide **safe interior mutability**.
- A `Cell` is like an `UnsafeCell` but it provides a safe interface:
  - it only permits taking a copy of the current value or replacing it, not taking a reference
  - since it is not Sync, it cannot be shared between threads.
- We cannot use `Cell` directly with `static` as `static` variables must be `Sync`
- The `Mutex<T>` implements `Sync` for any `T` which is `Send` - such as a `Cell`
- For types that do not have the `Copy` trait, use `RefCell`

## Sharing Peripherals

- To safely share peripheral access, we can use the `Mutex` we saw before. We'll also need to use `RefCell`, which uses a runtime check to ensure only one reference to a peripheral is given out at a time.
- To move the peripheral into the shared variable after it has been initialised in the main code: use the `Option` type, initialised to `None` and later set to the instance of the peripheral.

```rust
use core::cell::RefCell;
use cortex_m::interrupt::{self, Mutex};
use stm32f4::stm32f405;

static MY_GPIO: Mutex<RefCell<Option<stm32f405::GPIOA>>> =
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // Obtain the peripheral singletons and configure it.
    // This example is from an svd2rust-generated crate, but
    // most embedded device crates will be similar.
    let dp = stm32f405::Peripherals::take().unwrap();
    let gpioa = &dp.GPIOA;

    // Some sort of configuration function.
    // Assume it sets PA0 to an input and PA1 to an output.
    configure_gpio(gpioa);

    // Store the GPIOA in the mutex, moving it.
    interrupt::free(|cs| MY_GPIO.borrow(cs).replace(Some(dp.GPIOA)));
    // We can no longer use `gpioa` or `dp.GPIOA`, and instead have to
    // access it via the mutex.

    // Be careful to enable the interrupt only after setting MY_GPIO:
    // otherwise the interrupt might fire while it still contains None,
    // and as-written (with `unwrap()`), it would panic.
    set_timer_1hz();
    let mut last_state = false;
    loop {
        // We'll now read state as a digital input, via the mutex
        let state = interrupt::free(|cs| {
            let gpioa = MY_GPIO.borrow(cs).borrow();
            gpioa.as_ref().unwrap().idr.read().idr0().bit_is_set()
        });

        if state && !last_state {
            // Set PA1 high if we've seen a rising edge on PA0.
            interrupt::free(|cs| {
                let gpioa = MY_GPIO.borrow(cs).borrow();
                gpioa.as_ref().unwrap().odr.modify(|_, w| w.odr1().set_bit());
            });
        }
        last_state = state;
    }
}

#[interrupt]
fn timer() {
    // This time in the interrupt we'll just clear PA0.
    interrupt::free(|cs| {
        // We can use `unwrap()` because we know the interrupt wasn't enabled
        // until after MY_GPIO was set; otherwise we should handle the potential
        // for a None value.
        let gpioa = MY_GPIO.borrow(cs).borrow();
        gpioa.as_ref().unwrap().odr.modify(|_, w| w.odr1().clear_bit());
    });
}
```

- The `Option` lets us initialise this variable to something empty, and only later actually move the variable in. We cannot access the peripheral singleton statically, only at runtime, so this is required.

```rust
interrupt::free(|cs| {
    let gpioa = MY_GPIO.borrow(cs).borrow();
    gpioa.as_ref().unwrap().odr.modify(|_, w| w.odr1().set_bit());
});
```

- Inside a critical section we can call `borrow()` on the mutex, which gives us a reference to the `RefCell`. We then call `replace()` to move our new value into the `RefCell`.
- Since we can't move the `GPIOA` out of the `&Option`, we need to convert it to an `&Option<&GPIOA>` with `as_ref()`, which we can finally `unwrap()` to obtain the `&GPIOA` which lets us modify the peripheral
- If we need a mutable reference to a shared resource, then `borrow_mut` and `deref_mut` should be used instead.

## RTIC

- One alternative is the RTIC framework, short for Real Time Interrupt-driven Concurrency.
- It enforces static priorities and tracks accesses to static mut variables ("resources") to statically ensure that shared resources are always accessed safely
