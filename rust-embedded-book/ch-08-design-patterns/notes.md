# Design Patterns

## HAL Design Patterns

### Checklist

- Naming (crate aligns with Rust naming conventions)
  - [ ] The crate is named appropriately (C-CRATE-NAME)
- Interoperability (crate interacts nicely with other library functionality)
  - [ ] Wrapper types provide a destructor method (C-FREE)
  - [ ] HALs reexport their register access crate (C-REEXPORT-PAC)
  - [ ] Types implement the embedded-hal traits (C-HAL-TRAITS)
- Predictability (crate enables legible code that acts how it looks)
  - [ ] Constructors are used instead of extension traits (C-CTOR)
- GPIO Interfaces (GPIO Interfaces follow a common pattern)
  - [ ] Pin types are zero-sized by default (C-ZST-PIN)
  - [ ] Pin types provide methods to erase pin and port (C-ERASED-PIN)
  - [ ] Pin state should be encoded as type parameters (C-PIN-STATE)

### Naming

- HAL crates should be named after the chip or family of chips they aim to support.
- Their name should end with `-hal` to distinguish them from register access crates.
- The name should not contain underscores (use dashes instead).

### Interoperability

- Wrapper types provide a destructor method (C-FREE)
  - Any non-`Copy` wrapper type provided by the HAL should provide a `free` method that consumes the wrapper and returns back the raw peripheral (and possibly other objects) it was created from.
  - The `free` method should shut down and reset the peripheral if necessary. Calling `new` with the raw peripheral returned by `free` should not fail due to an unexpected state of the peripheral.
  - If the HAL type requires other non-`Copy` objects to be constructed (for example I/O pins), any such object should be released and returned by `free` as well. `free` should return a tuple in that case.

Example:

```rust
pub struct Timer(TIMER0);

impl Timer {
    pub fn new(periph: TIMER0) -> Self {
        Self(periph)
    }

    /// free function consumes the wrapper and
    /// consumes the wrapper
    pub fn free(self) -> TIMER0 {
        self.0
    }
}
```

- HALs reexport their register access crate (C-REEXPORT-PAC)
  - HALs should always reexport the register access crate they are based on in their crate root.
  - A PAC should be reexported under the name pac, regardless of the actual name of the crate, as the name of the HAL should already make it clear what PAC is being accessed
- Types implement the embedded-hal traits (C-HAL-TRAITS)
  - Types provided by the HAL should implement all applicable traits provided by the embedded-hal crate.

### Predictability

- Constructors are used instead of extension traits (C-CTOR)
  - All peripherals to which the HAL adds functionality should be wrapped in a new type, even if no additional fields are required for that functionality.
  - Extension traits implemented for the _raw_ peripheral should be _avoided_.
- Methods are decorated with `#[inline]` where appropriate (C-INLINE)
  - This is related to optimization
  - As embedded applications are sensitive to unexpected code size increases, `#[inline]` should be used to guide the compiler as follows:
    - All "small" functions should be marked `#[inline]`. What qualifies as "small" is subjective, but generally all functions that are expected to compile down to **single-digit** instruction sequences qualify as small.
    - Functions that are very likely to take constant values as parameters should be marked as `#[inline]`. This enables the compiler to compute even complicated initialization logic at **compile time**, provided the function inputs are known.

### GPIO Interface

- Pin types are zero-sized by default (C-ZST-PIN)
  - GPIO Interfaces exposed by the HAL should provide dedicated zero-sized types for each pin on every interface or port, resulting in a zero-cost GPIO abstraction when all pin assignments are statically known.
  - Each GPIO Interface or Port should implement a split method returning a struct with every pin.

Example

```rust
/// Zero-sized types
pub struct PA0;
pub struct PA1;
// ...
/// End Zero-sized types

pub struct PortA;

impl PortA {
    /// Note that this consumes `self`
    pub fn split(self) -> PortAPins {
        PortAPins {
            pa0: PA0,
            pa1: PA1,
            // ...
        }
    }
}

pub struct PortAPins {
    pub pa0: PA0,
    pub pa1: PA1,
    // ...
}
```

- Pin types provide methods to `erase pin and port` (C-ERASED-PIN)
  - Pins should provide _type erasure methods_ that move their properties from compile time to runtime
  - Allow more flexibility in applications.
    Example:

```rust

/// Port A, pin 0.
pub struct PA0;

impl PA0 {
    /// Note that this consumes `self`
    pub fn erase_pin(self) -> PA {
        PA { pin: 0 }
    }
}

/// A pin on port A.
pub struct PA {
    /// The pin number.
    pin: u8,
}

impl PA {
    /// Note that this consumes `self`
    pub fn erase_port(self) -> Pin {
        Pin {
            port: Port::A,
            pin: self.pin,
        }
    }
}

pub struct Pin {
    port: Port,
    pin: u8,
    // (these fields can be packed to reduce the memory footprint)
}

enum Port {
    A,
    B,
    C,
    D,
}
```

- Pin state should be encoded as type parameters (C-PIN-STATE)
  - Pins may be configured as input or output with different characteristics depending on the chip or family. This state should be encoded in the **type system**
    - Additional, chip-specific state (eg. drive strength) may also be encoded in this way, using additional type parameters.
  - `into_input` and `into_output`: Methods for changing the pin state should be provided as `into_input` and `into_output` methods.
    - Additionally, `with_{input,output}_state` methods should be provided that **temporarily** reconfigure a pin in a different state **without moving it**.
  - The following methods should be provided for every pin type (that is, both erased and non-erased pin types should provide the same API):

```rust
// 1
pub fn into_input<N: InputState>(self, input: N) -> Pin<N>
// 2
pub fn into_output<N: OutputState>(self, output: N) -> Pin<N>

// 3
pub fn with_input_state<N: InputState, R>(
    &mut self,
    input: N,
    f: impl FnOnce(&mut PA1<N>) -> R,
) -> R

// 4
pub fn with_output_state<N: OutputState, R>(
    &mut self,
    output: N,
    f: impl FnOnce(&mut PA1<N>) -> R,
) -> R
```

- Pin state should be bounded by `sealed` traits. Users of the HAL should have no need to add their own state.
- The traits can provide HAL-specific methods required to implement the pin state API.

Example:

```rust
// Sealed trait
mod sealed {
    pub trait Sealed {}
}

// Pin states being bounded by seal traits
pub trait PinState: sealed::Sealed {}
pub trait OutputState: sealed::Sealed {}
pub trait InputState: sealed::Sealed {
    // ...
}

pub struct Output<S: OutputState> {
    _p: PhantomData<S>,
}

impl<S: OutputState> PinState for Output<S> {}
impl<S: OutputState> sealed::Sealed for Output<S> {}

pub struct PushPull;
pub struct OpenDrain;

impl OutputState for PushPull {}
impl OutputState for OpenDrain {}
impl sealed::Sealed for PushPull {}
impl sealed::Sealed for OpenDrain {}

pub struct Input<S: InputState> {
    _p: PhantomData<S>,
}

impl<S: InputState> PinState for Input<S> {}
impl<S: InputState> sealed::Sealed for Input<S> {}

pub struct Floating;
pub struct PullUp;
pub struct PullDown;

impl InputState for Floating {}
impl InputState for PullUp {}
impl InputState for PullDown {}
impl sealed::Sealed for Floating {}
impl sealed::Sealed for PullUp {}
impl sealed::Sealed for PullDown {}

pub struct PA1<S: PinState> {
    _p: PhantomData<S>,
}

/// Implement the 4 required functions:
/// 1. into_input
/// 2. into_output
/// 3. with_input_state
/// 4. with_output_state
impl<S: PinState> PA1<S> {
    /// Note that this consumes `self`
    pub fn into_input<N: InputState>(self, input: N) -> PA1<Input<N>> {
        todo!()
    }

    /// Note that this consumes `self`
    pub fn into_output<N: OutputState>(self, output: N) -> PA1<Output<N>> {
        todo!()
    }

    pub fn with_input_state<N: InputState, R>(
        &mut self,
        input: N,
        f: impl FnOnce(&mut PA1<N>) -> R,
    ) -> R {
        todo!()
    }

    pub fn with_output_state<N: OutputState, R>(
        &mut self,
        output: N,
        f: impl FnOnce(&mut PA1<N>) -> R,
    ) -> R {
        todo!()
    }
}

// Same for `PA` and `Pin`, and other pin types.
```
