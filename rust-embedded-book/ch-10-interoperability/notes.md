# Ch 10 - Interoperability

- Interoperability between Rust and C code is always dependent on transforming data between the two languages.
- For this purposes there are two dedicated modules in the ``stdlib` called [`std::ffi`](https://doc.rust-lang.org/std/ffi/index.html) and [`std::os::raw`](https://doc.rust-lang.org/std/os/raw/index.html).

  - `std::os::raw` deals with low-level primitive types that can be converted _implicitly_ by the compiler
  - `std::ffi` provides some utility for converting more complex types such as Strings, mapping both `&str` and `String` to C-types that are easier and safer to handle.
  - Neither of these modules are available in `core`, but you can find a `#![no_std]` compatible version of:
    - `std::ffi::{CStr,CString}` in the [`cstr_core`](https://crates.io/crates/cstr_core) crate
    - most of the `std::os::raw` types in the [`cty`](https://crates.io/crates/cty) crate.

- Intermediate can be either the `cstr_core` or `cty` crate

| Rust type      | Intermediate | C type         |
| -------------- | ------------ | -------------- |
| `String`       | `CString`    | `*char`        |
| `&str`         | `CStr`       | `*const char`  |
| `()`           | `c_void`     | `void`         |
| `u32` or `u64` | `c_uint`     | `unsigned int` |

### Interoperability with other systems

- A common requirement for including Rust in your embedded project is combining Cargo with your existing build system, such as `make` or `cmake`.
- Check on [issue #61](https://github.com/rust-embedded/book/issues/61)

### Interoperability with RTOSs

- Integrating Rust with an RTOS such as FreeRTOS or ChibiOS is still a work in progress
- Check on [issue #62](https://github.com/rust-embedded/book/issues/62)
