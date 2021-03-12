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

---

## A little C with your Rust

Using C or C++ inside of a Rust project consists of two major parts:

- Wrapping the exposed C API for use with Rust
- Building your C or C++ code to be integrated with the Rust code

It is recommended to use the `C ABI` when combining Rust with C or C++.

### Defining an Interface

In Rust, it is necessary to either manually translate header definitions found in header files to Rust, or use a tool to generate these definitions.

#### Wrapping C Functions and Datatypes

```C
/* File: cool.h */
typedef struct CoolStruct {
    int x;
    int y;
} CoolStruct;

void cool_function(int i, char c, CoolStruct* cs);
```

In Rust:

```rust
/* File: cool_bindings.rs */

use cty;

#[repr(C)]
pub struct CoolStruct {
    pub x: cty::c_int,
    pub y: cty::c_int,
}

// Similar to a `header` file's signature without defining the body of the function
pub extern "C" fn cool_function(
    i: cty::c_int,
    c: cty::c_char,
    cs: *mut CoolStruct
);
```

- `repr(C)` instructs the Rust compiler to always use the same rules C does for organizing data within a struct.
- It is recommended to use primitive data types defined in `cty`, which will map types from C to types in Rust
- `pub extern "C" fn cool_function( ... );` This statement defines the signature of a function that uses the C ABI, `called cool_function`. By defining the signature without defining the body of the function, the definition of this function will need to be provided elsewhere, or linked into the final library or binary from a static library.
- We have one new type here, `*mut CoolStruct`. As C does not have a concept of Rust's references, which would look like this: `&mut CoolStruct`, we instead have a `raw pointer`. As dereferencing this pointer is `unsafe`, and the pointer may in fact be a null pointer, **care must be taken to ensure the guarantees typical of Rust when interacting with C or C++ code**.

#### Automatically generating the interface

Use [bindgen](https://github.com/rust-lang/rust-bindgen)! [bindgen's manual](https://rust-lang.github.io/rust-bindgen/).

Typical Process:

1. Gather all C or C++ headers defining interfaces or datatypes you would like to use with Rust
2. Write a `bindings.h` file, which `#include "..."'s` each of the files you gathered in step one.
3. Feed this `bindings.h` file, along with any _compilation flags_ used to compile your code into bindgen.
   - Tip: use `Builder.ctypes_prefix("cty")` / `--ctypes-prefix=cty` and `Builder.use_core()` / `--use-core` to make the generated code `#![no_std]` compatible.
4. `bindgen` will produce the generated Rust code to the output of the terminal window. This file may be piped to a file in your project, such as `bindings.rs`. You may use this file in your Rust project to interact with C/C++ code compiled and linked as an external library.
   - Tip: don't forget to use the `cty` crate if your types in the generated bindings are prefixed with `cty`.

| Type               | `cty`                          | `core`               |
| ------------------ | ------------------------------ | -------------------- |
| In code            | `Builder.ctypes_prefix("cty")` | `Builder.use_core()` |
| Command line flags | `--ctypes-prefix=cty`          | `--use-core`         |

### Building your C/C++ Code

- It is necessary to compile your non-Rust code ahead of time.
  - For embedded projects, this most commonly means compiling the C/C++ code to a **static archive** (such as `cool-library.a`), which can then be combined with your Rust code at the final linking step.
  - If the static archive is already distributed, it is not necessary to rebuild the code. Simply convert the provided interface header file as described above, and include the static archive file at compile / link time.
- If your code exists as a source project, it will be necessary to compile your C/C++ code to a static library, either by:
  - triggering your existing build system (such as make, CMake, etc.)
  - by porting the necessary compilation steps to use a tool called the `cc` crate.
- For both of these steps, it is necessary to use a `build.rs` script.

#### Rust build.rs script

- A `build.rs` script is a file written in Rust syntax, that is executed on your compilation machine, **AFTER dependencies** of your project have been built, but **BEFORE your project** is built.
- `build.rs` scripts are useful for generating code (such as via [bindgen](https://github.com/rust-lang/rust-bindgen)), calling out to external build systems such as Make, or directly compiling C/C++ through use of the `cc` crate
- `build.rs` runs on your local machine so you can use any crates that can run on the compilation host.

#### Triggering external build systems

- For projects with complex external projects or build systems, it may be easiest to use [`std::process::Command`](https://doc.rust-lang.org/std/process/struct.Command.html) to call other commands, such as `make`, etc.

#### Build C/C++ with `cc` crate

- the [`cc` crate](https://github.com/alexcrichton/cc-rs), which provides an idiomatic Rust interface to the compiler provided by the host.
- example `build.rs`:

```rust
// build.rs
use cc;

fn main() {
    cc::Build::new()
        .file("foo.c")
        .compile("libfoo.a");
}
```
