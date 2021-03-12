# A little Rust with your C

Using Rust code inside a C or C++ project mostly consists of two parts.

- Creating a C-friendly API in Rust
- Embedding your Rust project into an external build system

## Setting up a Project

- There are flags to tell `cargo` to emit a systems library, instead of its regular rust target.
- This also allows you to set a different output name for your library, if you want it to differ from the rest of your crate.

```toml
[lib]
name = "your_crate"
crate-type = ["cdylib"]      # Creates dynamic lib
# crate-type = ["staticlib"] # Creates static lib
```

## Building a C API

- Because C++ has no stable ABI for the Rust compiler to target, we use C for any interoperability between different languages.
- `#[no_mangle]` → any function that Rust exports to be used outside of Rust needs to be told not to be mangled by the compiler.
- `extern "C"` → we need to tell the compiler to use the System ABI instead of the Rust ABI
  - We might need to target a specific ABI version. Documentation can be found [here](https://doc.rust-lang.org/reference/items/external-blocks.html)
- Putting it all together...

```rust
#[no_mangle]
pub extern "C" fn rust_function() {

}
```

We also need to transform data from and to a form that the rest of the application will understand.

## Linking and greater project context

- cargo will create a `my_lib.so/my_lib.dll` or `my_lib.a` file, depending on your platform and settings. This library can simply be linked by your build system.
- Every function in your Rust-ffi API **needs** to have a corresponding header function.

```rust
#[no_mangle]
pub extern "C" fn rust_function() {}
```

would then become

```C++
void rust_function();
```

- There is a tool to automate this process, called [cbindgen](https://github.com/eqrion/cbindgen) which analyses your Rust code and then generates headers for your C and C++ projects from it

- Then, to call your function from C/C++

```C++
#include "my-rust-project.h"
rust_function();
```
