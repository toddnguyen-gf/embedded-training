# Chapter 9 - Tips for Embedded C Developers

## Preprocessor

Rust's alternatives to:

- Compile-time selection of code blocks with #ifdef
- Compile-time array sizes and computations
- Macros to simplify common patterns (to avoid function call overhead)

### Compile-time selection of code blocks with #ifdef

The closest match to `#ifdef ... #endif` in Rust are **Cargo features**.

- These are a little more formal than the C preprocessor: all possible features are explicitly listed per crate, and can only be either on or off.
- Features are turned on when you list a crate as a dependency, and are **additive**: if any crate in your dependency tree enables a feature for another crate, that feature will be **enabled for all users of that crate**.
- You could declare a Cargo feature for each component in your Cargo.toml. Example:

```toml
[features]
FIR = []
IIR = []
```

Then, in your code, use `#[cfg(feature="FIR")]` to control what is included.

```rust
/// In your top-level lib.rs

#[cfg(feature="FIR")]
pub mod fir;

#[cfg(feature="IIR")]
pub mod iir;
```

- You can similarly include code blocks only if a feature is **not** enabled, or if _any combination of features_ are or are not enabled.
- For full details of the conditional compilation support, refer to the [conditional compilation](https://doc.rust-lang.org/reference/conditional-compilation.html) chapter of the Rust reference.
- The conditional compilation will only apply to the next statement or block.
- Most of the time, it might be better to include ALL the code and allow the compiler to remove dead code when optimizing.

### Compile-time array sizes and computations

- fairly new feature (included in Rust 1.31)
- Rust supports `const fn`, functions which are **guaranteed to be evaluable at compile-time** and can therefore be used where constants are required, such as in the size of arrays. This can be used alongside features mentioned above, for example:

```rust
// const fn in use!
const fn array_size() -> usize {
    #[cfg(feature="use_more_ram")]
    { 1024 }
    #[cfg(not(feature="use_more_ram"))]
    { 128 }
}

static BUF: [u32; array_size()] = [0u32; array_size()];
```

### Macros to simplify common patterns

- There are two varieties of Rust macro: macros by example and procedural macros.

| Macros Type       | Description                                                                                                                               |
| ----------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| Macros by Example | Simpler and most common; they look like function calls and can expand to a complete expression, statement, item, or pattern               |
| Procedural Macros | More complex but permit extremely powerful additions to the Rust language: they can transform arbitrary Rust syntax into new Rust syntax. |

- In general, where you might have used a C preprocessor macro, you probably want to see if a macro-by-example can do the job instead.
  - macro by example can be defined in your crate!
  - Note that some use cases of C preprocessor macros will not work. For example, a macro that expands to part of a variable name or an incomplete set of items in a list.
- As with Cargo features, it is worth considering if you even need the macro. _In many cases a regular function is easier to understand_ and will be inlined to the same code as a macro.

### Build System

- Cargo provides [`build.rs` scripts](https://doc.rust-lang.org/cargo/reference/build-scripts.html) for this build customizations. They are Rust scripts which can interact with the Cargo build system as required.
- Common use cases for build scripts include:
  - provide build-time information, for example statically embedding the build date or Git commit hash into your executable
  - generate linker scripts at build time depending on selected features or other logic
  - change the Cargo build configuration
  - add extra static libraries to link against
- No support for post-build scripts yet

### Cross-Compiling

- In most cases it suffices to tell Cargo `--target thumbv6m-none-eabi`
- You can use [`Xargo`](https://github.com/japaric/xargo) on platforms that are not supported by Cargo to build `libcore`

---

## Iterators vs. Array Access

- In Rust, array access via its index is an anti-pattern! In Rust, indexed access can be slower **(as it needs to be bounds checked)** and may prevent various compiler optimisations.
- Use iterators instead!!!

```rust
let arr = [0u16; 16];
for element in arr.iter() {
    process(*element);
}
```

- Iterator methods can also be chained, giving very readable data processing code.
- See the [Iterators in the Book](https://doc.rust-lang.org/book/ch13-02-iterators.html) and [Iterator documentation](https://doc.rust-lang.org/core/iter/trait.Iterator.html) for more details.

## References vs. Pointers

- In Rust, pointers (called [raw pointers](https://doc.rust-lang.org/book/ch19-01-unsafe-rust.html#dereferencing-a-raw-pointer)) exist but are only used in specific circumstances, as **dereferencing them is always considered unsafe**
- In most cases, we instead use references, indicated by the `&` symbol, or mutable references, indicated by `&mut`.
- References behave similarly to pointers, in that they _can be dereferenced to access the underlying values_, but they are a key part of Rust's ownership system
  - Rust will strictly enforce that you may only have only one of the following to the same value at any given time:
    - one mutable reference, or
    - multiple non-mutable references
- One situation where you might still use raw pointers is interacting directly with hardware (for example, writing a pointer to a buffer into a DMA peripheral register)

## Volatile Access

- In C, individual variables may be marked `volatile`, indicating to the compiler that the value in the variable may change between accesses.
- In Rust, instead of marking a variable as volatile, we use specific methods to perform volatile access: `core::ptr::read_volatile` and `core::ptr::write_volatile`. These methods take a `*const T` or a `*mut T` (**raw pointers**, as discussed above) and perform a volatile read or write.

```rust
static mut SIGNALLED: bool = false;

#[interrupt]
fn ISR() {
    // Signal that the interrupt has occurred
    // (In real code, you should consider a higher level primitive,
    //  such as an atomic type).
    unsafe { core::ptr::write_volatile(&mut SIGNALLED, true) };
}

fn driver() {
    loop {
        // Sleep until signalled
        while unsafe { !core::ptr::read_volatile(&SIGNALLED) } {}
        // Reset signalled indicator
        unsafe { core::ptr::write_volatile(&mut SIGNALLED, false) };
        // Perform some task that was waiting for the interrupt
        run_task();
    }
}
```

- `&mut T` automatically converts to a `*mut T` (and the same for `*const T`)
- Note that for concurrency, there are better options available (see Chapter 06 - Concurrency)

## Packed and Aligned Types

- In Rust, alignment / struct packed / struct aligned is controlled by the `repr` attribute on a struct or union.
- To ensure layouts that are interoperable with C, use `repr(C)`:

```rust
#[repr(C)]
struct Foo {
    x: u16,
    y: u8,
    z: u16,
}

fn main() {
    let v = Foo { x: 0, y: 0, z: 0 };
    println!("{:p} {:p} {:p}", &v.x, &v.y, &v.z);
}

// 0x7fffd0d84c60 0x7fffd0d84c62 0x7fffd0d84c64
// Ordering is preserved and the layout will not change over time.
// `z` is two-byte aligned so a byte of padding exists between `y` and `z
```

- To ensure a packed representation, use `repr(packed)`. Note that using `repr(packed)` also sets the alignment of the type to `1`:

```rust
#[repr(packed)]
struct Foo {
    x: u16,
    y: u8,
    z: u16,
}

fn main() {
    let v = Foo { x: 0, y: 0, z: 0 };
    // Unsafe is required to borrow a field of a packed struct.
    unsafe { println!("{:p} {:p} {:p}", &v.x, &v.y, &v.z) };
}

// 0x7ffd33598490 0x7ffd33598492 0x7ffd33598493
// No padding has been inserted between `y` and `z`, so now `z` is unaligned.
```

- Finally, to specify a specific alignment, use `repr(align(n))`, where `n` is the number of bytes to align to (and **must be a power of two**):

```rust
#[repr(C)]
#[repr(align(4096))]
struct Foo {
    x: u16,
    y: u8,
    z: u16,
}

fn main() {
    let v = Foo { x: 0, y: 0, z: 0 };
    let u = Foo { x: 0, y: 0, z: 0 };
    println!("{:p} {:p} {:p}", &v.x, &v.y, &v.z);
    println!("{:p} {:p} {:p}", &u.x, &u.y, &u.z);
}

// 0x7ffec909a000 0x7ffec909a002 0x7ffec909a004
// 0x7ffec909b000 0x7ffec909b002 0x7ffec909b004
// The two instances `u` and `v` have been placed on 4096-byte alignments,
// evidenced by the `000` at the end of their addresses.
```

- Note we can combine `repr(C)` with `repr(align(n))` to obtain an aligned and C-compatible layout.
- It is NOT permissible to combine `repr(align(n))` with `repr(packed)`, since `repr(packed)` sets the alignment to 1.
  - It is also NOT permissible for a `repr(packed)` type to contain a `repr(align(n))` type.

| Type 1           | Type 2           | Permissible |
| ---------------- | ---------------- | ----------- |
| `repr(C)`        | `repr(align(n))` | Yes         |
| `repr(align(n))` | `repr(packed)`   | No          |
| `repr(packed)`   | `repr(align(n))` | No          |

- For further details on type layouts, refer to the [type layout](https://doc.rust-lang.org/reference/type-layout.html) chapter of the Rust Reference.
