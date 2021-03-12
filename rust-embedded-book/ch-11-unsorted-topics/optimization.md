# Optimizations: The speed size tradeoff

This section discusses the different optimization levels that rustc provides and how they affect the execution time and binary size of a program.

## No Optimizations

- This is the DEFAULT
- `cargo build` uses `-C opt-level = 0`
- At least for bare metal development, debuginfo is zero cost in the sense that it won't occupy space in Flash / ROM so we actually recommend that you enable debuginfo in the release profile

```toml
[profile.release]
# symbols are nice and they don't increase the size on Flash
debug = true
# debug is false by default
```

- No optimizations is great for debugging because stepping through the code feels like you are executing the program statement by statement
  - and you can `print` variables!
- The biggest downside of the `dev` profile is that the resulting binary will be _huge and slow_.

### Optimizing Dependencies

- There's a Cargo feature named `profile-overrides` that lets you override the optimization level of _dependencies_.
  - You can use that feature to **optimize all dependencies for size** while keeping the top crate unoptimized and debugger friendly.
- Override example

```toml
# Cargo.toml
[package]
name = "app"
# ..

# Explicitly set optimization level of ALL packages
[profile.dev.package."*"] # +
opt-level = "z" # +
```

- Result? A 6 KiB reduction in Flash usage without any loss in the debuggability of the top crate.
  - Downside? We cannot debug dependencies. Chances are though that you'll want to debug the top-level crate instead of its dependencies
  - If you want to debug dependencies, explicitly set its optimization level

```toml
# Cargo.toml
# ...

# don't optimize the `cortex-m-rt` crate
# cortex-m-rt is debugger friendly!
[profile.dev.package.cortex-m-rt] # +
opt-level = 0 # +

# but do optimize all the other dependencies
[profile.dev.package."*"]
codegen-units = 1 # better optimizations
opt-level = "z"
```

## Optimize for Speed

- As of 2018-09-18 `rustc` supports three "optimize for speed" levels: `opt-level = 1`, `2` and `3`.
  - `--release` === `opt-level 3`
- Both `opt-level = 2` and `3` optimize for speed at the expense of binary size, but level `3` does more **vectorization and inlining** than level `2`.
- `opt-level` 2+ WILL enable loop unrolling. There is no way to disable this.
  - If the increase in size is too much because of this, consider optimizing for size instead.

## Optimize for Size

- As of 2018-09-18 rustc supports two "optimize for size" levels: `opt-level = "s"` and `"z"`
  - `"z"` is meant to give the idea that it produces **smaller binaries** than `"s"`.
- If you want your release binaries to be optimized for size then change the `profile.release.opt-level`:

```toml
[profile.release]
# or "z"
opt-level = "s"
```

- These two optimization levels greatly reduce LLVM's **inline threshold**, a metric used to decide whether to inline a function or not.
- When optimizing for size you may want to try _increasing the inline threshold_ to see if that has any effect on the binary size.
  - The recommended way to change the inline threshold is to append the `-C inline-threshold` flag to the other rustflags in .cargo/config.

```toml
# .cargo/config
# this assumes that you are using the cortex-m-quickstart template
[target.'cfg(all(target_arch = "arm", target_os = "none"))']
rustflags = [
  # ..
  "-C", "inline-threshold=123", # +
]
```

- What are the `inline-threshold` values? As of [1.29.0](https://github.com/rust-lang/rust/blob/1.29.0/src/librustc_codegen_llvm/back/write.rs#L2105-L2122):
  - Speed:
    - `opt-level = 3` uses `275`
    - `opt-level = 2` uses `225`
  - Size:
    - `opt-level = "s"` uses `75`
    - `opt-level = "z"` uses `25`
