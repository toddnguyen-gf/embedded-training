# Static Guarantees

- Rust's type system prevents data races at compile time (see `Send` and `Sync` traits).
- The type system can also be used to check other properties at compile time; reducing the need for runtime checks in some cases.
