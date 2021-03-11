# Ch 07 - Collections

- As core is, by definition, free of memory allocations these implementations are not available there, but they can be found in the unstable `alloc` crate that's shipped with the compiler.
- If you need collections, a heap allocated implementation is not your only option. You can also use fixed capacity collections; one such implementation can be found in the `heapless` crate.

## Using alloc

- A self-cojntained version of `alloc` that has many `unsafe`!

```rust
// Bump pointer allocator implementation

extern crate cortex_m;

use core::alloc::GlobalAlloc;
use core::ptr;

use cortex_m::interrupt;

// Bump pointer allocator for *single* core systems
struct BumpPointerAlloc {
    head: UnsafeCell<usize>,
    end: usize,
}

unsafe impl Sync for BumpPointerAlloc {}

unsafe impl GlobalAlloc for BumpPointerAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        // `interrupt::free` is a critical section that makes our allocator safe
        // to use from within interrupts
        interrupt::free(|_| {
            let head = self.head.get();
            let size = layout.size();
            let align = layout.align();
            let align_mask = !(align - 1);

            // move start up to the next alignment boundary
            let start = (*head + align - 1) & align_mask;

            if start + size > self.end {
                // a null pointer signal an Out Of Memory condition
                ptr::null_mut()
            } else {
                *head = start + size;
                start as *mut u8
            }
        })
    }

    unsafe fn dealloc(&self, _: *mut u8, _: Layout) {
        // this allocator never deallocates memory
    }
}

// Declaration of the global memory allocator
// NOTE the user must ensure that the memory region `[0x2000_0100, 0x2000_0200]`
// is not used by other parts of the program
#[global_allocator]
static HEAP: BumpPointerAlloc = BumpPointerAlloc {
    head: UnsafeCell::new(0x2000_0100),
    end: 0x2000_0200,
};
```

- Users need to define how Out of Memory (OOM) errors are handled using the _unstable_ `alloc_error_handler`

```rust
#![feature(alloc_error_handler)]

use cortex_m::asm;

#[alloc_error_handler]
fn on_oom(_layout: Layout) -> ! {
    asm::bkpt();

    loop {}
}
```

## Using heapless

- `heapless` requires no setup as its collections don't depend on a global memory allocator. Just use its collections and proceed to instantiate them.
- Two differences between `heapless` and `alloc`
  - First, you have to declare upfront the capacity of the collection. `heapless` collections **never** reallocate and have **fixed** capacities
  - Second, the `push` method, and many other methods, return a `Result`. Since the `heapless` collections have fixed capacity all operations that insert elements into the collection can potentially fail. In contrast, `alloc` collections will reallocate themselves on the heap to increase their capacity.

## Tradeoffs

### Out of Memory and Error Handling

- With heap allocations Out Of Memory (OOM) is always a possibility and can occur in any place where a collection may need to grow
- `heapless` can never run out of memory. Instead, you'll have to deal with collections running out of capacity on a case by case basis (the returned `Result<>`)
- OOM can be harder to debug than unwrapping `Results`

### Memory Usage

- Reasoning about memory usage of heap allocated collections is hard because the capacity of long lived collections can change at runtime
- Ultimately, it's up to the allocator to decide whether to actually shrink the memory allocation or not
- On the other hand if you exclusively use fixed capacity collections, store most of them in `static` variables and set a maximum size for the call stack then the linker will detect if you try to use more memory than what's physically available.
- However, fixed capacity collections can not be shrunk which can result in lower load factors

### Worst Case Execution Time (WCET)

- hard to determine the WCET of, for example, the `alloc::Vec.push` operation as it depends on both the allocator being used and its runtime capacity.
- On the other hand fixed capacity collections never reallocate so all operations have a predictable execution time. For example, `heapless::Vec.push` executes in constant time.

### Ease of Use

- `alloc` requires setting up a global allocator whereas `heapless` does not.
- `heapless` requires you to pick the capacity of each collection that you instantiate.
