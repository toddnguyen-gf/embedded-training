# Hard Fault Handler

```openocd
ExceptionFrame {
    r0: 0x3ffffffe,
    r1: 0x00f00000,
    r2: 0x08004370,
    r3: 0x00000000,
    r12: 0x00000000,
    lr: 0x08000473,
    pc: 0x08000e92,
    xpsr: 0x61000200,
}
```

- The `pc` value is the value of the Program Counter at the time of the exception and it points to the instruction that triggered the exception.
