# Chapter 05 - Portability

- A common way to equalize such differences is via a layer called **Hardware Abstraction Layer** or **HAL**.

What is HAL?

- Hardware abstractions are sets of routines in software that emulate some platform-specific details, giving programs direct access to the hardware resources.
- They often allow programmers to write device-independent, high performance applications by providing standard operating system (OS) calls to hardware.

Wikipedia: [Hardware Abstraction Layer](https://en.wikipedia.org/wiki/Hardware_abstraction)

- Wikipedia's method is likely not the most productive approach to ensure portability.
- Enter `embedded-hal`!

## What is embedded-hal?

- In a nutshell it is a set of traits which define implementation contracts between HAL implementations, drivers and applications (or firmwares).
- Those contracts include both:
  - **capabilities** (i.e. if a trait is implemented for a certain type, the HAL implementation provides a certain capability)
  - **methods** (i.e. if you can construct a type implementing a trait it is guaranteed that you have the methods specified in the trait available).
- Some of the defined `traits` in embedded-hal are:
  - GPIO (input and output pins)
  - Serial communication
  - I2C
  - SPI
  - Timers/Countdowns
  - Analog Digital Conversion

Three main users of embedded-hal:

1.  HAL implementation
2.  Driver
3.  Application

**HAL implementation**

- A HAL implementation provides the interfacing between the hardware and the users of the HAL traits. Typical implementations consist of three parts:
  - One or more hardware specific types
  - Functions to create and initialize such a type, often providing various configuration options (speed, operation mode, use pins, etc.)
  - one or more `trait` `impl` of embedded-hal traits for that type

**Driver**

- A driver implements a set of custom functionality for an internal or external component, connected to a peripheral implementing the embedded-hal traits.
- A driver has to be initialized with an instance of type that implements a certain `trait` of the embedded-hal which is ensured via _trait bounds_
- A driver provides its own type instance with a custom set of methods allowing to interact with the driven device.

**Application**

- The application binds the various parts together and ensures that the desired functionality is achieved
