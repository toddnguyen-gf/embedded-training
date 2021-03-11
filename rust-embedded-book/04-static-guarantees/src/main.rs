pub mod foo_module {
    #[derive(Debug)]
    pub struct Foo {
        inner: u32,
    }

    pub struct FooBuilder {
        a: u32,
        b: u32,
    }

    impl FooBuilder {
        pub fn new(starter: u32) -> Self {
            Self {
                a: starter,
                b: starter,
            }
        }

        pub fn double_a(self) -> Self {
            Self {
                a: self.a * 2,
                b: self.b,
            }
        }

        pub fn into_foo(self) -> Foo {
            Foo {
                inner: self.a + self.b,
            }
        }
    }
}

fn main() {
    /*
    In this example, there is no direct way to create a Foo object. We must create a FooBuilder, and properly initialize it before we can obtain the Foo object we want.

    This minimal example encodes two states:
    - FooBuilder, which represents an "unconfigured", or "configuration in process" state
    - Foo, which represents a "configured", or "ready to use" state.

    In the process, FooBuider is destroyed.
    By creating a FooBuilder, and exchanging it for a Foo object, we have walked through the steps of a basic state machine.
    */
    let x = foo_module::FooBuilder::new(10).double_a().into_foo();

    println!("{:#?}", x);
}
