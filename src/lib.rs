#![no_std]
//#![no_main]
#![feature(abi_avr_interrupt)]
#![feature(asm_experimental_arch)]

mod sx127x;

pub use sx127x::SX127x;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
