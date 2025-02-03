# Access SX127x chips on ATMEGA328
Under development!

This library crate i wrote for interfacing SX127x chips (tested with sx1276) with an Atmega328p. 

No Arduino necessary, just a barbone chip on a breadbord, running on internal oscillator @1Mhz.

The crate is rather independent of the actual hardware, it only uses embedded-hal. But there is a delay routing specific to atmega@1Mhz, this should be fixed some day.

The reason i wrote this is as an exercise with Rust on Atmega328p. The code is probably re-inventing a wheel the n-th time. 
But maybe this simple lib is useful to someone else due to its simplicity. 
