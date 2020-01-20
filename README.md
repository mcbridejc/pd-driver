# `pd-driver`

STM32 driver for HV507 control and electrode capacitance sensing, targetting the stmf411 nucleo-64 board. 

This was created from the [cortex-m-quickstart template](https://github.com/rust-embedded/cortex-m-quickstart) and the [Rust Embedded Book](https://rust-embedded.github.io/book). 

## Running

You can program and debug with the ST-LINK built into the Nucleo board using openocd. Just run `openocd` from the project directory to start the server. Once that's running, `cargo run` will download the program to the nucleo board using the built-in ST-LINK, and connect GDB, based on the settings in `.cargo/config`. 

## Local Documentation

`cargo doc --open` is useful. 