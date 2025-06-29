# EpiMed Device

[![Rust](https://img.shields.io/badge/Rust-1.60%2B-orange.svg)](https://www.rust-lang.org/)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](LICENSE)

EpiMed Device is an embedded Rust project targeting the nRF52840 microcontroller. This project aims to develop a smart EpiPen device with Bluetooth Low Energy (BLE) capabilities for monitoring and communication, utilizing the Embassy framework and nRF SoftDevice for BLE functionality.

## Overview

This project leverages the power of Rust's safety guarantees and the Embassy framework's asynchronous runtime to create a reliable and efficient embedded application for medical device monitoring. The primary features include:

- **BLE Connectivity**: Implements BLE services for battery level monitoring and distance measurement.
- **Asynchronous Runtime**: Uses Embassy's executor for efficient task management on the nRF52840.
- **Custom Hardware Integration**: Tailored for the nRF52840-DK development kit with specific memory configurations.

## Repository Structure

- `src/`: Source code for the EpiMed Device application.
- `library/embassy/`: Submodule containing the Embassy framework for embedded Rust development.
- `library/nrf-softdevice/`: Submodule for nRF SoftDevice BLE stack integration.
- `Cargo.toml`: Project manifest with dependencies and feature configurations.
- `.cargo/config.toml`: Configuration for targeting the ARM Cortex-M4 architecture.

## Prerequisites

- **Rust Toolchain**: Version 1.60 or higher with support for embedded targets.
- **Cargo**: Rust's package manager and build tool.
- **nRF52840-DK**: Development kit for the nRF52840 microcontroller.
- **nRF Command Line Tools**: For flashing and debugging the device.

## Building the Project

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/EpiMed_Device.git
   cd EpiMed_Device
   git submodule update --init --recursive
   ```

2. **Build the Project**:
   Ensure you have the correct target installed for ARM Cortex-M4:
   ```bash
   rustup target add thumbv7em-none-eabihf
   cargo build --bin Epimed_Device --features nrf52840-dk
   ```

## Flashing the Device

To flash the built binary to your nRF52840-DK, use the nRF Command Line Tools or a compatible programmer:

```bash
nrfjprog --program target/thumbv7em-none-eabihf/debug/Epimed_Device -f NRF52 --sectorerase
nrfjprog --reset -f NRF52
```

## Running and Testing

After flashing, the device will start advertising as "EpiPen" over BLE. You can connect to it using a BLE scanner app or a custom application to monitor battery level and distance data.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request or open an Issue for any bugs, feature requests, or improvements.

## License

This project is licensed under the MIT License or Apache License 2.0, at your option. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [Embassy](https://github.com/embassy-rs/embassy) for the asynchronous embedded framework.
- [nRF SoftDevice](https://github.com/embassy-rs/nrf-softdevice) for BLE stack implementation.
- The Rust Embedded community for tools and resources.
