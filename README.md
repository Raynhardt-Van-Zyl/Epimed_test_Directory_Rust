# EpiMed Device

[![Rust](https://img.shields.io/badge/Rust-1.60%2B-orange.svg)](https://www.rust-lang.org/)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](LICENSE)

EpiMed Device is an embedded Rust project targeting the nRF52840 microcontroller. This project aims to develop a smart EpiPen device with Bluetooth Low Energy (BLE) capabilities for monitoring and communication, utilizing the Embassy framework and nRF SoftDevice for BLE functionality.

## Overview

This project leverages the power of Rust's safety guarantees and the Embassy framework's asynchronous runtime to create a reliable and efficient embedded application for medical device monitoring. The primary features include:

- **BLE Connectivity**: Implements BLE services for battery level monitoring and distance measurement.
- **Power Saving Mode Monitoring**: Indicates when the device is in a dormant state (sleep mode) to extend battery life by reducing power usage.
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

## Future Power Saving Enhancements

To further optimize power consumption and extend battery life for the EpiMed Device, the following enhancements are planned:

- **Dynamic BLE Advertising Intervals**: Adjust BLE advertising intervals based on usage patterns or battery level to reduce power usage during periods of low activity.
- **Peripheral Power Management**: Disable or put peripheral components (like sensors or LEDs) into low power states when not in use.
- **Advanced Sleep Modes**: Explore deeper sleep modes such as System OFF with RAM retention for minimal power draw, waking only on critical events.
- **Power-Aware Task Scheduling**: Implement task scheduling that prioritizes power efficiency, running intensive tasks only when necessary.
- **Battery Level Adaptive Behavior**: Modify device behavior based on battery level, reducing functionality or frequency of operations as battery depletes.
- **Wake-Up Source Optimization**: Configure specific wake-up sources (e.g., GPIO interrupts for user interaction or BLE connection requests) to minimize unnecessary wake-ups.

## Acknowledgments

- [Embassy](https://github.com/embassy-rs/embassy) for the asynchronous embedded framework.
- [nRF SoftDevice](https://github.com/embassy-rs/nrf-softdevice) for BLE stack implementation.
- The Rust Embedded community for tools and resources.

## Code Functionality (`src/main.rs`)

The main application logic in `src/main.rs` is built on the `embassy` asynchronous framework and the `nrf-softdevice` for BLE capabilities. It defines two primary BLE services, manages their state through concurrent tasks, and includes power-saving features.

### Key Components:

*   **BLE GATT Services**:
    *   **`BatteryService`**: Exposes the device's battery level and power-saving mode.
        *   `battery_level` (`u8`): A readable and notifiable characteristic representing the battery percentage.
        *   `power_saving_mode` (`u8`): A readable and notifiable characteristic indicating the power state (0 for normal, 1 for sleep).
    *   **`DistanceService`**: A custom service for distance measurement.
        *   `distance` (`f32`): A readable and notifiable characteristic that simulates a changing distance value.

*   **Asynchronous Tasks**:
    *   **`main`**: The entry point of the application. It initializes the Embassy framework, configures and enables the nRF SoftDevice, sets up a GPIO for the LED, and spawns all other application tasks.
    *   **`softdevice_task`**: A critical task that runs the BLE SoftDevice, which manages the radio and BLE protocol stack.
    *   **`led_task`**: A simple task that blinks an on-board LED to provide a visual indication that the device is running.
    *   **`bluetooth_task`**: Handles BLE advertising and connection management. When a device connects, it runs the GATT server and is intended to run the `distance_update_task` and `power_saving_task` concurrently.
    *   **`distance_update_task`**: Periodically updates the `distance` characteristic, simulating a sensor reading. It notifies connected clients of the new value.
    *   **`power_saving_task`**: Manages the device's power state. It simulates a gradual decrease in battery level and, after a period of inactivity, attempts to put the device into a low-power sleep mode by calling `nrf_softdevice::raw::sd_power_system_off()`.

## Power and Efficiency Notes from `embassy-nrf` and `nrf-softdevice`

This section summarizes key power-saving and efficiency features available in the underlying libraries, which can be leveraged for future optimizations.

### Core Concepts (Embassy & nRF)

*   **Async & Low Power**: Embassy's `async` model is fundamental to power saving. It allows the CPU to enter a low-power sleep state while waiting for long-running peripheral operations (like DMA transfers) to complete, waking only when necessary.
*   **PPI/DPPI (Programmable Peripheral Interconnect)**: This powerful nRF feature, exposed via `embassy-nrf::ppi`, allows peripherals to interact directly without CPU intervention. For example, a timer can trigger a SAADC sample, or a GPIO event can start a DMA transfer, all while the CPU remains asleep. This is crucial for building complex, ultra-low-power applications.
*   **EasyDMA**: Peripherals with EasyDMA access RAM directly, offloading data transfer tasks from the CPU. The `embassy-nrf` drivers use this extensively. For best performance and efficiency, ensure data buffers are in RAM to avoid runtime copies from flash.

### System Power States

*   **System OFF**: The deepest low-power state can be entered using `embassy_nrf::power::set_system_off()`.
*   **Wake-up Sources**: The device can wake from System OFF on various events. The cause of the wake-up can be identified via `embassy_nrf::reset::read_reasons()`. Key low-power wake-up sources include GPIO pin changes (DETECT signal), LPCOMP, NFC field detection, and VBUS detection.

### Clock Management

*   **Low-Frequency Clock (LFCLK)**: The choice of the LFCLK source is critical. Using an external 32.768 kHz crystal (`LfclkSource::ExternalXtal`) is significantly more power-efficient and accurate for low-power tasks than the internal RC oscillator. This is configured in `embassy-nrf::init` and `nrf-softdevice::enable`.

### Peripheral-Specific Optimizations

*   **DCDC Regulators**: On supported chips (like the nRF52840), enabling the DCDC regulators in the `embassy-nrf::config::Config` is more efficient than the default LDO regulators, especially at higher CPU loads or when the radio is active.
*   **RADIO (BLE)**:
    *   **TX Power**: Use the lowest acceptable transmission power, configurable in `nrf_softdevice::ble::peripheral::Config`.
    *   **Connection Parameters**: For peripherals, tuning the connection interval, slave latency, and supervision timeout in `ble_gap_conn_params_t` is crucial. Longer intervals and higher latency allow the device to sleep more.
    *   **Advertising**: Longer advertising intervals and using timeouts reduce power consumption.
*   **QSPI**: The `embassy-nrf::qspi::DeepPowerDownConfig` can put external flash memory into a low-power state.
*   **GPIO**: Use standard drive strength (`OutputDrive::Standard`) unless higher drive is necessary, as high-drive modes consume more power.
*   **WDT (Watchdog Timer)**: Can be configured to pause during sleep modes via `Config::action_during_sleep`.
