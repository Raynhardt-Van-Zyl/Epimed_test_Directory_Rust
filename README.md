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
- **VL53L4CD Distance Sensor**: Integrated STMicroelectronics VL53L4CD ToF sensor with advanced power management.

## Repository Structure

- `src/`: Source code for the EpiMed Device application.
- `library/embassy/`: Submodule containing the Embassy framework for embedded Rust development.
- `library/nrf-softdevice/`: Submodule for nRF SoftDevice BLE stack integration.
- `Cargo.toml`: Project manifest with dependencies and feature configurations.
- `.cargo/config.toml`: Configuration for targeting the ARM Cortex-M4 architecture.

## Hardware Components

### Primary MCU: nRF52840
- ARM Cortex-M4F @ 64MHz
- 1MB Flash, 256KB RAM
- Bluetooth 5.0 Low Energy
- Optimized for ultra-low power operation

### Distance Sensor: STMicroelectronics VL53L4CD
- Time-of-Flight (ToF) ranging sensor
- Operating range: 1mm to 1.3m
- Ultra-low power consumption with multiple power states
- I2C interface with hardware shutdown control

### Sensor Functionality

Based on the `EpiMed.ino` example, the VL53L4CD Time-of-Flight sensor is integrated into the system with the following functionality:

- **Initialization**: The sensor is initialized on startup using the `begin()` and `InitSensor()` methods, preparing it for distance measurements.
- **Configuration**: The ranging timing is configured using `VL53L4CD_SetRangeTiming()`, which in the example is set to 200ms. This allows for a balance between measurement frequency and power consumption.
- **Measurement Control**: The sensor's ranging can be started and stopped using the `VL53L4CD_StartRanging()` and `VL53L4CD_Off()` methods, respectively. This allows for on-demand measurements, which is crucial for power-saving.
- **Data Acquisition**: Distance readings are obtained by calling the `VL53L4CD_GetResult()` method, which returns the distance in millimeters.
- **Interrupts**: The system clears sensor interrupts using `VL53L4CD_ClearInterrupt()`, which is important for ensuring that new measurements can be taken.
- **Power Management**: The sensor's hardware shutdown pin (`XSHUT_PIN`) is used to completely power off the sensor, reducing power consumption to a minimum when not in use.

### Battery: 110mAh Li-ion
- Nominal voltage: 3.7V
- Operating range: 3.0V - 4.2V
- Optimized power management for extended operation

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

After flashing, the device will start advertising as "EpiMed" over BLE. You can connect to it using a BLE scanner app or a custom application to monitor battery level and distance data.

## Power Management & Battery Life Analysis

### VL53L4CD Power Consumption Profile

The STMicroelectronics VL53L4CD sensor has multiple power states optimized for different operation modes:

| Power State | Current Consumption | Use Case |
|-------------|-------------------|----------|
| **Powered Off (XSHUT low)** | <5µA | Maximum power savings during system sleep |
| **Hardware Standby** | 55µA | Ready for quick measurement, I2C inactive |
| **Ultra-Low Power (ULP)** | <100µA | Proximity detection only |
| **Active Measurement** | 300-600µA | Full ToF ranging operation |

### System Power Consumption Breakdown

**Total System Current (Typical Operation):**
- nRF52840 (optimized): 2-20µA
- BLE Stack (optimized): 1.5-15µA  
- VL53L4CD (duty cycle dependent): 5-450µA
- **Total Range: 8.5µA - 485µA**

### Battery Life Scenarios for 110mAh Battery

**Scenario 1 - Ultra Low Power** (Sensor mostly powered off):
- Components: nRF52840 (2µA) + BLE (1.5µA) + VL53L4CD (2µA)
- **Total: ~5.5µA = ≈680 days battery life**

**Scenario 2 - Moderate Usage** (5% measurement duty cycle):
- Components: nRF52840 (10µA) + BLE (5µA) + VL53L4CD (75µA weighted avg)
- **Total: ~90µA = ≈43 days battery life**

**Scenario 3 - Active Monitoring** (20% measurement duty cycle):
- Components: nRF52840 (20µA) + BLE (15µA) + VL53L4CD (134µA weighted avg)
- **Total: ~169µA = ≈23 days battery life**

**Scenario 4 - Continuous Operation** (100% active measurements):
- Components: nRF52840 (20µA) + BLE (15µA) + VL53L4CD (450µA continuous)
- **Total: ~485µA = ≈8 days battery life**

### Power Optimization Strategies

#### 1. Adaptive Sensor Power Management
- **Critical Battery (≤5%)**: Complete sensor shutdown (<5µA)
- **Low Battery (6-15%)**: ULP mode only (<100µA) 
- **Moderate Battery (16-50%)**: Hardware standby (55µA) with periodic measurements
- **Normal Operation (51-100%)**: Active measurements every 30 seconds

#### 2. BLE Optimization
- **Advertising Interval**: Increased from 100ms to 1000ms (10x power reduction)
- **Connection Parameters**: 500-1000ms intervals with higher slave latency
- **TX Power**: Reduced to -12dBm for adequate range with lower power consumption

#### 3. Measurement Duty Cycle Control
- **Normal Operation**: 20ms measurement every 30 seconds = 0.067% duty cycle
- **On-Demand**: Measurements triggered only by BLE requests or specific events
- **Sleep Mode**: Complete sensor shutdown during system OFF state

#### 4. Hardware Power Control
- **XSHUT Pin**: Complete sensor power-off when not needed
- **DCDC Regulators**: Enabled for improved power efficiency vs LDO
- **32.768kHz Crystal**: External crystal for accurate, low-power timekeeping

## Code Functionality (`src/main.rs`)

The main application logic in `src/main.rs` is built on the `embassy` asynchronous framework and the `nrf-softdevice` for BLE capabilities. It defines two primary BLE services, manages their state through concurrent tasks, and includes advanced power-saving features.

### Key Components:

*   **BLE GATT Services**:
    *   **`BatteryService`**: Exposes the device's battery level and power-saving mode.
        *   `battery_level` (`u8`): A readable and notifiable characteristic representing the battery percentage.
        *   `power_saving_mode` (`u8`): A readable and notifiable characteristic indicating the power state (0 for normal, 1 for sleep).
    *   **`DistanceService`**: A custom service for distance measurement.
        *   `distance` (`f32`): A readable and notifiable characteristic that provides ToF sensor readings.

*   **Asynchronous Tasks**:
    *   **`main`**: The entry point of the application. It initializes the Embassy framework, configures and enables the nRF SoftDevice, sets up GPIO for LED and sensor control, and spawns all other application tasks.
    *   **`softdevice_task`**: A critical task that runs the BLE SoftDevice, which manages the radio and BLE protocol stack.
    *   **`led_task`**: Power-optimized LED control task with configurable blink patterns for different power modes.
    *   **`bluetooth_task`**: Handles BLE advertising and connection management with optimized parameters for 110mAh battery operation.
    *   **`vl53l4cd_power_management_task`**: Advanced power management task that coordinates nRF52840, BLE, and VL53L4CD power states based on battery level and activity.
    *   **`distance_update_task`**: Manages ToF sensor measurements with adaptive scheduling based on power constraints.

### Power Management Implementation

The application implements a sophisticated power management system that considers:

- **Battery Level Monitoring**: Real-time battery percentage and voltage tracking
- **Adaptive Duty Cycles**: Measurement frequency adjusted based on battery level
- **Component Power States**: Individual control of nRF52840, BLE stack, and VL53L4CD sensor
- **System Sleep Coordination**: Synchronized entry into ultra-low power modes
- **Wake-up Management**: Intelligent wake-up from various interrupt sources

## Future Power Saving Enhancements

To further optimize power consumption and extend battery life for the EpiMed Device, the following enhancements are planned:

- **Machine Learning Power Prediction**: Implement predictive power management based on usage patterns
- **Environmental Adaptive Sensing**: Adjust sensor sensitivity and measurement frequency based on environmental conditions
- **Advanced BLE Connection Management**: Dynamic connection parameter negotiation based on battery state
- **Temperature Compensation**: Battery capacity and sensor performance optimization across temperature ranges
- **User Interaction Learning**: Adaptive measurement scheduling based on learned user interaction patterns
- **Emergency Power Reserves**: Reserved battery capacity management for critical medical device functions
- **Wireless Power Transfer Ready**: Hardware preparation for future wireless charging capabilities

## VL53L4CD Rust Driver (`vl53l4cd.rs`)

This section outlines the plan for porting the VL53L4CD C++ library to a Rust driver (`vl53l4cd.rs`). The new driver will be integrated into the `src/main.rs` application and will provide a safe and idiomatic Rust interface for the sensor.

### Key Components to Port

The following components from the C++ library will be ported to Rust:

- **Structs and Enums**: All necessary data structures, such as `VL53L4CD_Result_t` and error enums, will be redefined in Rust to ensure type safety and clarity.
- **Core Functions**: The essential functions for sensor interaction will be ported, including:
    - `VL53L4CD_SensorInit()`: Initializes the sensor.
    - `VL53L4CD_StartRanging()`: Begins a ranging session.
    - `VL53L4CD_StopRanging()`: Halts the ranging session.
    - `VL53L4CD_CheckForDataReady()`: Checks if new data is available.
    - `VL53L4CD_GetResult()`: Retrieves the measurement results.
    - `VL53L4CD_ClearInterrupt()`: Clears the sensor's interrupt flag.
- **I2C Communication**: The platform-specific I2C functions (`VL53L4CD_I2CRead`, `VL53L4CD_I2CWrite`, etc.) will be re-implemented using the `embassy-nrf/i2c` module to ensure compatibility with the nRF52840.

### Implementation Plan

1.  **Create `vl53l4cd.rs`**: A new file will be created in the `src` directory to house the Rust driver.
2.  **Define Data Structures**: All C++ structs and enums will be translated into their Rust equivalents.
3.  **Implement the `VL53L4CD` Struct**: A `VL53L4CD` struct will be created to manage the sensor's state and provide a clean interface for the driver's functions.
4.  **Port Core Functions**: Each of the core C++ functions will be ported to Rust, ensuring that they are both safe and idiomatic.
5.  **Integrate with `main.rs`**: The new driver will be integrated into the main application, replacing the existing placeholder logic.

By following this plan, we will create a robust and reliable Rust driver for the VL53L4CD sensor, enabling us to fully leverage its capabilities in the EpiMed device.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request or open an Issue for any bugs, feature requests, or improvements.

## License

This project is licensed under the MIT License or Apache License 2.0, at your option. See the [LICENSE](LICENSE) file for details.

## Power and Efficiency Notes from `embassy-nrf` and `nrf-softdevice`

This section summarizes key power-saving and efficiency features available in the underlying libraries, which can be leveraged for future optimizations.

### Core Concepts (Embassy & nRF)

*   **Async & Low Power**: Embassy's `async` model is fundamental to power saving. It allows the CPU to enter a low-power sleep state while waiting for long-running peripheral operations (like DMA transfers) to complete, waking only when necessary.
*   **PPI/DPPI (Programmable Peripheral Interconnect)**: This powerful nRF feature, exposed via `embassy-nrf::ppi`, allows peripherals to interact directly without CPU intervention. For example, a timer can trigger a SAADC sample, or a GPIO event can start a DMA transfer, all while the CPU remains asleep. This is crucial for building complex, ultra-low-power applications.
*   **EasyDMA**: Peripherals with EasyDMA access RAM directly, offloading data transfer tasks from the CPU. The `embassy-nrf` drivers use this extensively. For best performance and efficiency, ensure data buffers are in RAM to avoid runtime copies from flash.

### System Power States

*   **System OFF**: The deepest low-power state can be entered using `embassy_nrf::power::set_system_off()`. This has been implemented in `src/main.rs` within the power management tasks to optimize power consumption by entering System OFF mode during periods of inactivity.
*   **Wake-up Sources**: The device can wake from System OFF on various events. The cause of the wake-up can be identified via `embassy_nrf::reset::read_reasons()`. Key low-power wake-up sources include GPIO pin changes (DETECT signal), LPCOMP, NFC field detection, and VBUS detection.

### Clock Management

*   **Low-Frequency Clock (LFCLK)**: The choice of the LFCLK source is critical. Using an external 32.768 kHz crystal (`LfclkSource::ExternalXtal`) is significantly more power-efficient and accurate for low-power tasks than the internal RC oscillator. This has been implemented in `src/main.rs` with `NRF_CLOCK_LF_SRC_XTAL` for improved power efficiency and an accuracy of 20 PPM.

### Peripheral-Specific Optimizations

*   **DCDC Regulators**: On supported chips (like the nRF52840), enabling the DCDC regulators in the `embassy-nrf::config::Config` is more efficient than the default LDO regulators, especially at higher CPU loads or when the radio is active. This has been implemented in `src/main.rs` by enabling both `reg0` (first stage DCDC, VDDH -> VDD) and `reg1` (second stage DCDC, VDD -> DEC4) for improved power efficiency.
*   **RADIO (BLE)**:
    *   **TX Power**: Use the lowest acceptable transmission power, configurable in `nrf_softdevice::ble::peripheral::Config` via the `tx_power` field. The implementation uses `TxPower::Minus12dBm` for optimal power savings.
    *   **Connection Parameters**: Tuned connection intervals (500-1000ms), slave latency (up to 9 events), and supervision timeout (6 seconds) allow the device to sleep more, significantly reducing power usage.
    *   **Advertising**: Optimized advertising intervals (1000ms) and using timeouts reduce power consumption during advertising phases.
*   **GPIO**: Use standard drive strength (`OutputDrive::Standard`) unless higher drive is necessary, as high-drive modes consume more power.

## Acknowledgments

- [Embassy](https://github.com/embassy-rs/embassy) for the asynchronous embedded framework.
- [nRF SoftDevice](https://github.com/embassy-rs/nrf-softdevice) for BLE stack implementation.
- [STMicroelectronics](https://www.st.com/) for the VL53L4CD ToF sensor and comprehensive documentation.
- The Rust Embedded community for tools and resources.
- Medical device regulatory guidance and safety standards compliance research.