//! A Rust driver for the VL53L4CD Time-of-Flight sensor.

use defmt::*;

use embassy_nrf::twim::{self, Twim};
use embassy_nrf::gpio::Output;
use embassy_time::{Duration, Timer};

pub const DEFAULT_ADDRESS: u8 = 0x29;

/// The error type for the VL53L4CD driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Error {
    /// I2C communication error.
    I2c(twim::Error),
    /// The sensor returned an invalid ID.
    InvalidId,
    /// A timeout occurred while waiting for the sensor.
    Timeout,
    /// An invalid argument was passed to a function.
    InvalidArgument,
}

/// The result of a ranging measurement.
#[derive(Debug, Clone, Copy, Default, defmt::Format)]
pub struct Results {
    pub range_status: u8,
    pub distance_mm: u16,
    pub ambient_rate_kcps: u16,
    pub ambient_per_spad_kcps: u16,
    pub signal_rate_kcps: u16,
    pub signal_per_spad_kcps: u16,
    pub number_of_spad: u16,
    pub sigma_mm: u16,
}

/// The VL53L4CD sensor driver.
pub struct Vl53l4cd<'d, T: twim::Instance> {
    i2c: Twim<'d, T>,
    xshut: Output<'d>,
    address: u8,
}

impl<'d, T: twim::Instance> Vl53l4cd<'d, T> {
    /// Creates a new `Vl53l4cd` driver instance.
    pub fn new(
        i2c: Twim<'d, T>,
        mut xshut: Output<'d>,
        address: u8,
    ) -> Self {
        xshut.set_low();
        Self { i2c, xshut, address }
    }

    /// Initializes the sensor. Can optionally set a new I2C address.
    pub async fn init(&mut self, new_address: Option<u8>) -> Result<(), Error> {
        info!("VL53L4CD: Initializing...");
        Timer::after(Duration::from_millis(10)).await;
        self.xshut.set_high();
        Timer::after(Duration::from_millis(10)).await;
        info!("VL53L4CD: XSHUT high");

        if let Some(addr) = new_address {
            self.set_i2c_address(addr).await?;
            info!("VL53L4CD: I2C address set to {}", addr);
        }

        let id = self.get_sensor_id().await?;
        info!("VL53L4CD: Sensor ID: {:#X}", id);
        if id != 0xEBAA {
            error!("VL53L4CD: Invalid sensor ID");
            return Err(Error::InvalidId);
        }

        // Load default configuration
        info!("VL53L4CD: Loading default configuration...");
        for (i, &val) in DEFAULT_CONFIGURATION.iter().enumerate() {
            self.write_reg((0x2D + i) as u16, val).await.map_err(|e| {
                error!("VL53L4CD: Failed to write default config at reg {:#X}", 0x2D + i);
                e
            })?;
        }
        info!("VL53L4CD: Default configuration loaded");

        // Start VHV
        info!("VL53L4CD: Starting VHV...");
        self.write_reg(registers::SYSTEM_START, 0x40).await.map_err(|e| {
            error!("VL53L4CD: Failed to start VHV");
            e
        })?;
        info!("VL53L4CD: VHV started");

        // Wait for data ready
        info!("VL53L4CD: Waiting for data ready...");
        let mut i = 0;
        loop {
            if self.is_data_ready().await? {
                break;
            }
            if i >= 1000 {
                error!("VL53L4CD: Timeout waiting for data ready");
                return Err(Error::Timeout);
            }
            i += 1;
            Timer::after(Duration::from_millis(1)).await;
        }
        info!("VL53L4CD: Data ready");

        self.clear_interrupt().await?;
        info!("VL53L4CD: Interrupt cleared");
        self.stop_ranging().await?;
        info!("VL53L4CD: Ranging stopped");

        self.write_reg(registers::VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09).await?;
        self.write_reg(0x0B, 0).await?;
        self.write_reg16(0x0024, 0x500).await?;
        info!("VL53L4CD: VHV config set");

        self.set_range_timing(50, 0).await?;
        info!("VL53L4CD: Range timing set");

        info!("VL53L4CD: Initialization complete");
        Ok(())
    }

    /// Sets a new I2C address for the sensor.
    pub async fn set_i2c_address(&mut self, new_address: u8) -> Result<(), Error> {
        self.write_reg(registers::I2C_SLAVE_DEVICE_ADDRESS, new_address >> 1).await?;
        self.address = new_address;
        Ok(())
    }

    /// Gets the sensor's model ID.
    pub async fn get_sensor_id(&mut self) -> Result<u16, Error> {
        self.read_reg16(registers::IDENTIFICATION_MODEL_ID).await
    }

    /// Starts a ranging measurement.
    pub async fn start_ranging(&mut self) -> Result<(), Error> {
        let inter_measurement = self.read_reg32(registers::INTERMEASUREMENT_MS).await?;
        if inter_measurement == 0 {
            // Continuous mode
            self.write_reg(registers::SYSTEM_START, 0x21).await?;
        } else {
            // Autonomous mode
            self.write_reg(registers::SYSTEM_START, 0x40).await?;
        }

        // Wait for data ready
        let mut i = 0;
        loop {
            if self.is_data_ready().await? {
                break;
            }
            if i >= 1000 {
                return Err(Error::Timeout);
            }
            i += 1;
            Timer::after(Duration::from_millis(1)).await;
        }
        self.clear_interrupt().await
    }

    /// Stops the ranging measurement.
    pub async fn stop_ranging(&mut self) -> Result<(), Error> {
        self.write_reg(registers::SYSTEM_START, 0x00).await
    }

    /// Checks if data is ready to be read.
    pub async fn is_data_ready(&mut self) -> Result<bool, Error> {
        let temp = self.read_reg(registers::GPIO_HV_MUX_CTRL).await?;
        let int_pol = if (temp & 0x10) >> 4 == 1 { 0 } else { 1 };
        let status = self.read_reg(registers::GPIO_TIO_HV_STATUS).await?;
        Ok((status & 0x01) == int_pol)
    }

    /// Reads the measurement results.
    pub async fn get_result(&mut self) -> Result<Results, Error> {
        let mut range_status = self.read_reg(registers::RESULT_RANGE_STATUS).await? & 0x1F;
        if (range_status as usize) < STATUS_RTN.len() {
            range_status = STATUS_RTN[range_status as usize];
        }

        let number_of_spad = self.read_reg16(registers::RESULT_SPAD_NB).await? / 256;

        if number_of_spad == 0 {
            return Ok(Results {
                range_status: 255,
                ..Default::default()
            });
        }

        let signal_rate_kcps = self.read_reg16(registers::RESULT_SIGNAL_RATE).await? * 8;
        let ambient_rate_kcps = self.read_reg16(registers::RESULT_AMBIENT_RATE).await? * 8;
        let sigma_mm = self.read_reg16(registers::RESULT_SIGMA).await? / 4;
        let distance_mm = self.read_reg16(registers::RESULT_DISTANCE).await?;

        let signal_per_spad_kcps = signal_rate_kcps / number_of_spad;
        let ambient_per_spad_kcps = ambient_rate_kcps / number_of_spad;

        Ok(Results {
            range_status,
            distance_mm,
            ambient_rate_kcps,
            ambient_per_spad_kcps,
            signal_rate_kcps,
            signal_per_spad_kcps,
            number_of_spad,
            sigma_mm,
        })
    }

    /// Sets new range timing.
    pub async fn set_range_timing(&mut self, timing_budget_ms: u32, inter_measurement_ms: u32) -> Result<(), Error> {
        if timing_budget_ms < 10 || timing_budget_ms > 200 {
            return Err(Error::InvalidArgument);
        }

        let osc_frequency = self.read_reg16(0x0006).await?;
        if osc_frequency == 0 {
            return Err(Error::InvalidArgument);
        }
        let macro_period_us = (2304 * (0x40000000 / osc_frequency as u32)) >> 6;

        let mut timing_budget_us = timing_budget_ms * 1000;

        if inter_measurement_ms == 0 {
            // Continuous mode
            self.write_reg32(registers::INTERMEASUREMENT_MS, 0).await?;
            timing_budget_us -= 2500;
        } else if inter_measurement_ms > timing_budget_ms {
            // Autonomous low power mode
            let clock_pll = self.read_reg16(registers::RESULT_OSC_CALIBRATE_VAL).await? & 0x3FF;
            let inter_measurement_factor = (1055 * inter_measurement_ms * clock_pll as u32) / 1000;
            self.write_reg32(registers::INTERMEASUREMENT_MS, inter_measurement_factor).await?;
            timing_budget_us -= 4300;
            timing_budget_us /= 2;
        } else {
            return Err(Error::InvalidArgument);
        }

        let mut ms_byte: u16;
        let mut ls_byte: u32;

        // For RANGE_CONFIG_A
        ms_byte = 0;
        let tmp = macro_period_us * 16;
        ls_byte = (((timing_budget_us << 12) + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1;
        while (ls_byte & 0xFFFFFF00) > 0 {
            ls_byte >>= 1;
            ms_byte += 1;
        }
        self.write_reg16(registers::RANGE_CONFIG_A, (ms_byte << 8) + (ls_byte as u16 & 0xFF)).await?;

        // For RANGE_CONFIG_B
        ms_byte = 0;
        let tmp = macro_period_us * 12;
        ls_byte = (((timing_budget_us << 12) + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1;
        while (ls_byte & 0xFFFFFF00) > 0 {
            ls_byte >>= 1;
            ms_byte += 1;
        }
        self.write_reg16(registers::RANGE_CONFIG_B, (ms_byte << 8) + (ls_byte as u16 & 0xFF)).await?;

        Ok(())
    }

    /// Clears the interrupt flag.
    pub async fn clear_interrupt(&mut self) -> Result<(), Error> {
        self.write_reg(registers::SYSTEM_INTERRUPT_CLEAR, 0x01).await
    }

    async fn read_reg(&mut self, reg: u16) -> Result<u8, Error> {
        let mut buf = [0; 1];
        self.i2c
            .write_read(self.address, &reg.to_be_bytes(), &mut buf)
            .await
            .map_err(Error::I2c)?;
        Ok(buf[0])
    }

    async fn write_reg(&mut self, reg: u16, val: u8) -> Result<(), Error> {
        let bytes = [reg.to_be_bytes()[0], reg.to_be_bytes()[1], val];
        self.i2c.write(self.address, &bytes).await.map_err(Error::I2c)
    }

    async fn read_reg16(&mut self, reg: u16) -> Result<u16, Error> {
        let mut buf = [0; 2];
        self.i2c
            .write_read(self.address, &reg.to_be_bytes(), &mut buf)
            .await
            .map_err(Error::I2c)?;
        Ok(u16::from_be_bytes(buf))
    }

    async fn write_reg16(&mut self, reg: u16, val: u16) -> Result<(), Error> {
        let reg_bytes = reg.to_be_bytes();
        let val_bytes = val.to_be_bytes();
        let bytes = [reg_bytes[0], reg_bytes[1], val_bytes[0], val_bytes[1]];
        self.i2c.write(self.address, &bytes).await.map_err(Error::I2c)
    }

    async fn read_reg32(&mut self, reg: u16) -> Result<u32, Error> {
        let mut buf = [0; 4];
        self.i2c
            .write_read(self.address, &reg.to_be_bytes(), &mut buf)
            .await
            .map_err(Error::I2c)?;
        Ok(u32::from_be_bytes(buf))
    }

    async fn write_reg32(&mut self, reg: u16, val: u32) -> Result<(), Error> {
        let reg_bytes = reg.to_be_bytes();
        let val_bytes = val.to_be_bytes();
        let bytes = [reg_bytes[0], reg_bytes[1], val_bytes[0], val_bytes[1], val_bytes[2], val_bytes[3]];
        self.i2c.write(self.address, &bytes).await.map_err(Error::I2c)
    }
}

const STATUS_RTN: &[u8] = &[ 255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255, 9, 13, 255, 255, 255, 255, 10, 6, 255, 255, 11, 12 ];

const DEFAULT_CONFIGURATION: &[u8] = &[
  0x12, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C),
   else don't touch */
  0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1
   (pull up at AVDD) */
  0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1
  (pull up at AVDD) */
  0x11, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low
  (bits 3:0 must be 0x1), use SetInterruptPolarity() */
  0x02, /* 0x31 : bit 1 = interrupt depending on the polarity,
  use CheckForDataReady() */
  0x00, /* 0x32 : not user-modifiable */
  0x02, /* 0x33 : not user-modifiable */
  0x08, /* 0x34 : not user-modifiable */
  0x00, /* 0x35 : not user-modifiable */
  0x08, /* 0x36 : not user-modifiable */
  0x10, /* 0x37 : not user-modifiable */
  0x01, /* 0x38 : not user-modifiable */
  0x01, /* 0x39 : not user-modifiable */
  0x00, /* 0x3a : not user-modifiable */
  0x00, /* 0x3b : not user-modifiable */
  0x00, /* 0x3c : not user-modifiable */
  0x00, /* 0x3d : not user-modifiable */
  0xff, /* 0x3e : not user-modifiable */
  0x00, /* 0x3f : not user-modifiable */
  0x0F, /* 0x40 : not user-modifiable */
  0x00, /* 0x41 : not user-modifiable */
  0x00, /* 0x42 : not user-modifiable */
  0x00, /* 0x43 : not user-modifiable */
  0x00, /* 0x44 : not user-modifiable */
  0x00, /* 0x45 : not user-modifiable */
  0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high,
  2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
  0x0b, /* 0x47 : not user-modifiable */
  0x00, /* 0x48 : not user-modifiable */
  0x00, /* 0x49 : not user-modifiable */
  0x02, /* 0x4a : not user-modifiable */
  0x14, /* 0x4b : not user-modifiable */
  0x21, /* 0x4c : not user-modifiable */
  0x00, /* 0x4d : not user-modifiable */
  0x00, /* 0x4e : not user-modifiable */
  0x05, /* 0x4f : not user-modifiable */
  0x00, /* 0x50 : not user-modifiable */
  0x00, /* 0x51 : not user-modifiable */
  0x00, /* 0x52 : not user-modifiable */
  0x00, /* 0x53 : not user-modifiable */
  0xc8, /* 0x54 : not user-modifiable */
  0x00, /* 0x55 : not user-modifiable */
  0x00, /* 0x56 : not user-modifiable */
  0x38, /* 0x57 : not user-modifiable */
  0xff, /* 0x58 : not user-modifiable */
  0x01, /* 0x59 : not user-modifiable */
  0x00, /* 0x5a : not user-modifiable */
  0x08, /* 0x5b : not user-modifiable */
  0x00, /* 0x5c : not user-modifiable */
  0x00, /* 0x5d : not user-modifiable */
  0x01, /* 0x5e : not user-modifiable */
  0xcc, /* 0x5f : not user-modifiable */
  0x07, /* 0x60 : not user-modifiable */
  0x01, /* 0x61 : not user-modifiable */
  0xf1, /* 0x62 : not user-modifiable */
  0x05, /* 0x63 : not user-modifiable */
  0x00, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB),
   use SetSigmaThreshold(), default value 90 mm  */
  0xa0, /* 0x65 : Sigma threshold LSB */
  0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB),
   use SetSignalThreshold() */
  0x80, /* 0x67 : Min count Rate LSB */
  0x08, /* 0x68 : not user-modifiable */
  0x38, /* 0x69 : not user-modifiable */
  0x00, /* 0x6a : not user-modifiable */
  0x00, /* 0x6b : not user-modifiable */
  0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register,
   use SetIntermeasurementInMs() */
  0x00, /* 0x6d : Intermeasurement period */
  0x0f, /* 0x6e : Intermeasurement period */
  0x89, /* 0x6f : Intermeasurement period LSB */
  0x00, /* 0x70 : not user-modifiable */
  0x00, /* 0x71 : not user-modifiable */
  0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB),
   use SetD:tanceThreshold() */
  0x00, /* 0x73 : distance threshold high LSB */
  0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB),
   use SetD:tanceThreshold() */
  0x00, /* 0x75 : distance threshold low LSB */
  0x00, /* 0x76 : not user-modifiable */
  0x01, /* 0x77 : not user-modifiable */
  0x07, /* 0x78 : not user-modifiable */
  0x05, /* 0x79 : not user-modifiable */
  0x06, /* 0x7a : not user-modifiable */
  0x06, /* 0x7b : not user-modifiable */
  0x00, /* 0x7c : not user-modifiable */
  0x00, /* 0x7d : not user-modifiable */
  0x02, /* 0x7e : not user-modifiable */
  0xc7, /* 0x7f : not user-modifiable */
  0xff, /* 0x80 : not user-modifiable */
  0x9B, /* 0x81 : not user-modifiable */
  0x00, /* 0x82 : not user-modifiable */
  0x00, /* 0x83 : not user-modifiable */
  0x00, /* 0x84 : not user-modifiable */
  0x01, /* 0x85 : not user-modifiable */
  0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
  0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(),
   If you want an automatic start after VL53L4CD_init() call,
    put 0x40 in location 0x87 */
];

/// Register addresses for the VL53L4CD sensor.
#[allow(dead_code)]
mod registers {
    pub const SOFT_RESET: u16 = 0x0000;
    pub const I2C_SLAVE_DEVICE_ADDRESS: u16 = 0x0001;
    pub const VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND: u16 = 0x0008;
    pub const XTALK_PLANE_OFFSET_KCPS: u16 = 0x0016;
    pub const XTALK_X_PLANE_GRADIENT_KCPS: u16 = 0x0018;
    pub const XTALK_Y_PLANE_GRADIENT_KCPS: u16 = 0x001A;
    pub const RANGE_OFFSET_MM: u16 = 0x001E;
    pub const INNER_OFFSET_MM: u16 = 0x0020;
    pub const OUTER_OFFSET_MM: u16 = 0x0022;
    pub const I2C_FAST_MODE_PLUS: u16 = 0x002D;
    pub const GPIO_HV_MUX_CTRL: u16 = 0x0030;
    pub const GPIO_TIO_HV_STATUS: u16 = 0x0031;
    pub const SYSTEM_INTERRUPT: u16 = 0x0046;
    pub const RANGE_CONFIG_A: u16 = 0x005E;
    pub const RANGE_CONFIG_B: u16 = 0x0061;
    pub const RANGE_CONFIG_SIGMA_THRESH: u16 = 0x0064;
    pub const MIN_COUNT_RATE_RTN_LIMIT_MCPS: u16 = 0x0066;
    pub const INTERMEASUREMENT_MS: u16 = 0x006C;
    pub const THRESH_HIGH: u16 = 0x0072;
    pub const THRESH_LOW: u16 = 0x0074;
    pub const SYSTEM_INTERRUPT_CLEAR: u16 = 0x0086;
    pub const SYSTEM_START: u16 = 0x0087;
    pub const RESULT_RANGE_STATUS: u16 = 0x0089;
    pub const RESULT_SPAD_NB: u16 = 0x008C;
    pub const RESULT_SIGNAL_RATE: u16 = 0x008E;
    pub const RESULT_AMBIENT_RATE: u16 = 0x0090;
    pub const RESULT_SIGMA: u16 = 0x0092;
    pub const RESULT_DISTANCE: u16 = 0x0096;
    pub const RESULT_OSC_CALIBRATE_VAL: u16 = 0x00DE;
    pub const FIRMWARE_SYSTEM_STATUS: u16 = 0x00E5;
    pub const IDENTIFICATION_MODEL_ID: u16 = 0x010F;
}