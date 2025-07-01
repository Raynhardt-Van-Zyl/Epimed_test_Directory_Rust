#![no_std]
#![no_main]
#![macro_use]

mod vl53l4cd;

use defmt_rtt as _; // global logger
use embassy_nrf as _; // time driver
use panic_probe as _;


use core::mem;
use defmt::*;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::{bind_interrupts, twim};
use static_cell::StaticCell;
use embassy_time::{Duration, Timer};
use nrf_softdevice::ble::{gatt_server, peripheral, Connection};
use nrf_softdevice::{raw, Softdevice};
use vl53l4cd::{Vl53l4cd, DEFAULT_ADDRESS};

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<embassy_nrf::peripherals::TWISPI0>;
});

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

use nrf_softdevice::ble::advertisement_builder::{
    Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload, ServiceList, ServiceUuid16,
};

// Battery Service definition
#[nrf_softdevice::gatt_service(uuid = "180f")]
struct BatteryService {
    #[characteristic(uuid = "2a19", read, notify)]
    battery_level: u8,
    #[characteristic(uuid = "2a1a", read, notify)]
    power_saving_mode: u8, // 0 for normal, 1 for sleep mode (dormant state)
}

// Distance Service definition for EpiMed device
#[nrf_softdevice::gatt_service(uuid = "9e7312e0-2354-11eb-9f10-fbc30a62cf38")]
struct DistanceService {
    #[characteristic(uuid = "9e7312e1-2354-11eb-9f10-fbc30a62cf38", read, notify)]
    distance: u16,
}

// GATT Server definition for EpiMed device
#[nrf_softdevice::gatt_server]
struct Server {
    bas: BatteryService,
    dist: DistanceService,
}

// Task for distance measurement updates
#[embassy_executor::task]
async fn bluetooth_task(
    sd: &'static Softdevice,
    server: &'static Server,
    mut sensor: Vl53l4cd<'static, embassy_nrf::peripherals::TWISPI0>,
) {
    info!("Bluetooth task started");
    // Set up BLE advertisement
    static ADV_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .flags(&[Flag::GeneralDiscovery, Flag::LE_Only])
        .services_16(ServiceList::Complete, &[ServiceUuid16::BATTERY])
        .short_name("EpiMed")
        .build();

    static SCAN_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new().full_name("EpiMed Device").build();

    let mut config = peripheral::Config::default();
    config.interval = 160; // in units of 0.625ms, set to 100ms for lower power consumption
    config.tx_power = nrf_softdevice::ble::TxPower::Minus8dBm; // Lower TX power to reduce power usage

    let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
        adv_data: &ADV_DATA,
        scan_data: &SCAN_DATA,
    };

    if let Err(e) = sensor.init(None).await {
        error!("Failed to initialize VL53L4CD sensor: {:?}", e);
        return;
    }
    info!("VL53L4CD sensor initialized");

    loop {
        info!("Starting advertisement in bluetooth task...");
        // Start advertising in connectable mode
        let conn = unwrap!(peripheral::advertise_connectable(sd, adv, &config).await);
        info!("Advertisement started, connection established");

        let distance_fut = async {
            info!("Distance task started");
            if let Err(e) = sensor.start_ranging().await {
                error!("Failed to start ranging: {:?}", e);
                return;
            }
            info!("VL53L4CD ranging started");

            let mut ticker = embassy_time::Ticker::every(Duration::from_millis(100));
            loop {
                ticker.next().await;
                if let Ok(true) = sensor.is_data_ready().await {
                    match sensor.get_result().await {
                        Ok(result) => {
                            info!("Distance: {} mm, Status: {}", result.distance_mm, result.range_status);
                            if let Err(e) = server.dist.distance_set(&result.distance_mm) {
                                info!("Failed to set distance: {:?}", e);
                            }
                            if let Err(e) = server.dist.distance_notify(&conn, &result.distance_mm) {
                                info!("Failed to notify distance: {:?}", e);
                            }
                        }
                        Err(e) => {
                            error!("Failed to get result: {:?}", e);
                        }
                    }
                    if let Err(e) = sensor.clear_interrupt().await {
                        error!("Failed to clear interrupt: {:?}", e);
                    }
                }
            }
        };


        // Create futures for GATT server and other updates
        let gatt_fut = gatt_server::run(&conn, server, |e| match e {
            ServerEvent::Bas(event) => match event {
                BatteryServiceEvent::BatteryLevelCccdWrite { notifications } => {
                    info!("Battery level notifications: {}", notifications);
                }
                BatteryServiceEvent::PowerSavingModeCccdWrite { notifications } => {
                    info!("Power saving mode notifications: {}", notifications);
                }
            },
            ServerEvent::Dist(event) => match event {
                DistanceServiceEvent::DistanceCccdWrite { notifications } => {
                    info!("Distance notifications: {}", notifications);
                }
            },
        });

        let power_fut = power_saving_task(server, &conn, sd);

        futures::pin_mut!(gatt_fut, power_fut, distance_fut);
        futures::future::select(gatt_fut, futures::future::select(power_fut, distance_fut)).await;


        info!("Connection disconnected");
        info!("Restarting advertisement due to disconnection...");
    }
}

// Task for managing power saving mode
async fn power_saving_task(server: &Server, conn: &Connection, _sd: &'static Softdevice) {
    let mut power_saving_mode: u8 = 0; // 0 for normal, 1 for sleep mode
    let mut battery_level: u8 = 100;
    let mut update_interval = embassy_time::Ticker::every(Duration::from_millis(5000)); // Check every 5 seconds
    let mut inactivity_timer = embassy_time::Ticker::every(Duration::from_millis(10000)); // Enter sleep after 10 seconds of inactivity

    // Initialize power saving mode state
    if let Err(e) = server.bas.power_saving_mode_set(&power_saving_mode) {
        info!("Failed to set initial power saving mode: {:?}", e);
    }

    loop {
        // Update battery level periodically
        update_interval.next().await;

        // Simulate battery level decrease for demonstration
        if battery_level > 0 {
            battery_level -= 1;
        }

        // Update battery level
        if let Err(e) = server.bas.battery_level_set(&battery_level) {
            info!("Failed to set battery level: {:?}", e);
        }
        if let Err(e) = server.bas.battery_level_notify(conn, &battery_level) {
            info!("Failed to notify battery level: {:?}", e);
        }

        // Check for inactivity to enter sleep mode
        inactivity_timer.next().await;
        if power_saving_mode == 0 {
            power_saving_mode = 1;
            info!("Entering sleep mode to save power");
            if let Err(e) = server.bas.power_saving_mode_set(&power_saving_mode) {
                info!("Failed to set power saving mode: {:?}", e);
            }
            if let Err(e) = server.bas.power_saving_mode_notify(conn, &power_saving_mode) {
                info!("Failed to notify power saving mode: {:?}", e);
            }
            // Enter System OFF mode, the deepest low-power state (can be woken by configured wake-up sources)
            embassy_nrf::power::set_system_off();
            // Note: This call may not return if the system fully powers down.
            // Wake-up sources should be configured before entering this state.
        }

        // If woken up (e.g., by BLE connection or other interrupt), update state
        if power_saving_mode == 1 {
            power_saving_mode = 0;
            info!("Waking up from sleep mode");
            if let Err(e) = server.bas.power_saving_mode_set(&power_saving_mode) {
                info!("Failed to set power saving mode: {:?}", e);
            }
            if let Err(e) = server.bas.power_saving_mode_notify(conn, &power_saving_mode) {
                info!("Failed to notify power saving mode: {:?}", e);
            }
        }
    }
}

// Task for LED control
#[embassy_executor::task]
async fn led_task(mut led: Output<'static>) {
    info!("LED task started");
    loop {
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("EpiMed Device Starting...");

    // Initialize Embassy FIRST with proper interrupt priorities
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
    config.time_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
    // Enable DCDC Regulators for improved power efficiency
    config.dcdc = embassy_nrf::config::DcdcConfig {
        reg0: true, // Enable first stage DCDC (VDDH -> VDD)
        reg1: true, // Enable second stage DCDC (VDD -> DEC4)
        reg0_voltage: None, // Do not change the voltage setting
    };
    let p = embassy_nrf::init(config);

    // Configure SoftDevice with proper settings for EpiMed
    let config = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_XTAL as u8,
            rc_ctiv: 0, // Not used with XTAL, set to 0
            rc_temp_ctiv: 0, // Not used with XTAL, set to 0
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_20_PPM as u8, // Higher accuracy with external crystal
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 6,
            event_length: 6,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 128 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: raw::BLE_GATTS_ATTR_TAB_SIZE_DEFAULT,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 3,
            central_role_count: 3,
            central_sec_count: 1,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: b"EpiMed" as *const u8 as _,
            current_len: 6,
            max_len: 6,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(raw::BLE_GATTS_VLOC_STACK as u8),
        }),
        ..Default::default()
    };

    let sd = Softdevice::enable(&config);
    static SERVER: StaticCell<Server> = StaticCell::new();
    let server = SERVER.init(unwrap!(Server::new(sd)));
    unwrap!(spawner.spawn(softdevice_task(sd)));

    // Configure I2C
    let mut i2c_config = twim::Config::default();
    i2c_config.frequency = twim::Frequency::K100;
    static mut I2C_BUF: [u8; 32] = [0; 32];
    let i2c = twim::Twim::new(p.TWISPI0, Irqs, p.P0_05, p.P0_04, i2c_config, unsafe { &mut *(&raw mut I2C_BUF as *mut _) });

    // Configure sensor XSHUT pin
    let xshut = Output::new(p.P1_12, Level::Low, OutputDrive::Standard);

    // Create sensor instance
    let sensor = Vl53l4cd::new(i2c, xshut, DEFAULT_ADDRESS);

    // Configure GPIO pin for LED (using P0.06 as an example for nRF52840)
    let led = Output::new(p.P0_06, Level::Low, OutputDrive::Standard);
    info!("LED configured");

    // Spawn task for LED toggling
    unwrap!(spawner.spawn(led_task(led)));
    info!("LED task spawned");

    // Spawn Bluetooth task for advertisement
    unwrap!(spawner.spawn(bluetooth_task(sd, server, sensor)));
    info!("Bluetooth task spawned");
}
