#![no_std]
#![no_main]
#![macro_use]

use defmt_rtt as _; // global logger
use embassy_nrf as _; // time driver
use panic_probe as _;


use core::mem;
use defmt::*;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::interrupt;
use embassy_time::{Duration, Timer};
use nrf_softdevice::ble::{gatt_server, peripheral, Connection};
use nrf_softdevice::{raw, Softdevice};

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
}

// Distance Service definition for EpiPen device
#[nrf_softdevice::gatt_service(uuid = "9e7312e0-2354-11eb-9f10-fbc30a62cf38")]
struct DistanceService {
    #[characteristic(uuid = "9e7312e1-2354-11eb-9f10-fbc30a62cf38", read, notify)]
    distance: f32,
}

// GATT Server definition for EpiPen device
#[nrf_softdevice::gatt_server]
struct Server {
    bas: BatteryService,
    dist: DistanceService,
}

// Task for distance measurement updates
async fn distance_update_task(server: &Server, conn: &Connection) {
    let mut distance: f32 = 0.0;
    let mut update_interval = embassy_time::Ticker::every(Duration::from_millis(1000));

    loop {
        update_interval.next().await;

        // Update distance value
        if distance < 100.0 {
            distance += 0.5;
            if distance > 100.0 {
                distance = 100.0;
            }
            info!("Updating distance to: {}", distance);
            if let Err(e) = server.dist.distance_set(&distance) {
                info!("Failed to set distance: {:?}", e);
            }
            if let Err(e) = server.dist.distance_notify(conn, &distance) {
                info!("Failed to notify distance: {:?}", e);
            }
        }
    }
}

// Task for Bluetooth advertisement
#[embassy_executor::task]
async fn bluetooth_task(sd: &'static Softdevice, server: Server) {
    info!("Bluetooth task started");
    // Set up BLE advertisement
    static ADV_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .flags(&[Flag::GeneralDiscovery, Flag::LE_Only])
        .services_16(ServiceList::Complete, &[ServiceUuid16::BATTERY])
        .short_name("EpiPen")
        .build();

    static SCAN_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new().full_name("EpiPen Device").build();

    let mut config = peripheral::Config::default();
    config.interval = 32; // in units of 0.625ms

    let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
        adv_data: &ADV_DATA,
        scan_data: &SCAN_DATA,
    };

    loop {
        info!("Starting advertisement in bluetooth task...");
        // Start advertising in connectable mode
        let conn = unwrap!(peripheral::advertise_connectable(sd, adv, &config).await);
        info!("Advertisement started, connection established");

        // Create futures for GATT server and distance updates
        let gatt_fut = gatt_server::run(&conn, &server, |e| match e {
            ServerEvent::Bas(event) => match event {
                BatteryServiceEvent::BatteryLevelCccdWrite { notifications } => {
                    info!("Battery level notifications: {}", notifications);
                }
            },
            ServerEvent::Dist(event) => match event {
                DistanceServiceEvent::DistanceCccdWrite { notifications } => {
                    info!("Distance notifications: {}", notifications);
                }
            },
        });

        let distance_fut = distance_update_task(&server, &conn);

        futures::pin_mut!(gatt_fut);
        futures::pin_mut!(distance_fut);

        // Run both futures concurrently until disconnection
        let result = futures::future::select(gatt_fut, distance_fut).await;
        match result {
            futures::future::Either::Left((gatt_result, _)) => {
                info!("Connection disconnected with result: {:?}", gatt_result);
            }
            futures::future::Either::Right((_, _)) => {
                info!("Distance task completed unexpectedly");
            }
        }

        info!("Restarting advertisement due to disconnection...");
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
    info!("EpiPen Device Starting...");

    // Initialize Embassy FIRST with proper interrupt priorities
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = interrupt::Priority::P2;
    config.time_interrupt_priority = interrupt::Priority::P2;
    let p = embassy_nrf::init(config);

    // Configure SoftDevice with proper settings for EpiPen
    let config = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
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
            p_value: b"EpiPen" as *const u8 as _,
            current_len: 6,
            max_len: 6,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(raw::BLE_GATTS_VLOC_STACK as u8),
        }),
        ..Default::default()
    };

    let sd = Softdevice::enable(&config);
    let server = unwrap!(Server::new(sd));
    unwrap!(spawner.spawn(softdevice_task(sd)));

    // Configure GPIO pin for LED (using P0.06 as an example for nRF52840)
    let led = Output::new(p.P0_06, Level::Low, OutputDrive::Standard);
    info!("LED configured");

    // Spawn task for LED toggling
    unwrap!(spawner.spawn(led_task(led)));
    info!("LED task spawned");

    // Spawn Bluetooth task for advertisement
    unwrap!(spawner.spawn(bluetooth_task(sd, server)));
    info!("Bluetooth task spawned");

    // Keep the main task running
    info!("Main task running...");
    loop {
        Timer::after(Duration::from_millis(1000)).await;
    }
}
