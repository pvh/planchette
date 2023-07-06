#![allow(unused_imports)]
#![allow(clippy::single_component_path_imports)]

use core::ffi;

use std::fs;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::sync::{Condvar, Mutex};
use std::{cell::RefCell, env, sync::atomic::*, sync::Arc, thread, time::*};

use anyhow::{bail, Result};

use automerge::transaction::Transactable;
use automerge::{Automerge, ObjType, ReadDoc, ScalarValue};
use log::*;

use embedded_hal::adc::OneShot;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;

use embedded_svc::eth;
use embedded_svc::io;
use embedded_svc::ipv4;
use embedded_svc::mqtt::client::{Client, Connection, MessageImpl, Publish, QoS};
use embedded_svc::ping::Ping;
use embedded_svc::sys_time::SystemTime;
use embedded_svc::timer::TimerService;
use embedded_svc::timer::*;
use embedded_svc::utils::mqtt::client::ConnState;
use embedded_svc::wifi::*;

use esp_idf_svc::eventloop::*;
use esp_idf_svc::httpd as idf;
use esp_idf_svc::httpd::ServerRegistry;
use esp_idf_svc::mqtt::client::*;
use esp_idf_svc::netif::*;
use esp_idf_svc::nvs::*;
use esp_idf_svc::ping;
use esp_idf_svc::sntp;
use esp_idf_svc::systime::EspSystemTime;
use esp_idf_svc::timer::*;
use esp_idf_svc::wifi::*;

use esp_idf_hal::adc;
use esp_idf_hal::delay;
use esp_idf_hal::gpio::{self, AnyOutputPin};
use esp_idf_hal::i2c;
use esp_idf_hal::peripheral;
use esp_idf_hal::prelude::*;
use esp_idf_hal::spi;

use esp_idf_sys;
use esp_idf_sys::{esp, EspError};

use display_interface_spi::SPIInterfaceNoCS;

use embedded_graphics::mono_font::{ascii::FONT_10X20, MonoTextStyle};
use embedded_graphics::pixelcolor::*;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::text::*;

use mipidsi;
use ssd1306;
use ssd1306::mode::DisplayConfig;

use epd_waveshare::{epd4in2::*, graphics::VarDisplay, prelude::*};

const SSID: &str = "hardenet";
const PASS: &str = "easytoremember";

fn main() -> Result<()> {
    esp_idf_sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    // I2C pins
    // let sda = pins.gpio16;
    // let scl = pins.gpio17;

    // SPI pins

    // SD card pins
    // let sd_cs = pins.gpio21;
    let sd_en = pins.gpio5;

    // E-paper pins
    let epd_cs = pins.gpio22;
    let epd_dc = pins.gpio15;
    let epd_busy = pins.gpio34;
    let epd_en = pins.gpio12;
    let epd_rst = pins.gpio13;

    // PCF8574 pins
    let pcf_int = pins.gpio35;
    // let sd_cd     = pins.gpioP4; // input
    // let EXT_GPIO1 = pins.gpioP5;
    // let EXT_GPIO2 = pins.gpioP6;
    // let EXT_GPIO3 = pins.gpioP7;
    // let PCF_I2C_ADDR = 0x38;

    // LiPo
    // let charge_pin = pins.gpio36;
    let batt_en = pins.gpio25;
    // let batt_volt = pins.gpio39;

    // Buzzer
    // let buzr = pins.gpio26;

    // Buttons
    // Top to bottom
    /*
    let button_1_pin = pins.gpio14;
    let button_1_rtc_gpio = pins.gpio16;

    let button_2_pin = pins.gpio27;
    let button_2_rtc_gpio = pins.gpio17;

    let button_3_pin = pins.gpio4;
    let button_3_rtc_gpio = pins.gpio10;

    let button_4_pin = pins.gpio2;
    let button_4_rtc_gpio = pins.gpio12;
    */

    /*
    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_2_PIN, INPUT_PULLUP);
    pinMode(BUTTON_3_PIN, INPUT_PULLUP);
    pinMode(BUTTON_4_PIN, INPUT_PULLUP);
    */

    let sysloop = EspSystemEventLoop::take()?;
    let wifi = wifi(peripherals.modem, sysloop.clone())?;

    let mut data = Vec::with_capacity(8192);
    get_automerge_doc(&mut data)?;

    let mut doc = Automerge::new();
    doc.load_incremental(&data)?;

    let strokes = match doc.get(automerge::ROOT, "strokes")? {
        Some((automerge::Value::Object(ObjType::List), strokes)) => strokes,
        _ => panic!("strokes should be a list"),
    };
    let stroke = match doc.get(&strokes, 0)? {
        Some((automerge::Value::Object(ObjType::List), stroke)) => stroke,
        _ => panic!("stroke should be a list too"),
    };

    let point = match doc.get(&stroke, 0)? {
        Some((automerge::Value::Object(ObjType::Map), point)) => point,
        _ => panic!("point should be a map"),
    };

    let Some((value_x, _)) = doc.get(&point, "x")? else { panic!("X should exist")};
    let Some(x) = value_x.to_i64() else { panic!("X should be a number")};

    let Some((value_y, _)) = doc.get(&point, "y")? else { panic!("Y should exist")};
    let Some(y) = value_y.to_i64() else { panic!("Y should be a number")};

    info!("Position is {} {}", x, y);
    waveshare_epd_hello_world(
        x,
        y,
        peripherals.spi2,
        pins.gpio18.into(),
        pins.gpio23.into(),
        epd_en.into(),
        epd_cs.into(),
        epd_busy.into(),
        epd_dc.into(),
        epd_rst.into(),
        sd_en.into(),
        batt_en.into(),
        pcf_int.into(),
    )?;

    for s in 0..30 {
        info!("Shutting down in {} secs", 30 - s);
        thread::sleep(Duration::from_secs(1));
    }

    Ok(())
}

fn waveshare_epd_hello_world(
    x: i64,
    y: i64,
    spi: impl peripheral::Peripheral<P = impl spi::SpiAnyPins> + 'static,
    sclk: gpio::AnyOutputPin,
    sdo: gpio::AnyOutputPin,
    en: gpio::AnyOutputPin,
    cs: gpio::AnyOutputPin,
    busy_in: gpio::AnyInputPin,
    dc: gpio::AnyOutputPin,
    rst: gpio::AnyOutputPin,
    sd_en: gpio::AnyOutputPin,
    batt_en: gpio::AnyOutputPin,
    pcf_int: gpio::AnyInputPin,
) -> Result<()> {
    info!("About to initialize Waveshare 4.2 e-paper display");

    // Power up EPD
    let mut en = gpio::PinDriver::output(en)?;
    let cs = gpio::PinDriver::output(cs)?;
    let busy_in = gpio::PinDriver::input(busy_in)?;
    let mut dc = gpio::PinDriver::output(dc)?;
    let mut rst = gpio::PinDriver::output(rst)?;
    let mut sd_en = gpio::PinDriver::output(sd_en)?;
    let mut batt_en = gpio::PinDriver::output(batt_en)?;
    let mut pcf_int = gpio::PinDriver::input(pcf_int)?;

    en = en.into_output()?;
    rst = rst.into_output()?;
    sd_en = sd_en.into_output()?;
    batt_en = batt_en.into_output()?;
    pcf_int = pcf_int.into_input()?;

    en.set_low()?;
    rst.set_low()?;
    let fiddy: u8 = 50;
    delay::Ets.delay_ms(fiddy);
    rst.set_high()?;
    delay::Ets.delay_ms(fiddy);
    dc.set_high()?;

    let mut driver = spi::SpiDeviceDriver::new_single(
        spi,
        sclk,
        sdo,
        Option::<gpio::AnyIOPin>::None,
        Option::<gpio::AnyIOPin>::None,
        &spi::SpiDriverConfig::new().dma(spi::Dma::Disabled),
        &spi::SpiConfig::new().baudrate(26.MHz().into()),
    )?;

    // Setup EPD
    let mut epd = Epd4in2::new(&mut driver, cs, busy_in, dc, rst, &mut delay::Ets).unwrap();

    // Use display graphics from embedded-graphics
    let mut buffer =
        vec![DEFAULT_BACKGROUND_COLOR.get_byte_value(); WIDTH as usize / 8 * HEIGHT as usize];
    let mut display = VarDisplay::new(WIDTH, HEIGHT, &mut buffer);

    let style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

    // Create a text at position (20, 30) and draw it using the previously defined style
    Text::new(
        "POSITIONED",
        Point::new(x.try_into().unwrap(), y.try_into().unwrap()),
        style,
    )
    .draw(&mut display)?;

    // Display updated frame
    epd.update_frame(&mut driver, &display.buffer(), &mut delay::Ets)?;
    epd.display_frame(&mut driver, &mut delay::Ets)?;

    Ok(())
}

fn get_automerge_doc(data: &mut Vec<u8>) -> anyhow::Result<()> {
    use embedded_svc::http::{self, client::*, status, Headers, Status};
    use embedded_svc::io::Read;
    use embedded_svc::utils::io;
    use esp_idf_svc::http::client::*;

    let url = String::from("https://google.com");

    info!("About to fetch content from {}", url);

    let mut client = Client::wrap(EspHttpConnection::new(&Configuration {
        crt_bundle_attach: Some(esp_idf_sys::esp_crt_bundle_attach),

        ..Default::default()
    })?);

    let mut response = client.get(&url)?.submit()?;

    let mut body = [0_u8; 3048];

    let read = io::try_read_full(&mut response, &mut body).map_err(|err| err.0)?;

    info!(
        "Body (truncated to 3K):\n{:?}",
        String::from_utf8_lossy(&body[..read]).into_owned()
    );

    // Complete the response
    while response.read(&mut body)? > 0 {}

    let url = String::from("https://pvh.github.io/trail-runner/ink.amrg");

    info!("About to fetch content from {}", url);

    let mut client = Client::wrap(EspHttpConnection::new(&Configuration {
        crt_bundle_attach: Some(esp_idf_sys::esp_crt_bundle_attach),

        ..Default::default()
    })?);

    let mut response = client.get(&url)?.submit()?;

    info!("Reading response from {}", url);
    let read = io::try_read_full(&mut response, data).map_err(|err| err.0)?;

    // Complete the response
    while response.read(data)? > 0 {}

    info!("Done fetching content from {}", url);
    Ok(())
}

fn wifi(
    modem: impl peripheral::Peripheral<P = esp_idf_hal::modem::Modem> + 'static,
    sysloop: EspSystemEventLoop,
) -> Result<Box<EspWifi<'static>>> {
    use std::net::Ipv4Addr;

    use esp_idf_svc::handle::RawHandle;

    let mut esp_wifi = EspWifi::new(modem, sysloop.clone(), None)?;

    let mut wifi = BlockingWifi::wrap(&mut esp_wifi, sysloop)?;

    wifi.set_configuration(&Configuration::Client(ClientConfiguration::default()))?;

    info!("Starting wifi...");

    wifi.start()?;

    info!("Scanning...");

    let ap_infos = wifi.scan()?;

    let ours = ap_infos.into_iter().find(|a| a.ssid == SSID);

    let channel = if let Some(ours) = ours {
        info!(
            "Found configured access point {} on channel {}",
            SSID, ours.channel
        );
        Some(ours.channel)
    } else {
        info!(
            "Configured access point {} not found during scanning, will go with unknown channel",
            SSID
        );
        None
    };

    wifi.set_configuration(&Configuration::Mixed(
        ClientConfiguration {
            ssid: SSID.into(),
            password: PASS.into(),
            channel,
            ..Default::default()
        },
        AccessPointConfiguration {
            ssid: "aptest".into(),
            channel: channel.unwrap_or(1),
            ..Default::default()
        },
    ))?;

    info!("Connecting wifi...");

    wifi.connect()?;

    info!("Waiting for DHCP lease...");

    wifi.wait_netif_up()?;

    let ip_info = wifi.wifi().sta_netif().get_ip_info()?;

    info!("Wifi DHCP info: {:?}", ip_info);

    Ok(Box::new(esp_wifi))
}
