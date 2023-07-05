#![allow(unused_imports)]
#![allow(clippy::single_component_path_imports)]
//#![feature(backtrace)]

use core::ffi;

use std::fs;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::sync::{Condvar, Mutex};
use std::{cell::RefCell, env, sync::atomic::*, sync::Arc, thread, time::*};

use anyhow::{bail, Result};

use log::*;

use url;

use smol;

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

thread_local! {
    static TLS: RefCell<u32> = RefCell::new(13);
}

fn main() -> Result<()> {
    esp_idf_sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    // I2C pins
    let SDA = pins.gpio16;
    let SCL = pins.gpio17;

    // SPI pins

    // SD card pins
    let SD_CS = pins.gpio21;
    let SD_EN = pins.gpio5;

    // E-paper pins
    let EPD_CS = pins.gpio22;
    let EPD_DC = pins.gpio15;
    let EPD_BUSY = pins.gpio34;
    let EPD_EN = pins.gpio12;
    let EPD_RST = pins.gpio13;

    // PCF8574 pins
    let PCF_INT = pins.gpio35;
    // let SD_CD     = pins.gpioP4; // input
    // let EXT_GPIO1 = pins.gpioP5;
    // let EXT_GPIO2 = pins.gpioP6;
    // let EXT_GPIO3 = pins.gpioP7;
    // let PCF_I2C_ADDR = 0x38;

    // LiPo
    // let CHARGE_PIN = pins.gpio36;
    let BATT_EN = pins.gpio25;
    // let BATT_VOLT = pins.gpio39;

    // Buzzer
    // let BUZR = pins.gpio26;

    // Buttons
    // Top to bottom
    /*
    let BUTTON_1_PIN = pins.gpio14;
    let BUTTON_1_RTC_GPIO = pins.gpio16;

    let BUTTON_2_PIN = pins.gpio27;
    let BUTTON_2_RTC_GPIO = pins.gpio17;

    let BUTTON_3_PIN = pins.gpio4;
    let BUTTON_3_RTC_GPIO = pins.gpio10;

    let BUTTON_4_PIN = pins.gpio2;
    let BUTTON_4_RTC_GPIO = pins.gpio12;
    */

    /*
    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_2_PIN, INPUT_PULLUP);
    pinMode(BUTTON_3_PIN, INPUT_PULLUP);
    pinMode(BUTTON_4_PIN, INPUT_PULLUP);
    */

    /* digitalWrite(EPD_EN, LOW);
    digitalWrite(EPD_RST, LOW);
    delay(50);
    digitalWrite(EPD_RST, HIGH);
    delay(50); */

    // I can't explain why this line is necessary but it is.

    let sysloop = EspSystemEventLoop::take()?;

    waveshare_epd_hello_world(
        peripherals.spi2,
        pins.gpio18.into(),
        pins.gpio23.into(),
        EPD_EN.into(),
        EPD_CS.into(),
        EPD_BUSY.into(),
        EPD_DC.into(),
        EPD_RST.into(),
        SD_EN.into(),
        BATT_EN.into(),
        PCF_INT.into(),
    )?;

    let wifi = wifi(peripherals.modem, sysloop.clone())?;
    test_tcp()?;
    test_tcp_bind()?;

    let _sntp = sntp::EspSntp::new_default()?;
    info!("SNTP initialized");

    test_tcp_bind_async()?;

    let mutex = Arc::new((Mutex::new(None), Condvar::new()));

    let httpd = httpd(mutex.clone())?;

    for s in 0..30 {
        info!("Shutting down in {} secs", 30 - s);
        thread::sleep(Duration::from_secs(1));
    }

    drop(httpd);
    info!("Httpd stopped");

    #[cfg(not(feature = "qemu"))]
    {
        drop(wifi);
        info!("Wifi stopped");
    }

    Ok(())
}

fn waveshare_epd_hello_world(
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
    Text::new("Hello Rust!", Point::new(20, 30), style).draw(&mut display)?;

    // Display updated frame
    epd.update_frame(&mut driver, &display.buffer(), &mut delay::Ets)?;
    epd.display_frame(&mut driver, &mut delay::Ets)?;

    Ok(())
}

fn test_tcp() -> Result<()> {
    info!("About to open a TCP connection to 1.1.1.1 port 80");

    let mut stream = TcpStream::connect("one.one.one.one:80")?;

    let err = stream.try_clone();
    if let Err(err) = err {
        info!(
            "Duplication of file descriptors does not work (yet) on the ESP-IDF, as expected: {}",
            err
        );
    }

    stream.write_all("GET / HTTP/1.0\n\n".as_bytes())?;

    let mut result = Vec::new();

    stream.read_to_end(&mut result)?;

    info!(
        "1.1.1.1 returned:\n=================\n{}\n=================\nSince it returned something, all is OK",
        std::str::from_utf8(&result)?);

    Ok(())
}

fn test_tcp_bind() -> Result<()> {
    fn test_tcp_bind_accept() -> Result<()> {
        info!("About to bind a simple echo service to port 8080");

        let listener = TcpListener::bind("0.0.0.0:8080")?;

        for stream in listener.incoming() {
            match stream {
                Ok(stream) => {
                    info!("Accepted client");

                    thread::spawn(move || {
                        test_tcp_bind_handle_client(stream);
                    });
                }
                Err(e) => {
                    error!("Error: {}", e);
                }
            }
        }

        unreachable!()
    }

    fn test_tcp_bind_handle_client(mut stream: TcpStream) {
        // read 20 bytes at a time from stream echoing back to stream
        loop {
            let mut read = [0; 128];

            match stream.read(&mut read) {
                Ok(n) => {
                    if n == 0 {
                        // connection was closed
                        break;
                    }
                    stream.write_all(&read[0..n]).unwrap();
                }
                Err(err) => {
                    panic!("{}", err);
                }
            }
        }
    }

    thread::spawn(|| test_tcp_bind_accept().unwrap());

    Ok(())
}

fn test_tcp_bind_async() -> anyhow::Result<()> {
    async fn test_tcp_bind() -> smol::io::Result<()> {
        /// Echoes messages from the client back to it.
        async fn echo(stream: smol::Async<TcpStream>) -> smol::io::Result<()> {
            smol::io::copy(&stream, &mut &stream).await?;
            Ok(())
        }

        // Create a listener.
        let listener = smol::Async::<TcpListener>::bind(([0, 0, 0, 0], 8081))?;

        // Accept clients in a loop.
        loop {
            let (stream, peer_addr) = listener.accept().await?;
            info!("Accepted client: {}", peer_addr);

            // Spawn a task that echoes messages from the client back to it.
            smol::spawn(echo(stream)).detach();
        }
    }

    info!("About to bind a simple echo service to port 8081 using async (smol-rs)!");

    #[allow(clippy::needless_update)]
    {
        esp_idf_sys::esp!(unsafe {
            esp_idf_sys::esp_vfs_eventfd_register(&esp_idf_sys::esp_vfs_eventfd_config_t {
                max_fds: 5,
                ..Default::default()
            })
        })?;
    }

    thread::Builder::new().stack_size(4096).spawn(move || {
        smol::block_on(test_tcp_bind()).unwrap();
    })?;

    Ok(())
}

#[allow(unused_variables)]
fn httpd(
    mutex: Arc<(Mutex<Option<u32>>, Condvar)>,
) -> Result<esp_idf_svc::http::server::EspHttpServer> {
    use embedded_svc::http::server::{
        Connection, Handler, HandlerResult, Method, Middleware, Query, Request, Response,
    };
    use embedded_svc::io::Write;
    use esp_idf_svc::http::server::{fn_handler, EspHttpConnection, EspHttpServer};

    struct SampleMiddleware {}

    impl<C> Middleware<C> for SampleMiddleware
    where
        C: Connection,
    {
        fn handle<'a, H>(&'a self, connection: &'a mut C, handler: &'a H) -> HandlerResult
        where
            H: Handler<C>,
        {
            let req = Request::wrap(connection);

            info!("Middleware called with uri: {}", req.uri());

            let connection = req.release();

            if let Err(err) = handler.handle(connection) {
                if !connection.is_response_initiated() {
                    let mut resp = Request::wrap(connection).into_status_response(500)?;

                    write!(&mut resp, "ERROR: {err}")?;
                } else {
                    // Nothing can be done as the error happened after the response was initiated, propagate further
                    return Err(err);
                }
            }

            Ok(())
        }
    }

    struct SampleMiddleware2 {}

    impl<C> Middleware<C> for SampleMiddleware2
    where
        C: Connection,
    {
        fn handle<'a, H>(&'a self, connection: &'a mut C, handler: &'a H) -> HandlerResult
        where
            H: Handler<C>,
        {
            info!("Middleware2 called");

            handler.handle(connection)
        }
    }

    let mut server = EspHttpServer::new(&Default::default())?;

    server
        .fn_handler("/", Method::Get, |req| {
            req.into_ok_response()?
                .write_all("Hello from Rust!".as_bytes())?;

            Ok(())
        })?
        .fn_handler("/foo", Method::Get, |_| {
            Result::Err("Boo, something happened!".into())
        })?
        .fn_handler("/bar", Method::Get, |req| {
            req.into_response(403, Some("No permissions"), &[])?
                .write_all("You have no permissions to access this page".as_bytes())?;

            Ok(())
        })?
        .fn_handler("/panic", Method::Get, |_| panic!("User requested a panic!"))?
        .handler(
            "/middleware",
            Method::Get,
            SampleMiddleware {}.compose(fn_handler(|_| {
                Result::Err("Boo, something happened!".into())
            })),
        )?
        .handler(
            "/middleware2",
            Method::Get,
            SampleMiddleware2 {}.compose(SampleMiddleware {}.compose(fn_handler(|req| {
                req.into_ok_response()?
                    .write_all("Middleware2 handler called".as_bytes())?;

                Ok(())
            }))),
        )?;

    Ok(server)
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

    ping(ip_info.subnet.gateway)?;

    Ok(Box::new(esp_wifi))
}

fn ping(ip: ipv4::Ipv4Addr) -> Result<()> {
    info!("About to do some pings for {:?}", ip);

    let ping_summary = ping::EspPing::default().ping(ip, &Default::default())?;
    if ping_summary.transmitted != ping_summary.received {
        bail!("Pinging IP {} resulted in timeouts", ip);
    }

    info!("Pinging done");

    Ok(())
}
