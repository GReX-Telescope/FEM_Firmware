#![deny(unsafe_code)]
#![no_main]
#![no_std]

// External Imports
use atsamd_hal as hal;
use cortex_m_semihosting::hprintln;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;
use hal::adc::Adc;
use hal::clock::{ClockGenId, ClockSource, GenericClockController};
use hal::pwm::{Channel, Pwm0, Pwm2};
use hal::rtc::{Count32Mode, Duration, Rtc};
use hal::sercom::v2::uart;
use hal::target_device as pac;
use hal::time::*;
use heapless::Vec;
use pac::ADC;
use panic_semihosting as _;
use rtic::app;
use serde_json_core as json;


// Local modules
mod atten;
mod bsp;
mod calibration;
mod log_det;
mod m_c;

// Hardware specific constants
const LOG_DET_LOAD_RES: f32 = 33e3;
const BYTE_BUFF_SIZE: usize = 256;
const LNA_CAL_ON_FREQ: u32 = 32;

// Update period for ADCs
const ADC_UPDATE: u32 = 1;

// Give the PAC and one unused interrupt due to one priority for the software tasks
#[app(device = hal::target_device, peripherals = true, dispatchers = [EVSYS])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        rf1_power: f32,
        rf2_power: f32,
        uart: bsp::UartStruct,
        rf1_lna_en: bsp::Lna1En,
        rf2_lna_en: bsp::Lna2En,
        attenuator: atten::HMC291<bsp::V1, bsp::V2>,
        if_good_threshold: f32,
        cal_1: calibration::LnaCalibration<Pwm0>,
        cal_2: calibration::LnaCalibration<Pwm2>,
    }

    // Only single tasks will ever have access to these
    #[local]
    struct Local {
        // update_if_powers
        adc: Adc<ADC>,
        log_det_1: log_det::LT5537<bsp::AIN10>,
        log_det_2: log_det::LT5537<bsp::AIN11>,
        // Reading bytes
        byte_vec: Vec<u8, BYTE_BUFF_SIZE>,
        curly_counter: u8,
        is_reading: bool,
    }

    #[monotonic(binds = RTC, default = true)]
    type RtcMonotonic = Rtc<Count32Mode>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Extract peripherals from context
        let mut peripherals = cx.device;

        // Setup main clock
        let mut clocks = GenericClockController::with_internal_32kosc(
            peripherals.GCLK,
            &mut peripherals.PM,
            &mut peripherals.SYSCTRL,
            &mut peripherals.NVMCTRL,
        );

        // Configure monotonic timer
        let gclk0 = clocks.gclk0();
        let rtc_clock_src = clocks
            .configure_gclk_divider_and_source(ClockGenId::GCLK2, 1, ClockSource::XOSC32K, false)
            .unwrap();
        clocks.configure_standby(ClockGenId::GCLK2, true);
        let rtc_clock = clocks.rtc(&rtc_clock_src).unwrap();
        let rtc = Rtc::count32_mode(peripherals.RTC, rtc_clock.freq(), &mut peripherals.PM);

        // Construct all pins
        let pins = bsp::Pins::new(peripherals.PORT);

        // Configure UART
        let mut uart = bsp::uart(
            &mut clocks,
            &mut peripherals.PM,
            pins.rxd.into(),
            pins.txd.into(),
            peripherals.SERCOM0,
        );
        // We want a hardware task to trigger on recieved bytes
        uart.enable_interrupts(uart::Flags::RXC);

        // Configure ADC
        let rf1_if_pow: bsp::AIN10 = pins.rf1_if_pow.into();
        let rf2_if_pow: bsp::AIN11 = pins.rf2_if_pow.into();
        let adc = Adc::adc(peripherals.ADC, &mut peripherals.PM, &mut clocks);
        // Build wrappers
        let log_det_1 = log_det::LT5537::new(LOG_DET_LOAD_RES, rf1_if_pow);
        let log_det_2 = log_det::LT5537::new(LOG_DET_LOAD_RES, rf2_if_pow);

        // Configure GPIO
        let rf1_stat_led: bsp::Rf1Led = pins.rf1_stat_led.into();
        let rf2_stat_led: bsp::Rf2Led = pins.rf2_stat_led.into();
        let rf1_lna_en: bsp::Lna1En = pins.rf1_lna_en.into();
        let rf2_lna_en: bsp::Lna2En = pins.rf2_lna_en.into();

        // Configure attenuator
        let v1: bsp::V1 = pins.atten_v1.into();
        let v2: bsp::V2 = pins.atten_v2.into();
        let attenuator = atten::HMC291::new(v1, v2);

        // Configure I2C
        let i2c = bsp::i2c(
            &mut clocks,
            &mut peripherals.PM,
            peripherals.SERCOM2,
            pins.sda.into(),
            pins.scl.into(),
        );

        // Configure PWM
        // Do we need these pins???
        let _rf1_cal: bsp::Rf1Pwm = pins.rf1_cal_tone.into();
        let _rf2_cal: bsp::Rf2Pwm = pins.rf2_cal_tone.into();

        let pwm0 = Pwm0::new(
            &clocks.tcc0_tcc1(&gclk0).unwrap(),
            LNA_CAL_ON_FREQ.khz(),
            peripherals.TCC0,
            &mut peripherals.PM,
        );

        let pwm2 = Pwm2::new(
            &clocks.tcc2_tc3(&gclk0).unwrap(),
            LNA_CAL_ON_FREQ.khz(),
            peripherals.TCC2,
            &mut peripherals.PM,
        );

        let mut cal_1 = calibration::LnaCalibration::new(pwm0, Channel::_2);
        let mut cal_2 = calibration::LnaCalibration::new(pwm2, Channel::_1);

        // Disable on startup
        cal_1.disable();
        cal_2.disable();

        // Schedule all the periodic tasks
        update_if_powers::spawn_after(Duration::secs(ADC_UPDATE)).unwrap();
        hprintln!("FEM Initialized!").unwrap();

        // Return initial values for shared and local resources
        (
            Shared {
                rf1_power: 0.0,
                rf2_power: 0.0,
                uart,
                rf1_lna_en,
                rf2_lna_en,
                attenuator,
                if_good_threshold: 0.0,
                cal_1,
                cal_2,
            },
            Local {
                adc,
                log_det_1,
                log_det_2,
                byte_vec: Vec::<u8, BYTE_BUFF_SIZE>::new(),
                curly_counter: 0,
                is_reading: false,
            },
            init::Monotonics(rtc),
        )
    }

    #[task(local = [adc, log_det_1, log_det_2], shared = [rf1_power, rf2_power])]
    fn update_if_powers(mut cx: update_if_powers::Context) {
        // Unpack context
        let adc = cx.local.adc;
        let log_det_1 = cx.local.log_det_1;
        let log_det_2 = cx.local.log_det_2;
        // Grab updated values and update the shared resource
        cx.shared.rf1_power.lock(|x| *x = log_det_1.read(adc));
        cx.shared.rf2_power.lock(|x| *x = log_det_2.read(adc));
        // Schedule self for the future
        update_if_powers::spawn_after(Duration::secs(ADC_UPDATE)).unwrap();
    }

    #[task]
    fn rf_status_led(mut cs: rf_status_led::Context) {}

    #[task(local = [byte_vec, curly_counter, is_reading], shared = [uart], binds = SERCOM0)]
    fn handle_incoming_uart(mut cx: handle_incoming_uart::Context) {
        // Unpack context
        let byte_vec = cx.local.byte_vec;
        let is_reading = cx.local.is_reading;
        // RXC gets thrown on each byte and reading clears
        let byte = cx.shared.uart.lock(|uart| uart.read()).unwrap();
        // Read bytes until we've matched all the curlies
        // Once that's done - deserialize into a command and dispatch the event
        if *is_reading {
            byte_vec.push(byte).unwrap();
            match byte {
                b'{' => *cx.local.curly_counter += 1,
                b'}' => *cx.local.curly_counter -= 1,
                _ => (),
            }
            if *cx.local.curly_counter == 0 {
                *is_reading = false;
                // Deserialize
                match json::from_slice(byte_vec.as_slice()) {
                    Ok((payload, _)) => control::spawn(payload).unwrap(),
                    Err(_) => hprintln!("Invalid JSON Control Payload!").unwrap(),
                }
                // Clear all the bytes
                byte_vec.clear();
            }
        } else if byte == b'{' {
            // Start reading
            byte_vec.push(byte).unwrap();
            *cx.local.curly_counter += 1;
            *is_reading = true;
        }
    }

    #[task(shared = [rf1_lna_en, rf2_lna_en, attenuator, if_good_threshold, cal_1, cal_2])]
    fn control(mut cx: control::Context, payload: m_c::Control) {
        // Unpack context
        let mut rf1_lna_en = cx.shared.rf1_lna_en;
        let mut rf2_lna_en = cx.shared.rf2_lna_en;
        let mut attenuator = cx.shared.attenuator;
        let mut if_good_threshold = cx.shared.if_good_threshold;
        let mut cal_1 = cx.shared.cal_1;
        let mut cal_2 = cx.shared.cal_2;
        // Do all the updates from the new control payload
        // Calibration (PWM) outputs
        cal_1.lock(|cal| {
            if payload.cal_one {
                cal.enable();
            } else {
                cal.disable();
            }
        });
        cal_2.lock(|cal| {
            if payload.cal_two {
                cal.enable();
            } else {
                cal.disable();
            }
        });
        // LNA power outputs
        rf1_lna_en
            .lock(|pin| {
                if payload.lna_one_powered {
                    pin.set_high()
                } else {
                    pin.set_low()
                }
            })
            .unwrap();
        rf2_lna_en
            .lock(|pin| {
                if payload.lna_two_powered {
                    pin.set_high()
                } else {
                    pin.set_low()
                }
            })
            .unwrap();
        // Attenuation level
        attenuator.lock(|atten| {
            atten.set_atten(match payload.attenuation_level {
                0 => atten::Attenuation::Zero,
                1 => atten::Attenuation::Four,
                2 => atten::Attenuation::Eight,
                3 => atten::Attenuation::Twelve,
                _ => unreachable!(),
            })
        });
        // IF power good threshold
        if_good_threshold.lock(|x| *x = payload.if_power_threshold);
    }

    #[task]
    fn monitor(mut cx: monitor::Context) {}
}
