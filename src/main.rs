#![deny(unsafe_code)]
#![no_main]
#![no_std]

// Local modules
mod atten;
mod bsp;
mod calibration;
mod log_det;
mod m_c;
mod tmp100;

// External Imports
use atsamd_hal as hal;
use core::cell::RefCell;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;
use hal::adc::Adc;
use hal::clock::{ClockGenId, ClockSource, GenericClockController};
use hal::delay::Delay;
use hal::pac;
use hal::pwm::{Channel, Pwm0, Pwm2};
use hal::rtc::{Count32Mode, Duration, Rtc};
use hal::sercom::uart;
use hal::time::*;
use heapless::Vec;
use nb::block;
use pac::ADC;
use pac194x::regs as pwr_regs;
use pac194x::{AddrSelect, PAC194X};
use panic_probe as _;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};
use serde_json_core as json;
use shared_bus::I2cProxy;
use tmp100::TMP100;

// Hardware specific constants
const LOG_DET_LOAD_RES: f32 = 33e3;
const BYTE_BUFF_SIZE: usize = 256;
const LNA_CAL_ON_KHZ: u32 = 32;
const RSENSE_ANALOG_INPUT: f32 = 0.1;
const RSENSE_LNA: f32 = 0.5;

// Update period for sending the monitor payload
const MONITOR_UPDATE: u32 = 1;
// Voltage and current monitor also is once per second
const V_C_UPDATE: u32 = 2;

// Give the PAC and one unused interrupt due to one priority for the software tasks
#[app(device = pac, peripherals = true, dispatchers = [EVSYS])]
mod app {
    use super::*;

    type I2cBus = I2cProxy<'static, rtic::export::interrupt::Mutex<RefCell<bsp::I2c>>>;

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
        board_temp: f32,
        voltages: m_c::Voltages,
        currents: m_c::Currents,
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
        // Status LED
        rf1_stat_led: bsp::Rf1Led,
        rf2_stat_led: bsp::Rf2Led,
        // Power Monitor
        power_mon: PAC194X<I2cBus>,
        // Temperature Sensor
        temp_mon: TMP100<I2cBus>,
        // Delay
        delay: Delay,
    }

    #[monotonic(binds = RTC, default = true)]
    type RtcMonotonic = Rtc<Count32Mode>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Setup RTT
        rtt_init_print!();

        // Extract peripherals from context
        let mut peripherals = cx.device;
        let core = cx.core;

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

        // Delay Obj
        let mut delay = Delay::new(core.SYST, &mut clocks);

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
        let mut rf1_lna_en: bsp::Lna1En = pins.rf1_lna_en.into();
        let mut rf2_lna_en: bsp::Lna2En = pins.rf2_lna_en.into();

        // Set initial LNA powers to off
        rf1_lna_en.set_high().unwrap();
        rf2_lna_en.set_high().unwrap();

        // Configure attenuator
        let v1: bsp::V1 = pins.atten_v1.into();
        let v2: bsp::V2 = pins.atten_v2.into();
        let mut attenuator = atten::HMC291::new(v1, v2);

        attenuator.set_atten(atten::Attenuation::Twelve);

        // Configure I2C
        let i2c = bsp::i2c(
            &mut clocks,
            &mut peripherals.PM,
            peripherals.SERCOM2,
            pins.sda.into(),
            pins.scl.into(),
        );

        // We're going to be passing I2C around between devices, so we need to construct a shared bus
        let i2c_bus = shared_bus::new_cortexm!(bsp::I2c = i2c).unwrap();

        // For example, the first I2C object is the power sensor
        let power_mon = PAC194X::new(i2c_bus.acquire_i2c(), AddrSelect::VDD);

        // And next is the temperature sensor
        let temp_mon = TMP100::new(i2c_bus.acquire_i2c(), 0b1001000);

        // Configure PWM
        // Do we need these pins???
        let _rf1_cal: bsp::Rf1Pwm = pins.rf1_cal_tone.into();
        let _rf2_cal: bsp::Rf2Pwm = pins.rf2_cal_tone.into();

        let pwm0 = Pwm0::new(
            &clocks.tcc0_tcc1(&gclk0).unwrap(),
            LNA_CAL_ON_KHZ.khz(),
            peripherals.TCC0,
            &mut peripherals.PM,
        );

        let pwm2 = Pwm2::new(
            &clocks.tcc2_tc3(&gclk0).unwrap(),
            LNA_CAL_ON_KHZ.khz(),
            peripherals.TCC2,
            &mut peripherals.PM,
        );

        let mut cal_1 = calibration::LnaCalibration::new(pwm0, Channel::_2);
        let mut cal_2 = calibration::LnaCalibration::new(pwm2, Channel::_1);

        // Disable on startup
        cal_1.disable();
        cal_2.disable();

        // Schedule the periodic task of sending monitor data
        monitor::spawn_after(Duration::secs(MONITOR_UPDATE)).unwrap();
        update_voltage_currents::spawn_after(Duration::secs(V_C_UPDATE)).unwrap();
        rprintln!("FEM Initialized!");

        // Return initial values for shared and local resources
        (
            Shared {
                uart,
                rf1_lna_en,
                rf2_lna_en,
                attenuator,
                cal_1,
                cal_2,
                rf1_power: Default::default(),
                rf2_power: Default::default(),
                if_good_threshold: Default::default(),
                board_temp: Default::default(),
                voltages: Default::default(),
                currents: Default::default(),
            },
            Local {
                adc,
                log_det_1,
                log_det_2,
                byte_vec: Vec::<u8, BYTE_BUFF_SIZE>::new(),
                curly_counter: 0,
                is_reading: false,
                rf1_stat_led,
                rf2_stat_led,
                power_mon,
                temp_mon,
                delay,
            },
            init::Monotonics(rtc),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        // In the idle task, we update all of the monitor data
        // We set the status LED
        loop {
            update_if_powers::spawn().unwrap();
            update_status_led::spawn().unwrap();
            update_board_temp::spawn().unwrap();
        }
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
    }

    #[task(local = [temp_mon], shared = [board_temp])]
    fn update_board_temp(cx: update_board_temp::Context) {
        // Unpack context
        let mut board_temp = cx.shared.board_temp;
        let temp_mon = cx.local.temp_mon;
        // Grab updated values and update the shared resource
        board_temp.lock(|x| *x = temp_mon.temp_c().unwrap());
    }

    #[task(local = [power_mon], shared = [voltages, currents])]
    fn update_voltage_currents(cx: update_voltage_currents::Context) {
        // Unpack
        let power_mon = cx.local.power_mon;
        let mut voltages = cx.shared.voltages;
        let mut currents = cx.shared.currents;
        // Bus voltages
        let mut vbus = [0f32; 4];
        for i in 1..=4 {
            vbus[i - 1] = power_mon.read_bus_voltage_n(i as u8).unwrap();
        }
        // Sense voltages
        let mut vsense = [0f32; 4];
        for i in 1..=4 {
            vsense[i - 1] = power_mon.read_sense_voltage_n(i as u8).unwrap();
        }
        // Calculate currents
        // Channel Mapping
        // 1 - Input
        // 2 - LNA1
        // 3 - LNA2
        // 4 - Analog

        voltages.lock(|v| {
            v.raw_input = vbus[0];
            v.lna_one = vbus[1];
            v.lna_two = vbus[2];
            v.analog = vbus[3];
        });
        
        currents.lock(|c| {
            c.raw_input = vsense[0] / RSENSE_ANALOG_INPUT;
            c.lna_one = vsense[1] / RSENSE_LNA;
            c.lna_two = vsense[2] / RSENSE_LNA;
            c.analog = vsense[3] / RSENSE_ANALOG_INPUT;
        });
        
        // Refresh to get current values
        power_mon.refresh_v().unwrap();
        // Spawn self for future
        update_voltage_currents::spawn_after(Duration::secs(V_C_UPDATE)).unwrap();
    }

    #[task(shared = [if_good_threshold, cal_1, cal_2, rf1_power, rf2_power], local = [rf1_stat_led, rf2_stat_led])]
    fn update_status_led(cx: update_status_led::Context) {
        // Unpack Shared
        let if_good_threshold = cx.shared.if_good_threshold;
        let cal_1 = cx.shared.cal_1;
        let cal_2 = cx.shared.cal_2;
        let rf1_power = cx.shared.rf1_power;
        let rf2_power = cx.shared.rf2_power;
        // Unpack Local
        let rf1_stat_led = cx.local.rf1_stat_led;
        let rf2_stat_led = cx.local.rf2_stat_led;
        // If IF power is good, turn status ON, if cal is on, Blink
        // TODO Blink!
        (rf1_power, rf2_power, if_good_threshold).lock(|pow_1, pow_2, thresh| {
            if pow_1 >= thresh {
                rf1_stat_led.set_high().unwrap();
            } else {
                rf1_stat_led.set_low().unwrap();
            }
            if pow_2 >= thresh {
                rf2_stat_led.set_high().unwrap();
            } else {
                rf2_stat_led.set_low().unwrap();
            }
        });
    }

    #[task(local = [byte_vec, curly_counter, is_reading], shared = [uart], binds = SERCOM0)]
    fn handle_incoming_uart(mut cx: handle_incoming_uart::Context) {
        rprintln!("New UART Byte");
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
                    Err(_) => rprintln!("Invalid JSON Control Payload!"),
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
    fn control(cx: control::Context, payload: m_c::Control) {
        rprintln!("Got new control payload");
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

    #[task(shared = [rf1_power, rf2_power, attenuator, cal_1, cal_2, voltages, currents, uart, board_temp])]
    fn monitor(cx: monitor::Context) {
        rprintln!("Transmitting monitor payload");
        // Unpack context
        let mut board_temp = cx.shared.board_temp;
        let rf1_power = cx.shared.rf1_power;
        let rf2_power = cx.shared.rf2_power;
        let attenuator = cx.shared.attenuator;
        let cal_1 = cx.shared.cal_1;
        let cal_2 = cx.shared.cal_2;
        let mut voltages = cx.shared.voltages;
        let mut currents = cx.shared.currents;
        let mut uart = cx.shared.uart;

        // Build the payloads to serialize
        let if_power = (rf1_power, rf2_power).lock(|pow_1, pow_2| m_c::IfPower {
            channel_one: *pow_1,
            channel_two: *pow_2,
        });

        let status = (cal_1, cal_2, attenuator).lock(|cal_1, cal_2, atten| m_c::Status {
            cal_one: cal_1.is_enabled(),
            cal_two: cal_2.is_enabled(),
            attenuation_level: atten.get_atten() as u8,
        });

        let monitor = m_c::Monitor {
            voltages: voltages.lock(|v| v.clone()),
            currents: currents.lock(|c| c.clone()),
            status,
            board_temp: board_temp.lock(|t| t.clone()),
            if_power,
        };

        rprintln!("{:#?}", monitor);

        // Serialize and transmit

        let json_payload: Vec<u8, 1024> = json::to_vec(&monitor).unwrap();

        uart.lock(|uart| {
            for byte in json_payload {
                block!(uart.write(byte)).unwrap();
            }
        });

        // Schedule self for later
        monitor::spawn_after(Duration::secs(MONITOR_UPDATE)).unwrap();
    }
}
