use atsamd_hal as hal;

use hal::clock::GenericClockController;
use hal::prelude::*;
use hal::sercom::{i2c, uart, Sercom0, Sercom2};
use uart::{BaudMode, Oversampling};

use hal::pac;
use pac::{PM, SERCOM0, SERCOM2};

hal::bsp_pins!(
    // GPIO
    PA07 {
        name: rf1_stat_led,
        aliases: { PushPullOutput: Rf1Led }
    },
    PA06 {
        name: rf2_stat_led,
        aliases: { PushPullOutput: Rf2Led }
    },
    PA14 { name: alert_1 },
    PA15 { name: alert_2 },
    PA23 {
        name: rf1_lna_en,
        aliases: { PushPullOutput: Lna1En }
    },
    PA22 {
        name: rf2_lna_en,
        aliases: { PushPullOutput: Lna2En }
    },
    PA24 {
        name: atten_v1,
        aliases: { PushPullOutput: V1 }
    },
    PA25 {
        name: atten_v2,
        aliases: { PushPullOutput: V2 }
    },
    // PWM
    PA18 {
        name: rf1_cal_tone,
        aliases: { AlternateF: Rf1Pwm }
    }, //TCC0/WO[2]
    PA17 {
        name: rf2_cal_tone,
        aliases: { AlternateE: Rf2Pwm }
    }, //TCC2/WO[1]
    // ADC
    PB02 {
        name: rf1_if_pow,
        aliases: { AlternateB: AIN10 }
    }, //AIN[10]
    PB03 {
        name: rf2_if_pow,
        aliases: { AlternateB: AIN11 }
    }, //AIN[11]
    // I2C
    PA08 {
        name: sda,
        aliases: { AlternateD: Sda }
    }, //ALTD_SERCOM2[0]
    PA09 {
        name: scl,
        aliases: { AlternateD: Scl }
    }, //ALTD_SERCOM2[1]
    // UART
    PA10 {
        name: txd,
        aliases: { AlternateC: UartTx }
    }, //ALTC_SERCOM0[2]
    PA11 {
        name: rxd,
        aliases: { AlternateC: UartRx }
    }, //ALTC_SERCOM0[3]
);

pub type UartPads = uart::Pads<Sercom0, UartRx, UartTx>;
pub type UartConfig = uart::Config<UartPads, uart::EightBit>;
pub type UartStruct = uart::Uart<UartConfig, uart::Duplex>;

pub fn uart(
    clocks: &mut GenericClockController,
    pm: &mut PM,
    rx: UartRx,
    tx: UartTx,
    sercom: SERCOM0,
) -> UartStruct {
    let gclk0 = clocks.gclk0();
    let clock = clocks.sercom0_core(&gclk0).unwrap();
    let freq = clock.freq();

    let pads: UartPads = uart::Pads::<Sercom0>::default().rx(rx).tx(tx);
    let config: UartConfig = uart::Config::new(pm, sercom, pads, freq)
        .baud(115200.hz(), BaudMode::Fractional(Oversampling::Bits16));

    config.enable()
}

pub type I2cPads = i2c::Pads<Sercom2, Sda, Scl>;
pub type I2c = i2c::I2c<i2c::Config<I2cPads>>;
pub fn i2c(
    clocks: &mut GenericClockController,
    pm: &mut PM,
    sercom: SERCOM2,
    sda: Sda,
    scl: Scl,
) -> I2c {
    let gclk0 = clocks.gclk0();
    let clock = &clocks.sercom2_core(&gclk0).unwrap();
    let freq = clock.freq();

    let pads: I2cPads = i2c::Pads::new(sda, scl);
    i2c::Config::new(pm, sercom, pads, freq)
        .baud(100.khz())
        .enable()
}
