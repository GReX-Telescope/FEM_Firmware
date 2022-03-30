use atsamd_hal::adc::Adc;
use atsamd_hal::pac::ADC;
use embedded_hal::adc::{Channel, OneShot};

const ON_CHIP_PULLDOWN: f32 = 7.2e3;
const INTERNAL_SLOPE: f32 = 3.4e-6;

fn parallel(z1: f32, z2: f32) -> f32 {
    (z1 * z2) / (z1 + z2)
}

fn scale_adc_reading(load_res: f32, adc_value: u16, intercept: f32) -> f32 {
    let slope = INTERNAL_SLOPE * parallel(load_res.into(), ON_CHIP_PULLDOWN);
    slope * (adc_value as f32) + intercept
}

pub struct LT5537<P> {
    intercept: f32,
    load_resistance: f32,
    pin: P,
}

impl<P> LT5537<P>
where
    P: Channel<ADC, ID = u8>,
{
    pub fn new(load_resistance: f32, pin: P) -> Self {
        Self {
            intercept: -94.0,
            load_resistance,
            pin,
        }
    }

    pub fn read(&mut self, adc: &mut Adc<ADC>) -> f32 {
        let raw_adc_value: u16 = adc.read(&mut self.pin).unwrap();
        scale_adc_reading(self.load_resistance, raw_adc_value, self.intercept)
    }
}
