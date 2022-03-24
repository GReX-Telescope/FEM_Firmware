use embedded_hal::digital::v2::OutputPin;

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum Attenuation {
    Zero,
    Four,
    Eight,
    Twelve,
}

pub struct HMC291<V1Pin, V2Pin> {
    v1: V1Pin,
    v2: V2Pin,
    state: Attenuation,
}

impl<V1Pin, V2Pin> HMC291<V1Pin, V2Pin>
where
    V1Pin: OutputPin,
    V2Pin: OutputPin,
{
    pub fn new(v1: V1Pin, v2: V2Pin) -> Self {
        let mut s = Self {
            v1,
            v2,
            state: Attenuation::Zero,
        };
        s.set_atten(Attenuation::Zero);
        s
    }

    pub fn set_atten(&mut self, atten: Attenuation) {
        self.state = atten;
        match atten {
            Attenuation::Zero => {
                self.v1.set_low();
                self.v2.set_low();
            }
            Attenuation::Four => {
                self.v1.set_low();
                self.v2.set_high();
            }
            Attenuation::Eight => {
                self.v1.set_high();
                self.v2.set_low();
            }
            Attenuation::Twelve => {
                self.v1.set_high();
                self.v2.set_high();
            }
        };
    }

    pub fn get_atten(&self) -> Attenuation {
        self.state
    }
}
