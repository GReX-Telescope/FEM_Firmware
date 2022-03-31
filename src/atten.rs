use embedded_hal::digital::v2::OutputPin;

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum Attenuation {
    Zero,
    Four,
    Eight,
    Twelve,
}

pub enum Error<E> {
    /// I2C bus error
    Gpio(E),
    /// Errors such as overflowing the stack.
    Internal,
}

pub struct HMC291<V1Pin, V2Pin> {
    v1: V1Pin,
    v2: V2Pin,
    state: Attenuation,
}

impl<E, V1Pin, V2Pin> HMC291<V1Pin, V2Pin>
where
    V1Pin: OutputPin<Error = E>,
    V2Pin: OutputPin<Error = E>,
{
    pub fn new(v1: V1Pin, v2: V2Pin) -> Self {
        Self {
            v1,
            v2,
            state: Attenuation::Zero,
        }
    }

    pub fn set_atten(&mut self, atten: Attenuation) -> Result<(), Error<E>> {
        self.state = atten;
        match atten {
            Attenuation::Zero => {
                self.v1.set_high().map_err(Error::Gpio)?;
                self.v2.set_high().map_err(Error::Gpio)?;
            }
            Attenuation::Four => {
                self.v1.set_high().map_err(Error::Gpio)?;
                self.v2.set_low().map_err(Error::Gpio)?;
            }
            Attenuation::Eight => {
                self.v1.set_low().map_err(Error::Gpio)?;
                self.v2.set_high().map_err(Error::Gpio)?;
            }
            Attenuation::Twelve => {
                self.v1.set_low().map_err(Error::Gpio)?;
                self.v2.set_low().map_err(Error::Gpio)?;
            }
        };
        Ok(())
    }

    pub fn get_atten(&self) -> Attenuation {
        self.state
    }
}
