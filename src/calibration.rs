use atsamd_hal::pwm::Channel;
use embedded_hal::{Pwm};
pub struct LnaCalibration<PWM> {
    pwm: PWM,
    channel: Channel,
    enabled: bool
}

impl<PWM> LnaCalibration<PWM>
where
    PWM: Pwm<Channel = Channel>,
{
    pub fn new(pwm: PWM, channel: Channel) -> Self {
        Self { pwm, channel, enabled: false }
    }

    pub fn disable(&mut self) {
        self.pwm.disable(self.channel);
        self.enabled = false;
    }

    pub fn enable(&mut self) {
        self.pwm.enable(self.channel);
        self.enabled = true;
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
}
