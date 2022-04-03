use atsamd_hal::pwm::Channel;
use embedded_hal::Pwm;
pub struct LnaCalibration<PWM> {
    pwm: PWM,
    channel: Channel,
    enabled: bool,
    max_duty: u32,
}

impl<PWM> LnaCalibration<PWM>
where
    PWM: Pwm<Channel = Channel, Duty = u32>,
{
    pub fn new(pwm: PWM, channel: Channel) -> Self {
        let max_duty = pwm.get_max_duty();
        Self {
            pwm,
            channel,
            enabled: false,
            max_duty,
        }
    }

    pub fn disable(&mut self) {
        self.pwm.disable(self.channel);
        self.enabled = false;
    }

    pub fn enable(&mut self) {
        self.pwm.set_duty(self.channel, self.max_duty / 2);
        self.pwm.enable(self.channel);
        self.enabled = true;
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
}
