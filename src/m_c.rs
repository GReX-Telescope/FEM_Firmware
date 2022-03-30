use serde_derive::{Deserialize, Serialize};

#[derive(Debug, PartialEq, Serialize, Default, Clone, Copy)]
pub struct Voltages {
    pub raw_input: f32,
    pub analog: f32,
    pub lna_one: f32,
    pub lna_two: f32,
}

#[derive(Debug, PartialEq, Serialize, Default, Clone, Copy)]
pub struct Currents {
    pub raw_input: f32,
    pub analog: f32,
    pub lna_one: f32,
    pub lna_two: f32,
}

#[derive(Debug, PartialEq, Serialize)]
pub struct IfPower {
    pub channel_one: f32,
    pub channel_two: f32,
}

#[derive(Debug, PartialEq, Deserialize)]
pub struct Control {
    pub cal_one: bool,
    pub cal_two: bool,
    pub lna_one_powered: bool,
    pub lna_two_powered: bool,
    pub attenuation_level: u8,
    pub if_power_threshold: f32,
}

#[derive(Debug, PartialEq, Serialize)]
pub struct Status {
    pub cal_one: bool,
    pub cal_two: bool,
    pub attenuation_level: u8,
}

#[derive(Debug, PartialEq, Serialize)]
pub struct Monitor {
    pub board_temp: f32,
    pub voltages: Voltages,
    pub currents: Currents,
    pub status: Status,
    pub if_power: IfPower,
}
