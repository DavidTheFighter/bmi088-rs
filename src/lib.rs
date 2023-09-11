#![forbid(unsafe_code)]
#![cfg_attr(not(test), no_std)]

pub mod acc;
pub mod gyro;

use embedded_hal::i2c::blocking::I2c;

const TEMP_MSB: u8 = 0x22;

#[derive(Debug, Clone, Copy)]
pub struct AccelError {
    pub configuration_error: bool,
    pub fatal_error: bool,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum AccelFilterBandwidth {
    OSR4 = 0x00,
    OSR2 = 0x01,
    Normal = 0x02,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum AccelDataRate {
    Hz12_5 = 0x05,
    Hz25 = 0x06,
    Hz50 = 0x07,
    Hz100 = 0x08,
    Hz200 = 0x09,
    Hz400 = 0x0A,
    Hz800 = 0x0B,
    Hz1600 = 0x0C,
}

#[derive(Debug, Copy, Clone)]
pub enum AccelRange {
    G3 = 0x00,
    G6 = 0x01,
    G12 = 0x02,
    G24 = 0x03,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GyroRange {
    Deg2000 = 0x00,
    Deg1000 = 0x01,
    Deg500 = 0x02,
    Deg250 = 0x03,
    Deg125 = 0x04,
}

#[derive(Debug, Copy, Clone)]
pub enum GyroBandwidth {
    Data2000Filter532 = 0x00,
    Data2000Filter230 = 0x01,
    Data1000Filter116 = 0x02,
    Data400Filter47 = 0x03,
    Data200Filter23 = 0x04,
    Data100Filter12 = 0x05,
    Data200Filter64 = 0x06,
    Data100Filter32 = 0x07,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Bmi088PinMode {
    Input,
    Output,
    Disabled,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Bmi088PinBehavior {
    PushPull,
    OpenDrain,
}

#[derive(Debug)]
pub struct Bmi088Accelerometer<I2C> {
    i2c: I2C,
    address: u8,
    bandwidth: AccelFilterBandwidth,
    data_rate: AccelDataRate,
    range: AccelRange,
}

#[derive(Debug)]
pub struct Bmi088Gyroscope<I2C> {
    i2c: I2C,
    address: u8,
    bandwidth: GyroBandwidth,
    range: GyroRange,
}

impl<I2C: I2c> Bmi088Accelerometer<I2C> {
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            bandwidth: AccelFilterBandwidth::Normal,
            data_rate: AccelDataRate::Hz100,
            range: AccelRange::G6,
        }
    }

    pub fn read_temperature(&mut self) -> Result<i16, I2C::Error> {
        let mut data = [0u8; 2];
        self.i2c
            .write_read(self.address, &[TEMP_MSB], &mut data)
            .map(|_| (i16::from(data[1]) << 8) | i16::from(data[0]))
    }
}

impl<I2C: I2c> Bmi088Gyroscope<I2C> {
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            bandwidth: GyroBandwidth::Data2000Filter532,
            range: GyroRange::Deg2000,
        }
    }
}
