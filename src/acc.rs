use crate::{
    AccelDataRate, AccelError, AccelFilterBandwidth, AccelRange, Bmi088Accelerometer,
    Bmi088PinBehavior, Bmi088PinMode,
};
use embedded_hal::i2c::blocking::I2c;

const ACC_CHIP_ID: u8 = 0x00;
const ACC_ERR_REG: u8 = 0x02;
const ACC_STATUS: u8 = 0x03;
const ACC_X_LSB: u8 = 0x12;
const ACC_INT_STAT_1: u8 = 0x1D;
const ACC_CONFIG: u8 = 0x40;
const ACC_RANGE: u8 = 0x41;
const INT1_IO_CONF: u8 = 0x53;
const INT2_IO_CONF: u8 = 0x53;
const INT1_INT2_MAP_DATA: u8 = 0x58;
const ACC_PWR_CONF: u8 = 0x7C;
const ACC_PWR_CTRL: u8 = 0x7D;
const ACC_SOFTRESET: u8 = 0x7E;

impl<I2C: I2c> Bmi088Accelerometer<I2C> {
    pub fn read_chip_id(&mut self) -> Result<u8, I2C::Error> {
        let mut data = [0u8; 1];
        self.i2c
            .write_read(self.address, &[ACC_CHIP_ID], &mut data)
            .map(|_| data[0])
    }

    pub fn read_errors(&mut self) -> Result<AccelError, I2C::Error> {
        let mut data = [0u8; 1];
        self.i2c
            .write_read(self.address, &[ACC_ERR_REG], &mut data)
            .map(|_| AccelError {
                configuration_error: data[0] & 0x0C != 0,
                fatal_error: data[0] & 0x02 != 0,
            })
    }

    pub fn is_data_ready(&mut self) -> Result<bool, I2C::Error> {
        let mut data = [0u8; 1];
        self.i2c
            .write_read(self.address, &[ACC_STATUS], &mut data)
            .map(|_| data[0] & 0x80 != 0)
    }

    pub fn read_data(&mut self) -> Result<(i16, i16, i16), I2C::Error> {
        let mut data = [0u8; 6];
        self.i2c
            .write_read(self.address, &[ACC_X_LSB], &mut data)
            .map(|_| {
                (
                    (i16::from(data[1]) << 8) | i16::from(data[0]),
                    (i16::from(data[3]) << 8) | i16::from(data[2]),
                    (i16::from(data[5]) << 8) | i16::from(data[4]),
                )
            })
    }

    pub fn data_ready_interrupt(&mut self) -> Result<bool, I2C::Error> {
        let mut data = [0u8; 1];
        self.i2c
            .write_read(self.address, &[ACC_INT_STAT_1], &mut data)
            .map(|_| data[0] & 0x80 != 0)
    }

    pub fn set_bandwidth(&mut self, bandwidth: AccelFilterBandwidth) -> Result<(), I2C::Error> {
        let mut data = [0_u8; 1];

        // Read the exisitng register
        self.i2c
            .write_read(self.address, &[ACC_CONFIG], &mut data)?;

        // Clear and set the bandwidth bits
        data[0] &= 0b1000_1111;
        data[0] |= (bandwidth as u8) << 4;

        self.i2c.write(self.address, &[ACC_CONFIG, data[0]])
    }

    /// Reads the bandwidth setting from the device
    pub fn read_bandwidth(&mut self) -> Result<AccelFilterBandwidth, I2C::Error> {
        let mut data = [0_u8; 1];

        self.i2c
            .write_read(self.address, &[ACC_CONFIG], &mut data)?;

        data[0] &= 0b0111_0000;
        data[0] >>= 4;

        self.bandwidth = match data[0] {
            0x00 => AccelFilterBandwidth::OSR4,
            0x01 => AccelFilterBandwidth::OSR2,
            0x02 => AccelFilterBandwidth::Normal,
            _ => unreachable!(),
        };

        Ok(self.bandwidth)
    }

    pub fn get_bandwidth(&self) -> AccelFilterBandwidth {
        self.bandwidth
    }

    pub fn set_data_rate(&mut self, data_rate: AccelDataRate) -> Result<(), I2C::Error> {
        let mut data = [0_u8; 1];

        // Read the exisitng register
        self.i2c
            .write_read(self.address, &[ACC_CONFIG], &mut data)?;

        // Clear and set the bandwidth bits
        data[0] &= 0b1111_0000;
        data[0] |= data_rate as u8;

        self.i2c.write(self.address, &[ACC_CONFIG, data[0]])
    }

    /// Reads the data rate setting from the device
    pub fn read_data_rate(&mut self) -> Result<AccelDataRate, I2C::Error> {
        let mut data = [0_u8; 1];

        self.i2c
            .write_read(self.address, &[ACC_CONFIG], &mut data)?;

        data[0] &= 0b0000_1111;

        self.data_rate = match data[0] {
            0x05 => AccelDataRate::Hz12_5,
            0x06 => AccelDataRate::Hz25,
            0x07 => AccelDataRate::Hz50,
            0x08 => AccelDataRate::Hz100,
            0x09 => AccelDataRate::Hz200,
            0x0A => AccelDataRate::Hz400,
            0x0B => AccelDataRate::Hz800,
            0x0C => AccelDataRate::Hz1600,
            _ => unreachable!(),
        };

        Ok(self.data_rate)
    }

    pub fn get_data_rate(&self) -> AccelDataRate {
        self.data_rate
    }

    pub fn set_range(&mut self, range: AccelRange) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[ACC_RANGE, range as u8])
    }

    /// Reads the range setting from the device
    pub fn read_range(&mut self) -> Result<AccelRange, I2C::Error> {
        let mut data = [0_u8; 1];

        self.i2c.write_read(self.address, &[ACC_RANGE], &mut data)?;

        self.range = match data[0] {
            0x00 => AccelRange::G3,
            0x01 => AccelRange::G6,
            0x02 => AccelRange::G12,
            0x03 => AccelRange::G24,
            _ => unreachable!(),
        };

        Ok(self.range)
    }

    pub fn get_range(&self) -> AccelRange {
        self.range
    }

    pub fn configure_int1_pin(
        &mut self,
        mode: Bmi088PinMode,
        behavior: Bmi088PinBehavior,
        active_high: bool,
        map_interrupt: bool,
    ) -> Result<(), I2C::Error> {
        let mut data = [0_u8; 1];

        // Configure the IO_CONF register

        match mode {
            Bmi088PinMode::Input => data[0] |= 0b0001_0000,
            Bmi088PinMode::Output => data[0] |= 0b0000_1000,
            Bmi088PinMode::Disabled => {}
        }

        if let Bmi088PinBehavior::OpenDrain = behavior {
            data[0] |= 0b0000_0100;
        }

        if active_high {
            data[0] |= 0b0000_0010;
        }

        self.i2c.write(self.address, &[INT1_IO_CONF, data[0]])?;

        // Configure the MAP register

        self.i2c
            .write_read(self.address, &[INT1_INT2_MAP_DATA], &mut data)?;

        if map_interrupt {
            data[0] |= 0b0000_0100;
        } else {
            data[0] &= 0b1111_1011;
        }

        self.i2c.write(self.address, &[INT1_INT2_MAP_DATA, data[0]])
    }

    pub fn configure_int2_pin(
        &mut self,
        mode: Bmi088PinMode,
        behavior: Bmi088PinBehavior,
        active_high: bool,
        map_interrupt: bool,
    ) -> Result<(), I2C::Error> {
        let mut data = [0_u8; 1];

        // Configure the IO_CONF register

        match mode {
            Bmi088PinMode::Input => data[0] |= 0b0001_0000,
            Bmi088PinMode::Output => data[0] |= 0b0000_1000,
            Bmi088PinMode::Disabled => {}
        }

        if let Bmi088PinBehavior::OpenDrain = behavior {
            data[0] |= 0b0000_0100;
        }

        if active_high {
            data[0] |= 0b0000_0010;
        }

        self.i2c.write(self.address, &[INT2_IO_CONF, data[0]])?;

        // Configure the MAP register

        self.i2c
            .write_read(self.address, &[INT1_INT2_MAP_DATA], &mut data)?;

        if map_interrupt {
            data[0] |= 0b0100_0000;
        } else {
            data[0] &= 0b1011_1111;
        }

        self.i2c.write(self.address, &[INT1_INT2_MAP_DATA, data[0]])
    }

    pub fn set_suspended(&mut self, suspended: bool) -> Result<(), I2C::Error> {
        let data = if suspended { 0x03 } else { 0x00 };
        self.i2c.write(self.address, &[ACC_PWR_CONF, data])
    }

    pub fn set_on(&mut self, on: bool) -> Result<(), I2C::Error> {
        let data = if on { 0x04 } else { 0x00 };
        self.i2c.write(self.address, &[ACC_PWR_CTRL, data])
    }

    pub fn reset(&mut self) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[ACC_SOFTRESET, 0xB6])
    }
}
