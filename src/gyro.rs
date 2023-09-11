use crate::{Bmi088PinBehavior, GyroBandwidth, GyroRange, Bmi088Gyroscope};
use embedded_hal::i2c::blocking::I2c;

const GYRO_CHIP_ID: u8 = 0x00;
const GYRO_X_LSB: u8 = 0x02;
const GYRO_INT_STAT_1: u8 = 0x0A;
const GYRO_RANGE: u8 = 0x0F;
const GYRO_BANDWIDTH: u8 = 0x10;
const GYRO_LPM1: u8 = 0x11;
const GYRO_SOFTRESET: u8 = 0x14;
const GYRO_INT_CTRL: u8 = 0x15;
const GYRO_INT3_INT4_IO_CONF: u8 = 0x16;
const GYRO_INT3_INT4_IO_MAP: u8 = 0x18;

impl<I2C: I2c> Bmi088Gyroscope<I2C> {
    pub fn read_chip_id(&mut self) -> Result<u8, I2C::Error> {
        let mut data = [0u8; 1];
        self.i2c
            .write_read(self.address, &[GYRO_CHIP_ID], &mut data)
            .map(|_| data[0])
    }

    pub fn read_data(&mut self) -> Result<(i16, i16, i16), I2C::Error> {
        let mut data = [0u8; 6];
        self.i2c
            .write_read(self.address, &[GYRO_X_LSB], &mut data)
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
            .write_read(self.address, &[GYRO_INT_STAT_1], &mut data)
            .map(|_| data[0] & 0x80 != 0)
    }

    pub fn set_range(&mut self, range: GyroRange) -> Result<(), I2C::Error> {
        self.i2c
            .write(self.address, &[GYRO_RANGE, range as u8])
    }

    pub fn get_range(&mut self) -> Result<GyroRange, I2C::Error> {
        let mut data = [0u8; 1];
        self.i2c
            .write_read(self.address, &[GYRO_RANGE], &mut data)
            .map(|_| match data[0] {
                0x00 => GyroRange::Deg2000,
                0x01 => GyroRange::Deg1000,
                0x02 => GyroRange::Deg500,
                0x03 => GyroRange::Deg250,
                0x04 => GyroRange::Deg125,
                _ => unreachable!(),
            })
    }

    pub fn set_bandwidth(&mut self, bandwidth: GyroBandwidth) -> Result<(), I2C::Error> {
        self.i2c
            .write(self.address, &[GYRO_BANDWIDTH, bandwidth as u8])
    }

    pub fn get_bandwidth(&mut self) -> Result<GyroBandwidth, I2C::Error> {
        let mut data = [0u8; 1];
        self.i2c
            .write_read(self.address, &[GYRO_BANDWIDTH], &mut data)
            .map(|_| match data[0] & (!0x80) {
                // Bit 7 is always 1 and should be ignored
                0x00 => GyroBandwidth::Data2000Filter532,
                0x01 => GyroBandwidth::Data2000Filter230,
                0x02 => GyroBandwidth::Data1000Filter116,
                0x03 => GyroBandwidth::Data400Filter47,
                0x04 => GyroBandwidth::Data200Filter23,
                0x05 => GyroBandwidth::Data100Filter12,
                0x06 => GyroBandwidth::Data200Filter64,
                0x07 => GyroBandwidth::Data100Filter32,
                _ => unreachable!(),
            })
    }

    pub fn set_on(&mut self, on: bool) -> Result<(), I2C::Error> {
        let data = if on { 0x00 } else { 0x80 };
        self.i2c.write(self.address, &[GYRO_LPM1, data])
    }

    pub fn reset(&mut self) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[GYRO_SOFTRESET, 0xB6])
    }

    pub fn configure_int3_pin(
        &mut self,
        behavior: Bmi088PinBehavior,
        active_high: bool,
        map_interrupt: bool,
    ) -> Result<(), I2C::Error> {
        let mut data = [0_u8; 1];

        // Just enable interrupts, if the pins don't map them it doesn't do anything
        self.i2c.write(self.address, &[GYRO_INT_CTRL, 0x80])?;

        // Configure the IO_CONF register

        self.i2c
            .write_read(self.address, &[GYRO_INT3_INT4_IO_CONF], &mut data)?;

        if let Bmi088PinBehavior::OpenDrain = behavior {
            data[0] |= 0b0000_0010;
        } else {
            data[0] &= 0b1111_1101;
        }

        if active_high {
            data[0] |= 0b0000_0001;
        } else {
            data[0] &= 0b1111_1110;
        }

        self.i2c
            .write(self.address, &[GYRO_INT3_INT4_IO_CONF, data[0]])?;

        // Configure the MAP register

        self.i2c
            .write_read(self.address, &[GYRO_INT3_INT4_IO_MAP], &mut data)?;

        if map_interrupt {
            data[0] |= 0b0000_0001;
        } else {
            data[0] &= 0b1111_1110;
        }

        self.i2c
            .write(self.address, &[GYRO_INT3_INT4_IO_MAP, data[0]])
    }

    pub fn configure_int4_pin(
        &mut self,
        behavior: Bmi088PinBehavior,
        active_high: bool,
        map_interrupt: bool,
    ) -> Result<(), I2C::Error> {
        let mut data = [0_u8; 1];

        // Just enable interrupts, if the pins don't map them it doesn't do anything
        self.i2c.write(self.address, &[GYRO_INT_CTRL, 0x80])?;

        // Configure the IO_CONF register

        self.i2c
            .write_read(self.address, &[GYRO_INT3_INT4_IO_CONF], &mut data)?;

        if let Bmi088PinBehavior::OpenDrain = behavior {
            data[0] |= 0b0000_1000;
        } else {
            data[0] &= 0b1111_0111;
        }

        if active_high {
            data[0] |= 0b0000_0100;
        } else {
            data[0] &= 0b1111_1011;
        }

        self.i2c
            .write(self.address, &[GYRO_INT3_INT4_IO_CONF, data[0]])?;

        // Configure the MAP register

        self.i2c
            .write_read(self.address, &[GYRO_INT3_INT4_IO_MAP], &mut data)?;

        if map_interrupt {
            data[0] |= 0b1000_0000;
        } else {
            data[0] &= 0b0111_1111;
        }

        self.i2c
            .write(self.address, &[GYRO_INT3_INT4_IO_MAP, data[0]])
    }
}
