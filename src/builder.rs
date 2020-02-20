
use crate::interface::{I2cInterface, SpiInterface};

struct Builder {

}

impl Builder {

    /// Finish the builder and use I2C to communicate with the sensor
    pub fn connect_i2c<I2C, CommE>(&self, i2c: I2C) -> I2cInterface<I2C>
        where
            I2C: embedded_hal::blocking::i2c::Write<Error = CommE>,
    {
        I2cInterface::new(i2c, self.i2c_addr);
    }


    /// Finish the builder and use SPI to communicate with the display
    pub fn connect_spi<SPI, CommE>(
        &self,
        spi: SPI,
    ) -> SpiInterface<SPI, CommE>
        where
            SPI: embedded_hal::blocking::spi::Transfer<u8, Error = CommE>
            + embedded_hal::blocking::spi::Write<u8, Error = CommE>,
    {
        SpiInterface::new(spi)
    }
}