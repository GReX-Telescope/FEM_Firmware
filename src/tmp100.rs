use embedded_hal::blocking::i2c;

/// Driver errors.
#[derive(Debug, PartialEq)]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),
    /// Errors such as overflowing the stack.
    Internal,
}

pub struct TMP100<I> {
    i2c: I,
    addr: u8,
}

impl<E, I> TMP100<I>
where
    I: i2c::Read<Error = E> + i2c::Write<Error = E> + i2c::WriteRead<Error = E>,
{
    pub fn new(i2c: I, addr: u8) -> Self {
        Self { i2c, addr }
    }

    pub fn init(&mut self) -> Result<(), Error<E>> {
        // Select configuration register and set continuous conversion, comparator mode, 12-bit resolution
        let payload: [u8; 2] = [0x01, 0x60];
        self.i2c.write(self.addr, &payload).map_err(Error::I2c)
    }

    pub fn temp_c(&mut self) -> Result<f32, Error<E>> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(self.addr, &[0u8], &mut buf)
            .map_err(Error::I2c)?;
        Ok((((buf[0] as u16 * 256) + (buf[1] & 0xF0) as u16) / 16) as f32 * 0.0625)
    }
}
