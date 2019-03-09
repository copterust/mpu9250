use ak8963::{self, AK8963};

use core::mem;

use hal::blocking::delay::DelayMs;
use hal::blocking::i2c;
use hal::blocking::spi;
use hal::digital::OutputPin;

use generic_array::typenum::consts::*;
use generic_array::{ArrayLength, GenericArray};

use Register;

/// MPU's I2C address (AD0 low)
const MPU_I2C_ADDR: u8 = 0x68;

/// Releasable describes a type that can be destroyed
/// with a released asset.
pub trait Releasable {
    /// The type to be released
    type Released;

    /// Release the underlying asset
    fn release(self) -> Self::Released;
}

/// An MPU communication device abstraction
///
/// This allows us to generalize the device over either an I2C or SPI peripheral
pub trait Device: Releasable {
    /// The type of error for all results
    type Error;

    /// Read many values from register
    fn read_many<N: ArrayLength<u8>>(
        &mut self,
        reg: Register)
        -> Result<GenericArray<u8, N>, Self::Error>;

    /// Write the provided value to register
    fn write(&mut self, reg: Register, val: u8) -> Result<(), Self::Error>;

    /// Read a single value from the register
    fn read(&mut self, reg: Register) -> Result<u8, Self::Error> {
        let buffer = self.read_many::<U2>(reg)?;
        Ok(buffer[1])
    }

    /// Modify the value in the register using the provided closure. The closure
    /// accepts the current value of the register, permitting conditional checks
    /// before modification.
    fn modify<F>(&mut self, reg: Register, f: F) -> Result<(), Self::Error>
        where F: FnOnce(u8) -> u8
    {
        let r = self.read(reg)?;
        self.write(reg, f(r))?;

        Ok(())
    }
}

/// A SPI device. Use a SPI device when the MPU9250 is
/// connected via SPI
pub struct SpiDevice<SPI, GPIO> {
    /// Underlying peripheral
    spi: SPI,
    /// nCS
    ncs: GPIO,
}

impl<SPI, NCS, E> SpiDevice<SPI, NCS>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          NCS: OutputPin
{
    /// Create a new SpiDevice
    pub fn new(spi: SPI, ncs: NCS) -> Self {
        SpiDevice { spi,
                    ncs }
    }
}

impl<SPI, NCS, E> Releasable for SpiDevice<SPI, NCS>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          NCS: OutputPin
{
    type Released = (SPI, NCS);

    fn release(self) -> (SPI, NCS) {
        (self.spi, self.ncs)
    }
}

impl<SPI, NCS, E> Device for SpiDevice<SPI, NCS>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          NCS: OutputPin
{
    type Error = E;

    // Note: implementation is consistent with previous Mpu9250 private
    // methods. Using read and modify as default trait impls

    fn read_many<N>(&mut self, reg: Register) -> Result<GenericArray<u8, N>, E>
        where N: ArrayLength<u8>
    {
        let mut buffer: GenericArray<u8, N> = unsafe { mem::zeroed() };
        {
            let slice: &mut [u8] = &mut buffer;
            slice[0] = reg.read_address();
            self.ncs.set_low();
            self.spi.transfer(slice)?;
            self.ncs.set_high();
        }

        Ok(buffer)
    }

    // XXX: It seems that without inline(never), when compiling
    //      with opt-level:3, ncs is out of sync with transfer.
    //      This should be probably solved in HAL or HAL impl,
    //      as device drivers should not know about such
    //      minutiæ details.
    #[inline(never)]
    fn write(&mut self, reg: Register, val: u8) -> Result<(), E> {
        self.ncs.set_low();
        self.spi.write(&[reg.write_address(), val])?;
        self.ncs.set_high();
        Ok(())
    }
}

impl<SPI, NCS, E> AK8963 for SpiDevice<SPI, NCS>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          NCS: OutputPin
{
    type Error = E;

    fn init<D: DelayMs<u8>>(&mut self,
                            delay: &mut D)
                            -> Result<(), Self::Error> {
        // Isolate the auxiliary master I2C bus (AUX_CL, AUX_DA)
        // disable the slave I2C bus, make serial interface SPI only
        // reset the master I2C bus
        Device::write(self, Register::USER_CTRL, 0x32)?;
        delay.delay_ms(10);
        Ok(())
    }

    fn read(&mut self, reg: ak8963::Register) -> Result<u8, Self::Error> {
        Device::write(self,
                      Register::I2C_SLV4_ADDR,
                      ak8963::I2C_ADDRESS | ak8963::R)?;
        Device::write(self, Register::I2C_SLV4_REG, reg.addr())?;

        // start transfer
        Device::write(self, Register::I2C_SLV4_CTRL, 0x80)?;

        // wait until transfer is over
        while Device::read(self, Register::I2C_MST_STATUS)? & (1 << 6) == 0 {}

        Device::read(self, Register::I2C_SLV4_DI)
    }

    fn write(&mut self,
             reg: ak8963::Register,
             value: u8)
             -> Result<(), Self::Error> {
        Device::write(self,
                      Register::I2C_SLV4_ADDR,
                      ak8963::I2C_ADDRESS | ak8963::W)?;
        Device::write(self, Register::I2C_SLV4_REG, reg.addr())?;
        Device::write(self, Register::I2C_SLV4_DO, value)?;

        // start transfer
        Device::write(self, Register::I2C_SLV4_CTRL, 0x80)?;

        // wait until transfer is over
        while Device::read(self, Register::I2C_MST_STATUS)? & (1 << 6) == 0 {}

        Ok(())
    }

    fn read_xyz(&mut self) -> Result<GenericArray<u8, U7>, Self::Error> {
        Device::read_many::<U7>(self, Register::EXT_SENS_DATA_00)
    }
}

/// An I2C device. Use I2CDevice when the
/// MPU9250 is connected via I2C
pub struct I2cDevice<I2C> {
    i2c: I2C,
}

impl<E, I2C> I2cDevice<I2C>
    where I2C: i2c::Read<Error = E>
              + i2c::Write<Error = E>
              + i2c::WriteRead<Error = E>
{
    /// Create a new I2C device
    pub fn new(i2c: I2C) -> Self {
        I2cDevice { i2c }
    }
}

impl<E, I2C> Releasable for I2cDevice<I2C>
    where I2C: i2c::Read<Error = E>
              + i2c::Write<Error = E>
              + i2c::WriteRead<Error = E>
{
    type Released = I2C;

    fn release(self) -> I2C {
        self.i2c
    }
}

impl<E, I2C> Device for I2cDevice<I2C>
    where I2C: i2c::Read<Error = E>
              + i2c::Write<Error = E>
              + i2c::WriteRead<Error = E>
{
    type Error = E;

    fn read_many<N: ArrayLength<u8>>(
        &mut self,
        reg: Register)
        -> Result<GenericArray<u8, N>, Self::Error> {
        let mut buffer: GenericArray<u8, N> = unsafe { mem::zeroed() };
        {
            let slice: &mut [u8] = &mut buffer;
            self.i2c.write_read(MPU_I2C_ADDR,
                                 &[reg as u8],
                                 &mut slice[1..])?;
        }

        Ok(buffer)
    }

    fn write(&mut self, reg: Register, val: u8) -> Result<(), Self::Error> {
        let buff: [u8; 2] = [reg as u8, val];
        self.i2c.write(MPU_I2C_ADDR, &buff)
    }
}

impl<I2C, E> AK8963 for I2cDevice<I2C>
    where I2C: i2c::Read<Error = E>
              + i2c::Write<Error = E>
              + i2c::WriteRead<Error = E>
{
    type Error = E;

    fn init<D: DelayMs<u8>>(&mut self,
                            delay: &mut D)
                            -> Result<(), Self::Error> {
        log::trace!("Zeroing USER_CTRL...");
        Device::write(self, Register::USER_CTRL, 0)?;
        delay.delay_ms(10);

        log::trace!("Setting I2C bypass...");
        const LATCH_INT_EN: u8 = 1 << 5;
        const INT_ANYRD_CLEAR: u8 = 1 << 4;
        const ACTL_ACTIVE_LOW: u8 = 1 << 7;
        const BYPASS_EN: u8 = 1 << 1;
        Device::write(self,
                      Register::INT_PIN_CFG,
                      LATCH_INT_EN
                      | INT_ANYRD_CLEAR
                      | ACTL_ACTIVE_LOW
                      | BYPASS_EN)?;
        delay.delay_ms(10);

        Ok(())
    }

    fn read(&mut self, reg: ak8963::Register) -> Result<u8, Self::Error> {
        let mut buffer = [0; 1];
        self.i2c.write_read(ak8963::I2C_ADDRESS,
                             &[reg.addr()],
                             &mut buffer)?;
        Ok(buffer[0])
    }

    fn write(&mut self,
             reg: ak8963::Register,
             value: u8)
             -> Result<(), Self::Error> {
        let buff: [u8; 2] = [reg.addr(), value];
        self.i2c.write(ak8963::I2C_ADDRESS, &buff)
    }

    fn read_xyz(&mut self) -> Result<GenericArray<u8, U7>, Self::Error> {
        let mut buffer: GenericArray<u8, U7> = unsafe { mem::zeroed() };
        {
            let slice: &mut [u8] = &mut buffer;
            self.i2c.write_read(ak8963::I2C_ADDRESS,
                                 &[ak8963::Register::XOUT_L.addr()],
                                 &mut slice[1..])?;
        }
        // Required to read ST2 after axes, otherwise
        // we stop sampling
        AK8963::read(self, ak8963::Register::ST2)?;
        Ok(buffer)
    }
}
