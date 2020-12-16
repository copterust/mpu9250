use ak8963::{self, AK8963};

use hal::blocking::delay::DelayMs;
use hal::blocking::i2c;
use hal::blocking::spi;
use hal::digital::v2::OutputPin;

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
    fn read_many(&mut self,
                 reg: Register,
                 buffer: &mut [u8])
                 -> Result<(), Self::Error>;

    /// Write the provided value to register
    fn write(&mut self, reg: Register, val: u8) -> Result<(), Self::Error>;

    /// Write the provided data block (up to 16 bytes) to register
    fn write_many(&mut self,
                  reg: Register,
                  buffer: &[u8])
                  -> Result<(), Self::Error>;

    /// Read a single value from the register
    fn read(&mut self, reg: Register) -> Result<u8, Self::Error> {
        let buffer = &mut [0; 2];
        self.read_many(reg, buffer)?;
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

/// SPI Error
#[derive(Debug, Copy, Clone)]
pub enum SpiError<E, E2> {
    /// Bus io error
    BusError(E),
    /// NCS error
    NCSError(E2),
    /// Write many Error
    WriteManyError,
}

impl<E, E2> core::convert::From<E> for SpiError<E, E2> {
    fn from(error: E) -> Self {
        SpiError::BusError(error)
    }
}

impl<SPI, NCS, E, EO> Device for SpiDevice<SPI, NCS>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          NCS: OutputPin<Error = EO>
{
    type Error = SpiError<E, EO>;

    // Note: implementation is consistent with previous Mpu9250 private
    // methods. Using read and modify as default trait impls

    fn read_many(&mut self,
                 reg: Register,
                 buffer: &mut [u8])
                 -> Result<(), Self::Error> {
        buffer[0] = reg.read_address();
        self.ncs.set_low().map_err(SpiError::NCSError)?;
        self.spi.transfer(buffer)?;
        self.ncs.set_high().map_err(SpiError::NCSError)?;

        Ok(())
    }

    // XXX: It seems that without inline(never), when compiling
    //      with opt-level:3, ncs is out of sync with transfer.
    //      This should be probably solved in HAL or HAL impl,
    //      as device drivers should not know about such
    //      minutiæ details.
    #[inline(never)]
    fn write(&mut self, reg: Register, val: u8) -> Result<(), Self::Error> {
        self.ncs.set_low().map_err(SpiError::NCSError)?;
        self.spi.write(&[reg.write_address(), val])?;
        self.ncs.set_high().map_err(SpiError::NCSError)?;
        Ok(())
    }

    fn write_many(&mut self,
                  reg: Register,
                  buffer: &[u8])
                  -> Result<(), Self::Error> {
        if buffer.len() > 16 {
            return Err(Self::Error::WriteManyError);
        }
        self.ncs.set_low().map_err(SpiError::NCSError)?;
        self.spi.write(&[reg.write_address()])?;
        self.spi.write(buffer)?;
        self.ncs.set_high().map_err(SpiError::NCSError)?;
        Ok(())
    }
}

impl<SPI, NCS, E, EO> AK8963 for SpiDevice<SPI, NCS>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          NCS: OutputPin<Error = EO>
{
    type Error = SpiError<E, EO>;

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

    fn finalize<D: DelayMs<u8>>(&mut self,
                                delay: &mut D)
                                -> Result<(), Self::Error> {
        // set aux I2C frequency to 400 KHz (should be configurable?)
        Device::write(self, Register::I2C_MST_CTRL, 0x0d)?;

        delay.delay_ms(10);

        // configure sampling of magnetometer
        Device::write(self,
                      Register::I2C_SLV0_ADDR,
                      ak8963::I2C_ADDRESS | ak8963::R)?;
        Device::write(self,
                      Register::I2C_SLV0_REG,
                      ak8963::Register::XOUT_L.addr())?;
        Device::write(self, Register::I2C_SLV0_CTRL, 0x87)?;

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

    fn read_xyz(&mut self, buffer: &mut [u8; 7]) -> Result<(), Self::Error> {
        Device::read_many(self, Register::EXT_SENS_DATA_00, buffer)?;
        Ok(())
    }
}

/// I2C Error
#[derive(Debug, Copy, Clone)]
pub enum I2CError<E> {
    /// Bus io error
    BusError(E),
    /// Internal WriteMany Error when trying to write more than 16 bytes
    WriteManyError,
}

impl<E> core::convert::From<E> for I2CError<E> {
    fn from(error: E) -> Self {
        I2CError::BusError(error)
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
    type Error = I2CError<E>;

    fn read_many(&mut self,
                 reg: Register,
                 buffer: &mut [u8])
                 -> Result<(), Self::Error> {
        self.i2c.write_read(MPU_I2C_ADDR, &[reg as u8], &mut buffer[1..])?;
        Ok(())
    }

    fn write(&mut self, reg: Register, val: u8) -> Result<(), Self::Error> {
        let buff: [u8; 2] = [reg as u8, val];
        self.i2c.write(MPU_I2C_ADDR, &buff)?;
        Ok(())
    }

    fn write_many(&mut self,
                  reg: Register,
                  buffer: &[u8])
                  -> Result<(), Self::Error> {
        let size = if buffer.len() <= 16 {
            buffer.len()
        } else {
            return Err(Self::Error::WriteManyError);
        };
        let mut message: [u8; 17] = [0; 17];
        let message = &mut message[0..size + 1];
        message[0] = reg as u8;
        message[1..].copy_from_slice(&buffer[0..size]);
        self.i2c.write(MPU_I2C_ADDR, message)?;
        Ok(())
    }
}

impl<I2C, E> AK8963 for I2cDevice<I2C>
    where I2C: i2c::Read<Error = E>
              + i2c::Write<Error = E>
              + i2c::WriteRead<Error = E>
{
    type Error = I2CError<E>;

    fn init<D: DelayMs<u8>>(&mut self,
                            delay: &mut D)
                            -> Result<(), Self::Error> {
        Device::write(self, Register::USER_CTRL, 0)?;
        delay.delay_ms(10);

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
        self.i2c.write_read(ak8963::I2C_ADDRESS, &[reg.addr()], &mut buffer)?;
        Ok(buffer[0])
    }

    fn write(&mut self,
             reg: ak8963::Register,
             value: u8)
             -> Result<(), Self::Error> {
        let buff: [u8; 2] = [reg.addr(), value];
        self.i2c.write(ak8963::I2C_ADDRESS, &buff)?;
        Ok(())
    }

    fn read_xyz(&mut self, buffer: &mut [u8; 7]) -> Result<(), Self::Error> {
        // We need to leave the zeroth byte as a zero to conform with the
        // SPI device behaviors. We also want to read past the data bytes
        // to read register ST2. We're required to read ST2 after each
        // reading, otherwise the magnetometer blocks sampling. We can
        // achieve this in one I2C transaction
        self.i2c.write_read(ak8963::I2C_ADDRESS,
                             &[ak8963::Register::XOUT_L.addr()],
                             buffer)?;

        buffer[..].rotate_right(1);
        buffer[0] = 0; // Zero out ST2 afer rotation
        Ok(())
    }
}

/// The trait describes how to aquire 9 degrees-of-freedom
/// measurements (plug a temperature reading) from an MPU
pub trait NineDOFDevice:
    Device + AK8963<Error = <Self as Device>::Error>
{
    /// Perform a 9DOF reading
    ///
    /// The trait assumes a contiguous reading of (x, y, z) accelerometer,
    /// temperature, (x,y,z) gyroscope, and (x, y, z) magnetometer.
    /// Essentially, this is the layout of a single SPI read transaction.
    /// Any other implementors are required to meet this layout.
    fn read_9dof(&mut self,
                 reg: Register,
                 buffer: &mut [u8; 21])
                 -> Result<(), <Self as Device>::Error>;
}

impl<SPI, NCS, E, EO> NineDOFDevice for SpiDevice<SPI, NCS>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          NCS: OutputPin<Error = EO>
{
    fn read_9dof(&mut self,
                 reg: Register,
                 buffer: &mut [u8; 21])
                 -> Result<(), <Self as Device>::Error> {
        self.read_many(reg, buffer)?;
        Ok(())
    }
}

impl<I2C, E> NineDOFDevice for I2cDevice<I2C>
    where I2C: i2c::Read<Error = E>
              + i2c::Write<Error = E>
              + i2c::WriteRead<Error = E>
{
    fn read_9dof(&mut self,
                 reg: Register,
                 buffer: &mut [u8; 21])
                 -> Result<(), <Self as Device>::Error> {
        Device::read_many(self, reg, &mut buffer[0..15])?;

        let xyz_buffer = &mut [0; 7];
        AK8963::read_xyz(self, xyz_buffer)?;
        buffer[15..21].copy_from_slice(&xyz_buffer[1..7]);

        Ok(())
    }
}
