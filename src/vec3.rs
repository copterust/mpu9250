//! Vec3: Vector in 3D space
use cast::{f32, i32};
use core::ops::{Add, AddAssign, Div, DivAssign, Mul, Sub, SubAssign};

/// Vector in 3D space
#[derive(Clone, Copy, Debug)]
pub struct Vec3<T> {
    /// X component
    pub x: T,
    /// Y component
    pub y: T,
    /// Z component
    pub z: T,
}

impl<T: Default> Default for Vec3<T> {
    fn default() -> Self {
        Vec3 { x: T::default(),
               y: T::default(),
               z: T::default(), }
    }
}

impl<T> Add for Vec3<T> where T: Add<T, Output = T>
{
    type Output = Vec3<T>;

    fn add(self, rhs: Vec3<T>) -> Vec3<T> {
        Vec3 { x: self.x + rhs.x,
               y: self.y + rhs.y,
               z: self.z + rhs.z, }
    }
}

impl<T> AddAssign for Vec3<T> where T: Add<T, Output = T> + Copy
{
    fn add_assign(&mut self, rhs: Vec3<T>) {
        *self = Vec3 { x: self.x + rhs.x,
                       y: self.y + rhs.y,
                       z: self.z + rhs.z, };
    }
}

impl<T> SubAssign for Vec3<T> where T: Sub<T, Output = T> + Copy
{
    fn sub_assign(&mut self, rhs: Vec3<T>) {
        *self = Vec3 { x: self.x - rhs.x,
                       y: self.y - rhs.y,
                       z: self.z - rhs.z, };
    }
}

impl<T> DivAssign<T> for Vec3<T> where T: Div<T, Output = T> + Copy
{
    fn div_assign(&mut self, rhs: T) {
        *self = Vec3 { x: self.x / rhs,
                       y: self.y / rhs,
                       z: self.z / rhs, };
    }
}

/// Scale
pub trait Scale<RHS = Self> {
    /// Scale vector
    fn scale(self, rhs: RHS) -> Self;
}

impl<T> Scale for Vec3<T> where T: Mul<T, Output = T>
{
    fn scale(self, rhs: Vec3<T>) -> Vec3<T> {
        Vec3 { x: self.x * rhs.x,
               y: self.y * rhs.y,
               z: self.z * rhs.z, }
    }
}

impl<T> Scale<T> for Vec3<T> where T: Mul<T, Output = T> + Copy
{
    fn scale(self, rhs: T) -> Vec3<T> {
        Vec3 { x: self.x * rhs,
               y: self.y * rhs,
               z: self.z * rhs, }
    }
}

impl Vec3<i16> {
    /// Converts Vec<i16> to Vec<f32>
    pub fn f32(self) -> Vec3<f32> {
        Vec3 { x: f32(self.x),
               y: f32(self.y),
               z: f32(self.z), }
    }

    /// Converts Vec<i16> to Vec<i32>
    pub fn i32(self) -> Vec3<i32> {
        Vec3 { x: i32(self.x),
               y: i32(self.y),
               z: i32(self.z), }
    }
}
