use std::ops::{Add, Mul, Sub};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector2 {
    pub x: f64,
    pub y: f64,
}

impl Vector2 {
    pub const ZERO: Vector2 = Vector2 { x: 0.0, y: 0.0 };

    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn magnitude(self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    pub fn distance(self, other: Vector2) -> f64 {
        (self - other).magnitude()
    }

    pub fn normalize(self) -> Self {
        let mag = self.magnitude();
        if mag <= f64::EPSILON {
            Self::ZERO
        } else {
            self * (1.0 / mag)
        }
    }

    pub fn dot(self, other: Vector2) -> f64 {
        self.x * other.x + self.y * other.y
    }
}

impl Add for Vector2 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for Vector2 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Mul<f64> for Vector2 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::Vector2;

    #[test]
    fn normalization_produces_unit_vector() {
        let normalized = Vector2::new(3.0, 4.0).normalize();
        assert!((normalized.magnitude() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn distance_uses_euclidean_length() {
        let a = Vector2::new(1.0, 2.0);
        let b = Vector2::new(4.0, 6.0);
        assert!((a.distance(b) - 5.0).abs() < 1e-10);
    }
}
