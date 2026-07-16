use crate::math::Vector2;
use crate::navigation::ObstacleField;
use crate::robot::RobotSnapshot;
use std::time::Instant;

pub trait Sensor<T> {
    fn read(&mut self, robot: &RobotSnapshot) -> T;
}

#[derive(Debug, Clone, Copy)]
pub struct SensorPacket {
    pub timestamp: Instant,
    pub position: Vector2,
    pub battery: f64,
    pub obstacle_distance: Option<f64>,
}

pub struct DistanceSensor {
    obstacles: ObstacleField,
    rng: SimpleRng,
}

impl DistanceSensor {
    pub fn new(obstacles: ObstacleField, seed: u64) -> Self {
        Self {
            obstacles,
            rng: SimpleRng::new(seed),
        }
    }
}

impl Sensor<Option<f64>> for DistanceSensor {
    fn read(&mut self, robot: &RobotSnapshot) -> Option<f64> {
        self.obstacles
            .obstacles()
            .iter()
            .map(|obstacle| obstacle.distance_from_edge(robot.position))
            .min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .map(|distance| (distance + self.rng.noise(0.015)).max(0.0))
    }
}

pub struct BatterySensor {
    rng: SimpleRng,
}

impl BatterySensor {
    pub fn new(seed: u64) -> Self {
        Self {
            rng: SimpleRng::new(seed),
        }
    }
}

impl Sensor<f64> for BatterySensor {
    fn read(&mut self, robot: &RobotSnapshot) -> f64 {
        (robot.battery + self.rng.noise(0.05)).clamp(0.0, 100.0)
    }
}

pub struct PositionSensor {
    rng: SimpleRng,
}

impl PositionSensor {
    pub fn new(seed: u64) -> Self {
        Self {
            rng: SimpleRng::new(seed),
        }
    }
}

impl Sensor<Vector2> for PositionSensor {
    fn read(&mut self, robot: &RobotSnapshot) -> Vector2 {
        Vector2::new(
            robot.position.x + self.rng.noise(0.01),
            robot.position.y + self.rng.noise(0.01),
        )
    }
}

#[derive(Debug, Clone)]
struct SimpleRng {
    state: u64,
}

impl SimpleRng {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_f64(&mut self) -> f64 {
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let value = self.state >> 11;
        (value as f64) / ((1_u64 << 53) as f64)
    }

    fn noise(&mut self, amplitude: f64) -> f64 {
        (self.next_f64() * 2.0 - 1.0) * amplitude
    }
}
