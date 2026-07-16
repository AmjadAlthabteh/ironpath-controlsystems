use crate::math::Vector2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobotState {
    Idle,
    Navigating,
    AvoidingObstacle,
    LowBattery,
    Finished,
}

impl RobotState {
    pub fn as_str(self) -> &'static str {
        match self {
            RobotState::Idle => "Idle",
            RobotState::Navigating => "Navigating",
            RobotState::AvoidingObstacle => "AvoidingObstacle",
            RobotState::LowBattery => "LowBattery",
            RobotState::Finished => "Finished",
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RobotSnapshot {
    pub position: Vector2,
    pub velocity: Vector2,
    pub heading: f64,
    pub battery: f64,
}

#[derive(Debug, Clone)]
pub struct Robot {
    pub position: Vector2,
    pub velocity: Vector2,
    pub heading: f64,
    pub battery: f64,
    pub max_speed: f64,
    pub current_waypoint: Option<usize>,
    pub state: RobotState,
}

impl Robot {
    pub fn new(position: Vector2, max_speed: f64) -> Self {
        Self {
            position,
            velocity: Vector2::ZERO,
            heading: 0.0,
            battery: 100.0,
            max_speed,
            current_waypoint: None,
            state: RobotState::Idle,
        }
    }

    pub fn snapshot(&self) -> RobotSnapshot {
        RobotSnapshot {
            position: self.position,
            velocity: self.velocity,
            heading: self.heading,
            battery: self.battery,
        }
    }

    pub fn apply_motion(&mut self, desired_direction: Vector2, speed: f64, steering: f64, dt: f64) {
        let clamped_speed = speed.clamp(0.0, self.max_speed);
        self.heading = wrap_angle(self.heading + steering * dt);

        let facing = if desired_direction.magnitude() > f64::EPSILON {
            desired_direction.normalize()
        } else {
            Vector2::new(self.heading.cos(), self.heading.sin())
        };

        self.velocity = facing * clamped_speed;
        self.position = self.position + self.velocity * dt;
        self.battery = (self.battery - (0.018 + clamped_speed * 0.008) * dt * 100.0).max(0.0);

        if self.battery <= 15.0 && self.state != RobotState::Finished {
            self.state = RobotState::LowBattery;
        }
    }

    pub fn distance_traveled_increment(previous: Vector2, current: Vector2) -> f64 {
        previous.distance(current)
    }
}

fn wrap_angle(mut angle: f64) -> f64 {
    while angle > std::f64::consts::PI {
        angle -= std::f64::consts::TAU;
    }
    while angle < -std::f64::consts::PI {
        angle += std::f64::consts::TAU;
    }
    angle
}

#[cfg(test)]
mod tests {
    use super::{Robot, RobotState};
    use crate::math::Vector2;

    #[test]
    fn robot_enters_low_battery_state() {
        let mut robot = Robot::new(Vector2::ZERO, 1.0);
        robot.battery = 15.0;
        robot.apply_motion(Vector2::new(1.0, 0.0), 1.0, 0.0, 0.1);
        assert_eq!(robot.state, RobotState::LowBattery);
    }
}
