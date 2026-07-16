#[derive(Debug, Clone)]
pub struct PidController {
    kp: f64,
    ki: f64,
    kd: f64,
    integral: f64,
    previous_error: f64,
    output_min: f64,
    output_max: f64,
}

impl PidController {
    pub fn new(kp: f64, ki: f64, kd: f64, output_min: f64, output_max: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            previous_error: 0.0,
            output_min,
            output_max,
        }
    }

    pub fn update(&mut self, setpoint: f64, measurement: f64, dt: f64) -> f64 {
        let error = setpoint - measurement;
        self.integral += error * dt;
        let derivative = if dt > f64::EPSILON {
            (error - self.previous_error) / dt
        } else {
            0.0
        };
        self.previous_error = error;

        (self.kp * error + self.ki * self.integral + self.kd * derivative)
            .clamp(self.output_min, self.output_max)
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::PidController;

    #[test]
    fn pid_output_moves_toward_setpoint() {
        let mut pid = PidController::new(2.0, 0.2, 0.1, -10.0, 10.0);
        let output = pid.update(4.0, 1.0, 0.1);
        assert!(output > 0.0);
    }
}
