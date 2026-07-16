use ironpath::controller::PidController;
use ironpath::math::Vector2;
use ironpath::navigation::{Obstacle, ObstacleField, WaypointNavigator};
use ironpath::robot::{Robot, RobotState};

#[test]
fn vector_normalization_returns_unit_length() {
    let normalized = Vector2::new(10.0, 0.0).normalize();
    assert!((normalized.magnitude() - 1.0).abs() < 1e-10);
}

#[test]
fn vector_distance_is_correct() {
    let distance = Vector2::new(-1.0, -1.0).distance(Vector2::new(2.0, 3.0));
    assert!((distance - 5.0).abs() < 1e-10);
}

#[test]
fn pid_output_is_clamped() {
    let mut pid = PidController::new(100.0, 0.0, 0.0, -1.0, 1.0);
    assert_eq!(pid.update(10.0, 0.0, 0.1), 1.0);
}

#[test]
fn waypoint_completion_reports_finished() {
    let mut navigator = WaypointNavigator::new(vec![Vector2::new(0.2, 0.0)], 0.25);
    assert!(navigator.update(Vector2::ZERO));
    assert!(navigator.is_finished());
}

#[test]
fn low_battery_state_is_set_during_motion() {
    let mut robot = Robot::new(Vector2::ZERO, 1.0);
    robot.battery = 1.0;
    robot.apply_motion(Vector2::new(1.0, 0.0), 1.0, 0.0, 0.1);
    assert_eq!(robot.state, RobotState::LowBattery);
}

#[test]
fn obstacle_detection_returns_avoidance_direction() {
    let field = ObstacleField::new(vec![Obstacle::new(Vector2::new(1.0, 0.0), 0.4)]);
    let direction = field.avoidance_direction(Vector2::new(0.7, 0.0), 0.5);
    assert!(direction.is_some());
}
