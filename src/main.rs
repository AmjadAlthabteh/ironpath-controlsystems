use ironpath::controller::PidController;
use ironpath::math::Vector2;
use ironpath::navigation::{Obstacle, ObstacleField, WaypointNavigator};
use ironpath::robot::{Robot, RobotState, RobotSnapshot};
use ironpath::sensors::{
    BatterySensor, DistanceSensor, PositionSensor, Sensor, SensorPacket,
};
use ironpath::telemetry::{
    average_duration, percentile_duration, telemetry_writer, SimulationSummary, TelemetryRecord,
};
use ironpath::SimulationError;
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
struct SimulationConfig {
    control_hz: u64,
    max_steps: usize,
    safety_distance: f64,
    telemetry_path: &'static str,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            control_hz: 100,
            max_steps: 3_000,
            safety_distance: 0.85,
            telemetry_path: "telemetry.csv",
        }
    }
}

impl SimulationConfig {
    fn dt(&self) -> f64 {
        1.0 / self.control_hz as f64
    }

    fn target_period(&self) -> Duration {
        Duration::from_micros(1_000_000 / self.control_hz)
    }
}

fn main() {
    if let Err(err) = run() {
        eprintln!("IronPath failed: {err}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), SimulationError> {
    let config = SimulationConfig::default();
    let obstacles = ObstacleField::new(vec![
        Obstacle::new(Vector2::new(2.2, 0.7), 0.35),
        Obstacle::new(Vector2::new(4.3, 1.8), 0.45),
        Obstacle::new(Vector2::new(6.0, 0.4), 0.30),
    ]);

    let mut robot = Robot::new(Vector2::ZERO, 1.4);
    let mut navigator = WaypointNavigator::new(
        vec![
            Vector2::new(2.0, 0.0),
            Vector2::new(4.0, 1.2),
            Vector2::new(6.5, 1.0),
            Vector2::new(8.0, 0.0),
        ],
        0.18,
    );

    let (snapshot_tx, snapshot_rx) = mpsc::channel::<RobotSnapshot>();
    let (sensor_tx, sensor_rx) = mpsc::channel::<SensorPacket>();
    let sensor_obstacles = obstacles.clone();

    let sensor_thread = thread::spawn(move || {
        let mut distance_sensor = DistanceSensor::new(sensor_obstacles, 0xA11C_E55);
        let mut battery_sensor = BatterySensor::new(0xBA77_E9);
        let mut position_sensor = PositionSensor::new(0x9051_710);

        // The sensor thread owns each sensor and borrows each immutable snapshot only
        // while read() executes. Rust prevents those borrows from outliving the data.
        for snapshot in snapshot_rx {
            let packet = SensorPacket {
                timestamp: Instant::now(),
                position: position_sensor.read(&snapshot),
                battery: battery_sensor.read(&snapshot),
                obstacle_distance: distance_sensor.read(&snapshot),
            };
            if sensor_tx.send(packet).is_err() {
                break;
            }
        }
    });

    let (telemetry_tx, telemetry_rx) = mpsc::channel::<TelemetryRecord>();
    let telemetry_thread =
        thread::spawn(move || telemetry_writer(config.telemetry_path, telemetry_rx));

    let mut steering_pid = PidController::new(3.2, 0.0, 0.4, -2.5, 2.5);
    let mut speed_pid = PidController::new(1.8, 0.05, 0.1, 0.0, robot.max_speed);
    let mut latencies = Vec::with_capacity(config.max_steps);
    let mut distance_traveled = 0.0;
    let mut obstacles_avoided = 0;
    let mut was_avoiding = false;
    let started = Instant::now();
    let target_period = config.target_period();

    robot.state = RobotState::Navigating;

    // Channels transfer owned snapshots and telemetry records across threads.
    // The standard library mpsc types ensure there is no shared mutable state to race.
    for _step in 0..config.max_steps {
        let loop_started = Instant::now();
        snapshot_tx.send(robot.snapshot())?;

        let sensor_packet = sensor_rx
            .recv_timeout(Duration::from_millis(2))
            .map_err(|_| SimulationError::ChannelClosed("sensor packet"))?;

        let previous_position = robot.position;
        let finished = navigator.update(robot.position);
        robot.current_waypoint = navigator.active_index();

        if finished {
            robot.state = RobotState::Finished;
        } else if robot.battery <= 15.0 {
            robot.state = RobotState::LowBattery;
        } else if let Some(avoidance) =
            obstacles.avoidance_direction(robot.position, config.safety_distance)
        {
            robot.state = RobotState::AvoidingObstacle;
            if !was_avoiding {
                obstacles_avoided += 1;
            }
            was_avoiding = true;
            step_robot(
                &mut robot,
                &mut steering_pid,
                &mut speed_pid,
                avoidance,
                0.75,
                config.dt(),
            );
        } else if let Some(direction) = navigator.direction_to_active(robot.position) {
            robot.state = RobotState::Navigating;
            was_avoiding = false;
            let target_speed = robot.max_speed;
            step_robot(
                &mut robot,
                &mut steering_pid,
                &mut speed_pid,
                direction,
                target_speed,
                config.dt(),
            );
        }

        distance_traveled += Robot::distance_traveled_increment(previous_position, robot.position);

        let latency = loop_started.elapsed();
        latencies.push(latency);
        telemetry_tx.send(TelemetryRecord {
            timestamp_secs: started.elapsed().as_secs_f64(),
            position: sensor_packet.position,
            obstacle_distance: sensor_packet.obstacle_distance,
            sensor_age: sensor_packet.timestamp.elapsed(),
            speed: robot.velocity.magnitude(),
            heading: robot.heading,
            battery: sensor_packet.battery,
            state: robot.state,
            active_waypoint: robot.current_waypoint,
            loop_latency: latency,
        })?;

        if robot.state == RobotState::Finished || robot.state == RobotState::LowBattery {
            break;
        }

        let elapsed = loop_started.elapsed();
        if elapsed < target_period {
            thread::sleep(target_period - elapsed);
        }
    }

    drop(snapshot_tx);
    drop(telemetry_tx);

    sensor_thread
        .join()
        .map_err(|_| SimulationError::ChannelClosed("sensor thread"))?;
    let records_written = telemetry_thread
        .join()
        .map_err(|_| SimulationError::ChannelClosed("telemetry thread"))??;

    let summary = build_summary(
        started.elapsed(),
        distance_traveled,
        &latencies,
        robot.battery,
        obstacles_avoided,
    );

    print_summary(&summary, records_written);
    Ok(())
}

fn step_robot(
    robot: &mut Robot,
    steering_pid: &mut PidController,
    speed_pid: &mut PidController,
    direction: Vector2,
    target_speed: f64,
    dt: f64,
) {
    let desired_heading = direction.y.atan2(direction.x);
    let steering = steering_pid.update(desired_heading, robot.heading, dt);
    let speed = speed_pid.update(target_speed, robot.velocity.magnitude(), dt);
    robot.apply_motion(direction, speed, steering, dt);
}

fn build_summary(
    total_time: Duration,
    distance_traveled: f64,
    latencies: &[Duration],
    final_battery: f64,
    obstacles_avoided: usize,
) -> SimulationSummary {
    SimulationSummary {
        total_time,
        distance_traveled,
        average_speed: if total_time.as_secs_f64() > 0.0 {
            distance_traveled / total_time.as_secs_f64()
        } else {
            0.0
        },
        average_latency: average_duration(latencies),
        p95_latency: percentile_duration(latencies, 0.95),
        p99_latency: percentile_duration(latencies, 0.99),
        final_battery,
        obstacles_avoided,
    }
}

fn print_summary(summary: &SimulationSummary, records_written: usize) {
    println!("IronPath simulation complete");
    println!("Telemetry records: {records_written}");
    println!("Total simulation time: {:.2}s", summary.total_time.as_secs_f64());
    println!("Distance traveled: {:.2}m", summary.distance_traveled);
    println!("Average speed: {:.2}m/s", summary.average_speed);
    println!(
        "Average control-loop latency: {}us",
        summary.average_latency.as_micros()
    );
    println!("p95 latency: {}us", summary.p95_latency.as_micros());
    println!("p99 latency: {}us", summary.p99_latency.as_micros());
    println!("Final battery level: {:.2}%", summary.final_battery);
    println!("Number of obstacles avoided: {}", summary.obstacles_avoided);
}
