use crate::math::Vector2;
use crate::robot::RobotState;
use crate::SimulationError;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::sync::mpsc::Receiver;
use std::time::Duration;

#[derive(Debug, Clone)]
pub struct TelemetryRecord {
    pub timestamp_secs: f64,
    pub position: Vector2,
    pub speed: f64,
    pub heading: f64,
    pub battery: f64,
    pub state: RobotState,
    pub active_waypoint: Option<usize>,
    pub loop_latency: Duration,
}

#[derive(Debug, Clone)]
pub struct SimulationSummary {
    pub total_time: Duration,
    pub distance_traveled: f64,
    pub average_speed: f64,
    pub average_latency: Duration,
    pub p95_latency: Duration,
    pub p99_latency: Duration,
    pub final_battery: f64,
    pub obstacles_avoided: usize,
}

pub fn telemetry_writer(
    path: &str,
    rx: Receiver<TelemetryRecord>,
) -> Result<usize, SimulationError> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);
    writeln!(
        writer,
        "timestamp,position_x,position_y,speed,heading,battery,state,active_waypoint,loop_latency_us"
    )?;

    let mut count = 0;
    for record in rx {
        writeln!(
            writer,
            "{:.4},{:.4},{:.4},{:.4},{:.4},{:.2},{},{},{}",
            record.timestamp_secs,
            record.position.x,
            record.position.y,
            record.speed,
            record.heading,
            record.battery,
            record.state.as_str(),
            record
                .active_waypoint
                .map(|idx| idx.to_string())
                .unwrap_or_else(|| "none".to_string()),
            record.loop_latency.as_micros()
        )?;
        count += 1;
    }

    writer.flush()?;
    Ok(count)
}

pub fn average_duration(samples: &[Duration]) -> Duration {
    if samples.is_empty() {
        return Duration::ZERO;
    }

    let total_nanos: u128 = samples.iter().map(Duration::as_nanos).sum();
    Duration::from_nanos((total_nanos / samples.len() as u128) as u64)
}

pub fn percentile_duration(samples: &[Duration], percentile: f64) -> Duration {
    if samples.is_empty() {
        return Duration::ZERO;
    }

    let mut sorted = samples.to_vec();
    sorted.sort_unstable();
    let rank = ((sorted.len() as f64 - 1.0) * percentile).ceil() as usize;
    sorted[rank.min(sorted.len() - 1)]
}
