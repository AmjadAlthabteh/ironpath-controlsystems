
Rust Autonomous Robot Telemetry & Control Simulator

IronPath is a command-line Rust project that simulates a small autonomous ground robot. The robot receives virtual sensor data, follows waypoints, avoids static circular obstacles, tracks battery usage, and logs performance metrics such as latency, speed, distance traveled, and control-loop frequency.

## What the project includes

- Robot position, velocity, heading, battery state, active waypoint, and operational state
- Waypoint navigation with completion detection
- PID steering and speed control
- Simulated distance, battery, and position sensors with deterministic noise
- Static circular obstacle detection and avoidance
- Multithreaded sensor processing
- Thread-safe communication using `std::sync::mpsc` channels
- CSV telemetry logging to `telemetry.csv`
- Average, p95, and p99 control-loop latency metrics
- Unit and integration tests
- Custom recoverable error type implementing `Display` and `std::error::Error`

## Architecture

```text
ironpath/
|-- Cargo.toml
|-- README.md
|-- src/
|   |-- lib.rs
|   |-- main.rs
|   |-- robot.rs
|   |-- controller.rs
|   |-- sensors.rs
|   |-- navigation.rs
|   |-- telemetry.rs
|   `-- math.rs
`-- tests/
    `-- simulation_tests.rs
```

The main thread runs the 100 Hz robot control loop. A sensor thread owns the virtual sensors and sends readings through a channel. A telemetry thread owns CSV writing, which keeps disk I/O out of the control loop. This approach uses Rust ownership to keep data movement explicit and avoids shared mutable state between threads.

## Build and run

Install stable Rust from <https://rustup.rs>, then run:

```bash
cargo build
cargo run
cargo test
```

Running the simulator creates `telemetry.csv` in the project root.

## Telemetry CSV

The generated CSV contains:

- `timestamp`
- `position_x`
- `position_y`
- `obstacle_distance`
- `sensor_age_us`
- `speed`
- `heading`
- `battery`
- `state`
- `active_waypoint`
- `loop_latency_us`

## Example output

```text
IronPath simulation complete
Telemetry records: 1042
Total simulation time: 10.52s
Distance traveled: 8.06m
Average speed: 0.77m/s
Average control-loop latency: 142us
p95 latency: 310us
p99 latency: 540us
Final battery level: 70.91%
Number of obstacles avoided: 3
```

## Rust concepts demonstrated

- Ownership: the sensor thread owns sensor instances, while the telemetry thread owns the CSV writer.
- Borrowing: sensor reads borrow immutable robot snapshots only for the duration of `read()`.
- Traits: all sensors implement the generic `Sensor<T>` trait.
- Channels: snapshots, sensor packets, and telemetry records are transferred with `std::sync::mpsc`.
- Thread safety: thread communication uses ownership transfer instead of shared mutable memory.
- Error handling: `SimulationError` represents recoverable I/O, configuration, and channel failures.
- Data-race prevention: Rust's type system rejects unsafe aliasing of mutable state at compile time.
