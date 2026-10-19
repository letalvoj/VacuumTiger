# Mock Device User Guide

This guide explains how to use SangamIO's mock device driver for simulation-based development and testing.

## Overview

The mock device driver provides a complete simulation of CRL-200S hardware, enabling:

- **Algorithm development** without physical robots
- **Unit testing** with deterministic sensor data
- **CI/CD integration** for automated testing
- **Accelerated testing** with speed factor

## Quick Start

### 1. Build with Mock Feature

```bash
cd sangam-io
cargo build --release --features mock
```

### 2. Create or Use an Existing Map

Use the example map or create your own:

```bash
ls maps/
# example.yaml  example.pgm
```

### 3. Run Simulation

```bash
# Using default mock.toml
cargo run --release --features mock -- mock.toml

# Or with explicit config flag
cargo run --release --features mock -- --config mock.toml

# With debug logging
RUST_LOG=debug cargo run --release --features mock -- mock.toml
```

### 4. Connect Your Client

Connect to `localhost:5555` using the same protocol as real hardware:
- **UDP** for sensor data (automatic when TCP connects)
- **TCP** for commands

## Map Creation

### PGM + YAML Format

Maps use the ROS-standard format for maximum compatibility.

**Step 1: Create PGM Image**

Use any image editor (GIMP, Photoshop, or programmatically):

| Color | Value | Meaning |
|-------|-------|---------|
| White | 255 | Free space |
| Black | 0 | Wall/obstacle |
| Gray | 205 | Unknown |

Save as 8-bit grayscale PGM (P5 binary or P2 ASCII).

**Step 2: Create YAML Metadata**

```yaml
# maps/my_map.yaml
image: my_map.pgm           # PGM filename (relative to YAML)
resolution: 0.02            # meters per pixel
origin: [-5.0, -5.0, 0.0]   # [x, y, yaw] of bottom-left pixel
occupied_thresh: 0.65       # Pixels darker than this are occupied

# Optional masks, same dimensions as the main map:
cliff_mask: my_map_cliffs.pgm   # black where the cliff sensors trigger
lidar_mask: my_map_lidar.pgm    # what the lidar sees, see below
```

**Step 3: Calculate Origin**

The origin is the world coordinate of the bottom-left pixel:

```
origin_x = -width_pixels * resolution / 2
origin_y = -height_pixels * resolution / 2
```

For a 500x500 pixel map at 0.02 m/px:
```yaml
origin: [-5.0, -5.0, 0.0]  # Centers the map
```

### Using ROS Maps

Maps saved by ROS tools work directly:

```bash
# From ROS map_server
ros2 run nav2_map_server map_saver_cli -f my_map

# Produces my_map.pgm and my_map.yaml
# Copy to sangam-io/maps/
```

### Creating Cliff Masks

Cliff masks define areas where cliff sensors should trigger:

1. Create a PGM with same dimensions as main map
2. Paint black (0) where cliffs exist
3. Leave white (255) for safe floor
4. Reference in YAML: `cliff_mask: my_map_cliffs.pgm`

### Lidar Masks: Glass and Curtains

Without a mask the lidar sees exactly what blocks the robot. A `lidar_mask` (an extension of
the ROS format) separates the two: a cell's darkness is the chance that a beam crossing it
returns from it, so its effect depends on the resolution.

- Glass: occupied in the map, white (clear) in the mask. The robot bumps it, the lidar
  sees through it.
- A curtain: free in the map, gray in the mask. The robot pushes through, beams stop
  somewhere among its folds.

Ranges are exact (the ray walks the cells it crosses) but measured to a cell's edge: use
1 cm maps where range accuracy matters.

## Configuration Reference

### Minimal Configuration

```toml
[device]
type = "mock"
name = "Mock Robot"

[device.simulation]
map_file = "maps/example.yaml"

[network]
bind_address = "0.0.0.0:5555"
```

### Full Configuration

See `mock.toml` for all available options. Key sections:

#### Simulation Control

```toml
[device.simulation]
map_file = "maps/example.yaml"
start_x = 1.5            # Initial X position (meters)
start_y = 3.5            # Initial Y position (meters)
start_theta = 0.0        # Initial heading (radians, 0 = +X)
speed_factor = 1.0       # Time multiplier (2.0 = 2x speed)
random_seed = 42         # 0 = random, >0 = deterministic
log_level = "standard"   # "minimal", "standard", "verbose"
```

#### Robot Parameters

```toml
[device.simulation.robot]
wheel_base = 0.233       # Distance between wheels (meters)
ticks_per_meter = 4464.0 # Encoder resolution
max_linear_speed = 0.3   # Speed limit (m/s)
max_angular_speed = 1.0  # Turn rate limit (rad/s)
robot_radius = 0.17      # Collision radius (meters)
collision_mode = "stop"  # "stop", "slide", "passthrough"
response_delay = 0.08          # Dead time from command to wheels (s; v8: 0.08)
response_time_constant = 0.12  # First-order lag of the wheel speed (s; v8: 0.12)
response_gain = 0.9            # Reached / commanded speed (v8: 0.9)
```

The bumper interlock is SangamIO's (`core::drive::bumper_stops`): driving forward into a
pressed front bumper zeroes the command until the client sends another.

#### Lidar Simulation

```toml
[device.simulation.lidar]
num_rays = 360           # Points per scan
scan_rate_hz = 5.0       # Revolutions per second (v8: 6.03)
sweep_start_angle = 0.2182  # First beam (rad, robot frame); beams go clockwise
min_range = 0.15         # Minimum detection (meters)
max_range = 8.0          # Maximum detection (meters)
mounting_x = -0.110      # Lidar position in the robot frame (meters)
mounting_y = 0.0
optical_offset = 0.025   # Published range = true range - this (meters)

[device.simulation.lidar.noise]
range_stddev = 0.005     # Distance noise (meters)
range_stddev_per_m = 0.003  # ... plus this much per meter of range
angle_stddev = 0.001     # Angular noise (radians)
miss_rate = 0.01         # Invalid reading probability
quality_base = 200       # Base quality (0-255)
```

#### IMU Simulation

```toml
[device.simulation.imu]
gyro_scale = 1000.0        # Raw units per rad/s (v8: 5729.6)

[device.simulation.imu.gyro_noise]
stddev = [5.0, 5.0, 10.0]  # Per-axis noise
bias = [0.0, 0.0, 0.0]     # Constant bias
drift_rate = 0.0           # Bias drift (units/sec)
deadband = 0.0             # |raw| below this reads 0 (v8: 100)

[device.simulation.imu.accel_noise]
stddev = [10.0, 10.0, 20.0]
bias = [0.0, 0.0, 0.0]
```

#### Encoder Simulation

```toml
[device.simulation.encoder.noise]
slip_stddev = 0.002      # Wheel slip noise (0-1)
slip_bias = 0.0          # Systematic slip
quantization_noise = true # ±0.5 tick jitter
```

## Speed Factor Usage

The simulation always steps at the CRL-200S's 110 Hz in simulated time and stamps every
group with that clock. `speed_factor` only makes it run that many times as fast as the wall
clock: at 2.0, `sensor_status` arrives at 220 Hz wall time, still 110 Hz by its stamps.
Higher factors cost CPU and may not be sustainable.

`mock_bench.toml` is the mock as close to the measured v8 as we know it. Offline benches
don't run the daemon: they step a `MockRobot` (`devices::mock::robot`) as fast as they like
and read the same `sensor_status` and `lidar` groups.

## Deterministic Testing

For reproducible tests, set a fixed random seed:

```toml
[device.simulation]
random_seed = 12345  # Same seed = same sensor noise sequence
```

This ensures:
- Identical lidar noise patterns
- Same encoder slip variations
- Reproducible IMU readings

## Collision Modes

Three collision handling modes are available:

| Mode | Behavior | Use Case |
|------|----------|----------|
| `stop` | Robot stops on collision | Realistic behavior |
| `slide` | Robot slides along walls | Wall-following tests |
| `passthrough` | No collision | Debugging, path planning |

## Troubleshooting

### "Map file not found"

```
Error: Failed to read map YAML: No such file or directory
```

Check that `map_file` path is relative to where you run the command, or use absolute path.

### "Mock device not available"

```
Error: Mock device not available: rebuild with --features mock
```

The mock feature is not enabled. Rebuild:
```bash
cargo build --release --features mock
```

### High CPU Usage

If CPU usage is high:
1. Reduce `speed_factor`
2. Check `log_level` (verbose logging is expensive)
3. Ensure release build: `--release`

### No Lidar Data

Lidar must be explicitly enabled:
```python
# Send lidar enable command via TCP
cmd = ComponentControl(id="lidar", action=Enable())
```

### Client Not Receiving UDP

1. Ensure TCP connection is established first
2. Check firewall allows UDP on port 5555
3. Verify client is binding to correct port

## Integration with SLAM

The mock device produces identical sensor data format to real hardware:

```python
# Same code works for both real and simulated robot
client = SangamClient(host="localhost", port=5555)
# or
client = SangamClient(host="192.168.68.101", port=5555)
```

## Example Workflow

1. **Develop algorithm** using mock device on laptop
2. **Write unit tests** with deterministic random seed
3. **Run CI tests** with mock device (no hardware needed)
4. **Deploy to robot** - same client code works
5. **Record real data** using bag files
6. **Replay and compare** mock vs real performance
