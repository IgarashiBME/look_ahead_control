# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Package Overview

ROS2 (ament_python) pure-pursuit path-following controller for ground rovers. Ported from `references/qgc_look_ahead.py` (ROS1). Receives GNSS odometry and MAVLink mission waypoints, computes steering via look-ahead distance + PI cross-track error control, and publishes RC PWM (primary) and velocity commands (derived).

## Build & Run

```bash
# Build (from workspace root ~/ros2_ws)
colcon build --packages-select look_ahead_control
source install/setup.bash

# Run
ros2 run look_ahead_control look_ahead_following

# Run with parameters
ros2 run look_ahead_control look_ahead_following --ros-args \
  -p Kp:=0.1 -p Kcte:=0.01 -p look_ahead:=2.0 \
  -p throttle_scale:=0.5 -p pivot_scale:=0.5 -p driver_mix:=0.0
```

## Testing

```bash
# Linting tests (flake8, pep257, copyright)
colcon test --packages-select look_ahead_control
colcon test-result --verbose
```

No functional unit tests exist yet — only ament linting (flake8, pep257, copyright).

## Architecture

**Single node**: `LookAheadFollowing(rclpy.Node)` in `look_ahead_control/look_ahead_following.py`

**Threading model**: `rclpy.spin()` runs in a daemon thread for callback processing; the main thread runs `loop()` at 10 Hz with `create_rate()`.

**Control loop state machine** (in `loop()`):
1. Wait for all waypoints received (`mission_checker`)
2. Wait for mission start + arm state (`mission_start_checker`)
3. Run look-ahead control (coordinate transform → bearing → PI steering → publish)
4. Advance waypoints on arrival; break on mission complete

### ROS2 Interface

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/gnss_odom` | `nav_msgs/Odometry` | Vehicle pose (UTM position + quaternion). Used when `odom_source=odom` (default) |
| Sub | `/gnss` | `bme_common_msgs/GnssSolution` | GNSS solution with UTM + heading. Used when `odom_source=gnss` |
| Sub | `/mav/mission` | `std_msgs/Float64MultiArray` | `[seq, total_seq, command, lat, lon]` from mavlink_bridge |
| Sub | `/mav/modes` | `bme_common_msgs/MavModes` | mission_start, base_mode, custom_mode |
| Pub | `/rc_pwm` | `std_msgs/UInt16MultiArray` | **Primary output.** `[ch1_pwm, ch2_pwm]` — meaning depends on `driver_mix` mode |
| Pub | `/cmd_vel` | `geometry_msgs/Twist` | Secondary output. linear.x + angular.z derived from PWM values |
| Pub | `/auto_log` | `bme_common_msgs/AutoLog` | Telemetry: waypoints, CTE, PID values, steering |

### Parameters

All parameters are double type for MAVLink GCS (QGroundControl) compatibility. Declared on node, dynamically readable each loop iteration.

**Control:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `Kp` | 0.0 | Proportional gain (steering_ang in degrees) |
| `Kcte` | 0.0 | Cross-track error gain (lateral offset in meters) |
| `look_ahead` | 0.0 | Look-ahead distance (m) |
| `pivot_threshold` | 40.0 | Yaw error threshold for pivot turn (degrees) |
| `cte_threshold` | 0.1 | Lateral distance threshold to enable CTE correction (m) |
| `wp_arrival_dist` | 0.1 | Waypoint arrival distance along path axis (m) |
| `wp_skip_dist` | 0.8 | Skip consecutive waypoints closer than this (m) |

**Output:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `throttle_scale` | 0.5 | Straight-line motor output (0.0–1.0) |
| `pivot_scale` | 0.5 | Pivot turn motor output (0.0–1.0) |
| `driver_mix` | 0.0 | 0.0=differential (node mixes L/R), 1.0=passthrough (driver mixes) |
| `pwm_center` | 1500.0 | PWM center value in microseconds |
| `pwm_range` | 500.0 | Max PWM deviation from center |
| `pwm_min` | 1000.0 | PWM safety lower bound |
| `pwm_max` | 2000.0 | PWM safety upper bound |

**Other:** `odom_source` (string, default `odom`) — selects odometry topic (`odom` → `/gnss_odom`, `gnss` → `/gnss`)

### Parameter Sync from QGC

QGC writes parameters to mavlink_ros2_bridge only (via MAVLink PARAM_SET). This node syncs those parameters through two mechanisms:

1. **Startup load:** `_load_bridge_params()` reads `~/.ros/mavlink_bridge_params.yaml` (bridge's persistence file) and applies matching parameters
2. **Runtime sync:** Subscribes to `/parameter_events` and mirrors changes from `/mavlink_bridge_node` to local parameters

When adding new parameters to this node, also add them to the bridge's `paramEntries()` so QGC can set them.

### Workspace Context

```
mavlink_ros2_bridge  →  /mav/mission, /mav/modes  →  look_ahead_control  →  /rc_pwm   →  motor driver
                     ←  /auto_log                  ←                      →  /cmd_vel  →  (logging/simulation)
```

- **bme_common_msgs**: Message definitions (AutoLog, MavModes, GnssSolution) — must be built first
- **mavlink_ros2_bridge**: QGC↔ROS2 bridge over MAVLink UDP. Publishes mission/modes, subscribes to auto_log. Exposes all control/output parameters to GCS via `paramEntries()`. Parameters persist to `~/.ros/mavlink_bridge_params.yaml`.

## Dependencies

- `rclpy`, `std_msgs`, `nav_msgs`, `geometry_msgs`, `tf_transformations`, `bme_common_msgs`
- System: `python3-pyproj` (UTM coordinate conversion via `pip install pyproj` or `apt install python3-pyproj`)

## Key Constants

- `ARDUPILOT_AUTO_BASE = 217` / `ARDUPILOT_AUTO_CUSTOM = 10`: ArduPilot auto-mode identifiers matching mavlink_bridge
- `MAV_CMD_NAV_WAYPOINT = 16`: MAVLink waypoint command ID
- `FORWARD_CONST = 1` / `BACKWARD_CONST = -1`: Translation direction indicators

## References

- Original ROS1 code: `references/qgc_look_ahead.py`
- Migration spec (Japanese): `docs/initial_prompt.txt`
- Control output and parameter relationships (Japanese): `docs/control_output.md`
