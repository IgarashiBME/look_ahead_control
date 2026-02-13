# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Package Overview

ROS2 (ament_python) pure-pursuit path-following controller for ground rovers. Ported from `references/qgc_look_ahead.py` (ROS1). Receives GNSS odometry and MAVLink mission waypoints, computes steering via look-ahead distance + PI cross-track error control, and publishes velocity commands.

## Build & Run

```bash
# Build (from workspace root ~/ros2_ws)
colcon build --packages-select look_ahead_control
source install/setup.bash

# Run
ros2 run look_ahead_control look_ahead_following

# Run with parameters
ros2 run look_ahead_control look_ahead_following --ros-args -p Kp:=0.1 -p Ki:=0.01 -p look_ahead:=2.0
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
| Sub | `/gnss_odom` | `nav_msgs/Odometry` | Vehicle pose (UTM position + quaternion) |
| Sub | `/mav/mission` | `std_msgs/Float64MultiArray` | `[seq, total_seq, command, lat, lon]` from mavlink_bridge |
| Sub | `/mav/modes` | `bme_common_msgs/MavModes` | mission_start, base_mode, custom_mode |
| Pub | `/cmd_vel` | `geometry_msgs/Twist` | linear.x + angular.z velocity command |
| Pub | `/auto_log` | `bme_common_msgs/AutoLog` | Telemetry: waypoints, CTE, PID values, steering |

**Parameters** (declared on node, dynamically readable each loop): `Kp`, `Ki`, `look_ahead`

### Workspace Context

```
mavlink_ros2_bridge  →  /mav/mission, /mav/modes  →  look_ahead_control  →  /cmd_vel  →  motor controller
                     ←  /auto_log                  ←
```

- **bme_common_msgs**: Message definitions (AutoLog, MavModes) — must be built first
- **mavlink_ros2_bridge**: QGC↔ROS2 bridge over MAVLink UDP. Publishes mission/modes, subscribes to auto_log. Note: bridge currently expects `Float64MultiArray` on `/auto_log` and needs updating to use `AutoLog`.

## Dependencies

- `rclpy`, `std_msgs`, `nav_msgs`, `geometry_msgs`, `tf_transformations`, `bme_common_msgs`
- System: `python3-pyproj` (UTM coordinate conversion via `pip install pyproj` or `apt install python3-pyproj`)

## Key Constants

- `ARDUPILOT_AUTO_BASE = 217` / `ARDUPILOT_AUTO_CUSTOM = 10`: ArduPilot auto-mode identifiers matching mavlink_bridge
- `MAV_CMD_NAV_WAYPOINT = 16`: MAVLink waypoint command ID
- `YAW_TOLERANCE = 40.0°`: Threshold for pivot turn vs. PID steering
- `SPACING = 0.8m`: Minimum distance between consecutive waypoints (skip closer ones)

## References

- Original ROS1 code: `references/qgc_look_ahead.py`
- Migration spec (Japanese): `docs/initial_prompt.txt`
