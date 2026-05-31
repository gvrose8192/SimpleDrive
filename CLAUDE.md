# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS 2 package (`simple_drive`) that implements a finite state machine for robot movement with LIDAR-based obstacle avoidance. The robot executes a sequence: **Forward** → **Spin 180°** → **Forward (backward)** → **Spin 180° to face original direction**.

### Architecture

The main node (`simple_drive/simple_drive_node.py`) contains:
- A ROS 2 `SimpleDriveNode` class extending `Node` from rclpy
- Finite state machine with states: WAIT, FORWARD_1, TURN_SPIN, FORWARD_2, FINISH_SPIN, DONE, OBSTACLE_AVOID
- LaserScan parsing for obstacle detection using direct array access on message data
- Services: `/simple_drive/start`, `/simple_drive/reset`

### State Machine Flow (from README)

1. **WAIT**: Idle state, waits for start trigger
2. **FORWARD_1**: Move forward `move_distance` meters at `linear_speed`
3. **TURN_SPIN**: Spin 180° to reverse direction at `-angular_speed`
4. **FORWARD_2**: Move forward (backward relative to original heading)
5. **FINISH_SPIN**: Spin another 180° to face original direction
6. **DONE**: Completed sequence
7. **OBSTACLE_AVOID**: Emergency state when obstacle detected within `stop_distance`; performs up to `max_spin_attempts` spin checks at ~45° each (~0.6s at `spin_speed`) before giving up

### Key Architecture Decisions

**LaserScan Parsing**: The code uses direct array access on `ranges` from the LaserScan message ([simple_drive_node.py:100-115](file:///home/ubuntu/ros2_ws/src/simple_drive/simple_drive/simple_drive_node.py#L100-L115)). This is simpler than point cloud parsing but requires using angle_min and angle_increment for spatial awareness.

**No Calibration Needed**: LIDAR is forward-facing on the front bumper; Z/distances are naturally relative to front bumper (no offset required).

**Polling Frequency**: Timer runs at 500ms intervals (`timer_period = 0.5`) - slower CPU usage since only need to check obstacle when moving.

## Build Commands

```bash
# Build the package
colcon build --packages-select simple_drive

# Source the installation
source install/setup.bash

# Run main node
ros2 run simple_drive simple_drive_node

# Run reset service node (separate terminal)
ros2 run simple_drive reset_service_node
```

## Testing Commands

```bash
# Run all tests
colcon test --packages-select simple_drive

# Run a single test
colcon test --packages-select simple_drive --event-handlers console_direct+ --args --test-name <test_name>

# Check code style (if packages installed)
flake8 simple_drive/
```

## Service API

### Start Movement
```bash
ros2 service call /simple_drive/start std_srvs/srv/Trigger
```

### Reset to WAIT State
```bash
ros2 service call /simple_drive/reset std_srvs/srv/Trigger
```

### Optional: Reset via External Service Node
```bash
ros2 service call reset std_srvs/srv/Trigger
# Triggered from reset_service_node
```

## Parameters (with defaults)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `linear_speed` | 0.2 | Forward velocity magnitude (m/s) |
| `angular_speed` | 0.5 | Spin velocity for turns (rad/s) |
| `move_distance` | 1.25 | Distance to travel in forward phases (m) |
| `stop_distance` | 0.5 | Obstacle detection threshold (m, front-facing LIDAR) |
| `spin_speed` | 1.0 | Spin speed when avoiding obstacles (rad/s) |
| `max_spin_attempts` | 4 | Number of spin attempts before giving up |

Set parameters at runtime:
```bash
ros2 param set <node_name> <param_name> <value>
```

## LIDAR Topic

- **Topic**: `/scan`
- **Message Type**: `sensor_msgs/LaserScan`
- **Required Fields**: `ranges`, `angle_min`, `angle_increment`, `range_min`, `range_max`
- **Expected FOV**: ~90° (-45° to 45° from center)

### LIDAR Parsing Details

The obstacle detection (`find_closest_obstacle`) method:
1. Uses pre-calculated `ranges` array with angles derived from `angle_min + i * angle_increment`
2. Filters valid readings using `range_max * 0.95` threshold (noise filter)
3. Returns closest obstacle distance in forward hemisphere
4. No calibration offset needed (front-facing sensor)

## Known Behaviors & Edge Cases

### Spin Attempt Counter Reset
- Entering OBSTACLE_AVOID from forward states (FORWARD_1, FORWARD_2, FINISH_SPIN)
- Starting first spin check in OBSTACLE_AVOID state
- Calling reset service

### State Machine Logic

**Obstacle Avoidance Flow**:
1. When obstacle detected within `stop_distance`, transition to `DETERMINING_OBSTACLE` state
2. Accumulate spin angle over time at `spin_speed`
3. After ~45° of spinning (~0.6s), check if path is clear (`obstacle_dist > stop_distance`)
4. If clear, resume movement; if not, continue spinning up to `max_spin_attempts`
5. If max attempts exceeded, transition to DONE state

**Timer Design**: Only checks obstacles when robot is moving (FORWARD_1, TURN_SPIN, FORWARD_2, FINISH_SPIN). REDUCED CPU usage compared to depth_cam_drive by using 500ms polling interval.

## Package Dependencies (package.xml)

- `rclpy`: ROS 2 Python client library
- `geometry_msgs`: For Twist messages (velocity commands)
- `sensor_msgs`: For LaserScan messages (LIDAR data)
- `std_srvs`: For Trigger service type
- `std_msgs`: Standard message types

Test dependencies: `ament_copyright`, `ament_flake8`, `ament_pep257`, `pytest`

## Setup Files

- `setup.py`: Python package definition with console script entry points
- `setup.cfg`: Python configuration (PEP8 style checks)
- `resource/simple_drive`: ROS 2 resource for package indexing
