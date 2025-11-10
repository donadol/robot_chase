# robot_chase

ROS 2 package implementing a chase controller where Rick follows Morty using TF transformations and proportional control.

## Features

- TF-based tracking with P-controller for smooth following
- Hardcoded control parameters (modify [robot_chase.cpp:109-113](src/robot_chase.cpp#L109-L113) to tune)
- Safety features: minimum distance threshold (1.1x robot diameter), velocity clamping, chase completion detection
- Robust error handling with throttled logging

## Control Algorithm

1. Lookup transform from `rick/base_link` to `morty/base_link`
2. Calculate errors: distance (`std::hypot(x, y)`) and yaw (`std::atan2(y, x)`)
3. If distance < `min_distance`, stop and return
4. Compute velocities: `linear = kp_distance * error_distance`, `angular = kp_yaw * error_yaw`
5. Clamp velocities and publish to `/rick/cmd_vel`

**Control loop**: 10 Hz (100ms period)

## Parameters

All hardcoded as const values:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `kp_distance_` | 0.5 | Linear velocity gain |
| `kp_yaw_` | 1.0 | Angular velocity gain |
| `max_linear_velocity_` | 0.5 m/s | Maximum forward speed |
| `max_angular_velocity_` | 1.0 rad/s | Maximum turning speed |
| `min_distance_` | 0.356 * 1.1 m | Safety distance (1.1x robot diameter) |

## Usage

```bash
# Build
colcon build --packages-select robot_chase
source install/setup.bash

# Terminal 1: Launch simulation
ros2 launch barista_robot_description barista_two_robots.launch.py

# Terminal 2: Start chase node
ros2 run robot_chase robot_chase

# Terminal 3: Control Morty
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/morty/cmd_vel
```

## Tuning

Modify values in [robot_chase.cpp:109-113](src/robot_chase.cpp#L109-L113) and rebuild:

**kp_distance** (line 109):
- Higher (0.7-1.0): Faster approach, may oscillate
- Lower (0.3-0.4): Gentler approach

**kp_yaw** (line 110):
- Higher (1.5-2.0): Faster turning, may zigzag
- Lower (0.5-0.8): Smoother turns

**min_distance** (line 113):
- Larger multiplier (1.5-2.0): More separation
- Smaller (1.0): Closer approach

**Timer period** (line 28):
- 50ms: Smoother control, higher CPU
- 200ms: Less smooth, lower CPU

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Rick doesn't move | Check both robots spawned; verify TF: `ros2 run tf2_ros tf2_echo rick/base_link morty/base_link` |
| Too fast/slow | Adjust `kp_distance_` (line 109) and rebuild |
| Oscillates/zigzags | Lower `kp_distance_` and `kp_yaw_` (lines 109-110) |
| Crashes into Morty | Increase `min_distance_` multiplier (line 113) or lower `kp_distance_` |

## Testing

Monitor with:
```bash
ros2 topic echo /rick/cmd_vel
ros2 run tf2_ros tf2_echo rick/base_link morty/base_link
ros2 run robot_chase robot_chase --ros-args --log-level debug
```

Expected behaviors:
- **Stationary**: Approach and stop at min_distance
- **Straight line**: Follow without crashing
- **Circle**: Match circular motion
- **Stop/go**: Smooth acceleration/deceleration

## Dependencies

- ROS 2 Humble
- rclcpp, tf2, tf2_ros, geometry_msgs

## Implementation Notes

**Key design decisions:**
- Hardcoded parameters for simplicity and optimization
- `std::atan2()` returns [-π, π], no normalization needed
- `std::hypot()` for numerical accuracy
- Bidirectional clamping handles negative angular velocities
- Early return on completion avoids unnecessary computation
