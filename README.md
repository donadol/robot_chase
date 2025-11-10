# robot_chase

ROS 2 package implementing a simple chase controller where Rick follows Morty using TF transformations and proportional control.

## Overview

This package contains a C++ node that implements a robot chase behavior. The node listens to the TF frames of two robots (Rick and Morty) and calculates velocity commands to make Rick follow Morty. The controller uses proportional control for both linear and angular velocities based on the distance and angle errors between the robots.

## Features

- **TF-based tracking**: Uses TF2 to get real-time transforms between robots
- **Proportional control**: Simple P-controller for smooth following behavior
- **Configurable parameters**: Tunable gains and velocity limits
- **Safety features**: Minimum distance threshold to prevent collision
- **Robust error handling**: Handles TF lookup failures gracefully

## Node: robot_chase

### Published Topics

- `/rick/cmd_vel` (geometry_msgs/Twist): Velocity commands for Rick

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `kp_distance` | double | 0.5 | Proportional gain for linear velocity control |
| `kp_yaw` | double | 1.0 | Proportional gain for angular velocity control |
| `max_linear_velocity` | double | 0.5 | Maximum linear velocity (m/s) |
| `max_angular_velocity` | double | 1.0 | Maximum angular velocity (rad/s) |
| `min_distance` | double | 0.3 | Minimum distance to maintain (m) |
| `control_loop_rate` | int | 10 | Control loop frequency (Hz) |

### Control Algorithm

The node implements a simple proportional controller:

1. **Get Transform**: Lookup transform from `rick/base_link` to `morty/base_link`
2. **Calculate Errors**:
   - Distance error: `sqrt(x^2 + y^2)`
   - Yaw error: `atan2(y, x)`
3. **Compute Velocities**:
   - Linear velocity: `kp_distance * error_distance` (if distance > min_distance)
   - Angular velocity: `kp_yaw * error_yaw`
4. **Apply Limits**: Clamp velocities to maximum values
5. **Publish Command**: Send Twist message to `/rick/cmd_vel`

### Expected Behavior

1. **Straight Line Chase**: Rick catches up to Morty's speed without crashing at high speed
2. **Circle Following**: Rick matches Morty's speed and follows in circular motion
3. **Stop Behavior**: Rick slows down when Morty stops, making a gentle bump and stopping without pushing

## Building the Package

```bash
# Navigate to your workspace
cd ~/ros2_ws

# Build the package
colcon build --packages-select robot_chase

# Source the workspace
source install/setup.bash
```

## Usage

### Basic Usage

1. **Launch the multi-robot simulation** (from barista_robot_description):
```bash
ros2 launch barista_robot_description barista_two_robots.launch.py
```

2. **Start the robot chase node**:
```bash
ros2 run robot_chase robot_chase
```

3. **Control Morty** using teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/morty/cmd_vel
```

### Advanced Usage with Custom Parameters

```bash
ros2 run robot_chase robot_chase --ros-args \
  -p kp_distance:=0.6 \
  -p kp_yaw:=1.2 \
  -p max_linear_velocity:=0.7 \
  -p max_angular_velocity:=1.5 \
  -p min_distance:=0.25 \
  -p control_loop_rate:=20
```

### Parameter Tuning Guide

**Increase kp_distance** (e.g., 0.7-1.0):
- Faster approach to target
- May cause oscillations if too high

**Increase kp_yaw** (e.g., 1.5-2.0):
- Faster turning response
- May cause zigzag motion if too high

**Adjust min_distance** (e.g., 0.2-0.5):
- Smaller value: Rick gets closer before stopping
- Larger value: Maintains more separation

**Increase max_linear_velocity**:
- Allows faster chasing
- Requires higher kp values for good control

**Increase control_loop_rate**:
- Smoother control
- Higher CPU usage

## Implementation Details

### TF Frames Used

- **Source frame**: `rick/base_link` (Rick's reference)
- **Target frame**: `morty/base_link` (Morty's position)

The transform is always looked up from Rick's perspective, making the control calculations straightforward in Rick's local coordinate frame.

### Control Loop

The control loop runs at a configurable rate (default 10 Hz):
1. Attempt TF lookup
2. Calculate errors if transform is available
3. Compute velocity commands
4. Publish to Rick's cmd_vel topic

Failed TF lookups are logged with throttling (max once per 5 seconds) to avoid spam.

### Safety Features

- **Minimum distance threshold**: Stops linear motion when within `min_distance`
- **Velocity clamping**: Ensures commands never exceed safe limits
- **Graceful degradation**: Continues running even if TF lookups fail temporarily

## Dependencies

- ROS 2 Humble
- rclcpp
- tf2
- tf2_ros
- geometry_msgs

## Troubleshooting

### Rick doesn't move

**Problem**: TF transform not available

**Solution**:
- Ensure both robots are spawned in Gazebo
- Check TF tree: `ros2 run tf2_tools view_frames`
- Verify frames exist: `ros2 run tf2_ros tf2_echo rick/base_link morty/base_link`

### Rick moves too fast/slow

**Problem**: Control gains need tuning

**Solution**:
- Increase `kp_distance` for faster approach
- Decrease `kp_distance` for gentler approach
- Adjust `max_linear_velocity` to set speed ceiling

### Rick oscillates or zigzags

**Problem**: Control gains too high or loop rate too low

**Solution**:
- Decrease `kp_distance` and `kp_yaw`
- Increase `control_loop_rate` (e.g., 20 Hz)
- Increase `min_distance` to reduce aggressive behavior

### Rick crashes into Morty

**Problem**: `min_distance` too small or `kp_distance` too high

**Solution**:
- Increase `min_distance` (e.g., 0.4-0.5 m)
- Decrease `kp_distance` for gentler deceleration

## Testing

### Test Scenarios

1. **Stationary Target**:
   - Don't move Morty
   - Rick should approach and stop at `min_distance`

2. **Straight Line**:
   - Move Morty forward
   - Rick should follow in a straight line

3. **Circle**:
   - Move Morty in circles
   - Rick should follow the circular path

4. **Stop and Go**:
   - Start/stop Morty randomly
   - Rick should accelerate/decelerate smoothly

### Monitoring

```bash
# Monitor velocity commands
ros2 topic echo /rick/cmd_vel

# Monitor TF transform
ros2 run tf2_ros tf2_echo rick/base_link morty/base_link

# View node logs with debug level
ros2 run robot_chase robot_chase --ros-args --log-level debug
```

## Example Session

```bash
# Terminal 1: Launch simulation
ros2 launch barista_robot_description barista_two_robots.launch.py

# Terminal 2: Start chase node
ros2 run robot_chase robot_chase

# Terminal 3: Control Morty
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/morty/cmd_vel

# Expected output in Terminal 2:
# [INFO] [robot_chase]: Robot Chase node started
# [INFO] [robot_chase]:   kp_distance: 0.50
# [INFO] [robot_chase]:   kp_yaw: 1.00
# [INFO] [robot_chase]:   max_linear_velocity: 0.50 m/s
# [INFO] [robot_chase]:   max_angular_velocity: 1.00 rad/s
# [INFO] [robot_chase]:   min_distance: 0.30 m
# [INFO] [robot_chase]: Rick will now chase Morty!
```

## Future Improvements

- Add obstacle avoidance using laser scan data
- Implement PID control instead of simple P-control
- Add dynamic reconfigure for real-time parameter tuning
- Implement predictive control for smoother following
- Add formation control for multiple followers

## License

TODO: License declaration

## Author

TODO: Author information
