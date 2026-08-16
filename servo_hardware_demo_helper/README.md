# Attitude Publisher

ROS2 Python node that subscribes to `geometry_msgs/PoseStamped` messages and publishes roll and pitch angles.

## Overview

This package provides a simple node that:
- Subscribes to a `PoseStamped` message on the `pose` topic
- Extracts the orientation quaternion from the pose
- Converts the quaternion to Euler angles (roll, pitch, yaw)
- Publishes roll and pitch angles as a `std_msgs/Float64MultiArray` on the `attitude` topic

## Topics

### Subscribed Topics

- `pose` (`geometry_msgs/PoseStamped`): Input pose with orientation as quaternion

### Published Topics

- `attitude` (`std_msgs/Float64MultiArray`): Output array containing [roll, pitch] in radians

## Usage

Run the node:

```bash
ros2 run servo_hardware_demo_helper attitude_publisher_node
```

Remap topics if needed:

```bash
ros2 run servo_hardware_demo_helper attitude_publisher_node --ros-args \
  -r pose:=/my_pose_topic \
  -r attitude:=/my_attitude_topic
```

## Building

Build with colcon:

```bash
colcon build --packages-select servo_hardware_demo_helper
```
