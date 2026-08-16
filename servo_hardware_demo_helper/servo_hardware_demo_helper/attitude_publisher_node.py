#!/usr/bin/env python3
# Copyright 2026 Gerardo Puga
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import math


class AttitudePublisher(Node):
    """ROS2 node that publishes roll and pitch angles from pose data.

    Subscribes to PoseStamped messages and publishes roll and pitch angles
    as Float64MultiArray.
    """

    def __init__(self):
        """Initialize the attitude publisher node."""
        super().__init__('servo_hardware_demo_helper')

        # Declare parameters
        self.declare_parameter('roll_offset', math.pi / 2)
        self.declare_parameter('pitch_offset', math.pi / 2)

        # Create subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            '/pose',
            self.pose_callback,
            10)

        # Create publisher
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'commands',
            10)

        self.get_logger().info('Attitude publisher node started')

    def quaternion_to_roll_pitch(self, x, y, z, w):
        """
        Calculate roll and pitch from quaternion for a 2-servo gimbal system.

        This method extracts intrinsic Euler angles (roll-then-pitch) from a quaternion
        such that the platform's Z-axis matches the input orientation's Z-axis,
        independent of yaw.

        Physical setup:
        1. First servo applies roll (rotation around X-axis)
        2. Second servo applies pitch (rotation around Y-axis in the rolled frame)

        The calculation:
        1. Extracts the Z-axis direction from the input quaternion
        2. Solves for the intrinsic roll-then-pitch angles that produce the same Z-axis

        After roll r around X, then pitch p around the rotated Y-axis:
        Z-axis becomes: (sin(p), -sin(r)*cos(p), cos(r)*cos(p))

        Args:
            x, y, z, w: Quaternion components

        Returns:
            tuple: (roll, pitch) in radians
        """
        # Extract Z-axis direction from the quaternion
        z_x = 2*(x*z - w*y)
        z_y = 2*(y*z + w*x)
        z_z = 1 - 2*(x*x + y*y)

        # Solve for intrinsic roll-then-pitch Euler angles
        # From: z_x = sin(p), z_y = -sin(r)*cos(p), z_z = cos(r)*cos(p)

        # Clamp z_x to valid range for asin to handle numerical errors
        z_x_clamped = max(-1.0, min(1.0, z_x))
        pitch = math.asin(z_x_clamped)

        # Calculate roll from the Y and Z components
        roll = math.atan2(-z_y, z_z)

        return roll, pitch

    def pose_callback(self, msg):
        """Process incoming PoseStamped messages.

        Args:
            msg: PoseStamped message
        """
        # Extract quaternion from pose
        orientation = msg.pose.orientation

        # Calculate roll and pitch using geometric projection method
        roll, pitch = self.quaternion_to_roll_pitch(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )

        # Transport angles to the range [-π, π]
        roll = (roll + math.pi) % (2 * math.pi) - math.pi
        pitch = (pitch + math.pi) % (2 * math.pi) - math.pi

        # correct the sign and the offset of the angles for the rig
        roll_offset = self.get_parameter('roll_offset').value
        pitch_offset = self.get_parameter('pitch_offset').value
        roll = roll_offset - roll
        pitch = pitch_offset + pitch

        # Create and publish Float64MultiArray message
        attitude_msg = Float64MultiArray()
        attitude_msg.data = [roll, pitch]

        self.publisher.publish(attitude_msg)


def main(args=None):
    rclpy.init(args=args)

    servo_hardware_demo_helper = AttitudePublisher()

    try:
        rclpy.spin(servo_hardware_demo_helper)
    except KeyboardInterrupt:
        pass
    finally:
        servo_hardware_demo_helper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
