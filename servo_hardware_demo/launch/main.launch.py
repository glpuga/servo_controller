#!/usr/bin/python3

# Copyright 2024 Gerardo Puga
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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyACM0',
        description='Serial device path for servo hardware'
    )

    rviz_config_filepath = PathJoinSubstitution(
        [
            FindPackageShare('servo_hardware_demo'),
            'rviz',
            'display.rviz',
        ]
    )

    robot_description_content = Command(
        [
            'xacro ',
            PathJoinSubstitution(
                [
                    FindPackageShare('servo_hardware_demo'),
                    'urdf',
                    'servo_controller.urdf.xacro'
                ]
            ),
            ' device:=',
            LaunchConfiguration('device'),
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare('servo_hardware_demo'),
                    'config',
                    'controllers.yaml'
                ]
            ),
            {'robot_description': robot_description_content},
        ],
    )

    controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'servo_joints'],
        output='screen',
    )

    # Event handler to start spawner 3 seconds after controller manager is ready
    spawner_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[
                        controllers_spawner,
                    ],
                )
            ],
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_filepath],
        output='screen',
    )

    attitude_publisher_node = Node(
        package='servo_hardware_demo_helper',
        executable='attitude_publisher_node',
        name='servo_hardware_demo_helper',
        parameters=[
            {'roll_offset': 1.69},
            {'pitch_offset': 1.75},
        ],
        remappings=[
            ('/pose', '/deck_pose'),
            ('commands', '/servo_joints/commands'),
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            device_arg,
            robot_state_publisher_node,
            controller_manager_node,
            spawner_event_handler,
            rviz_node,
            attitude_publisher_node,
        ]
    )
