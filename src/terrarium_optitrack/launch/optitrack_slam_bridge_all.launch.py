#!/usr/bin/env python3
"""
OptiTrack SLAM Bridge launch file for ALL robots (workstation).

This launches optitrack_slam_bridge_node for all 5 TurtleBot4 robots:
turtle2, turtle3, turtle5, turtle7, turtle8

Each node publishes TF transforms (odom -> base_link) from OptiTrack data
for use with slam_toolbox.

Usage:
    ros2 launch terrarium_optitrack optitrack_slam_bridge_all.launch.py
"""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


# Robot configurations: (rigid_body_name, robot_namespace)
ROBOTS = [
    ('2', 'turtle2'),
    ('3', 'turtle3'),
    ('5', 'turtle5'),
    ('7', 'turtle7'),
    ('8', 'turtle8'),
]


def generate_launch_description():
    # Set line buffering
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)

    # Create a bridge node for each robot
    # Each node publishes to the robot's namespaced /tf topic (e.g., /turtle3/tf)
    # so the TF tree is visible to slam_toolbox running on that robot
    for rigid_body_name, robot_namespace in ROBOTS:
        node = Node(
            package='terrarium_optitrack',
            executable='optitrack_slam_bridge_node',
            name=f'optitrack_slam_bridge_{robot_namespace}',
            output='screen',
            parameters=[{
                'rigid_body_name': rigid_body_name,
                'robot_namespace': '',  # Empty = unprefixed frame names
                'odom_frame': 'odom',  # Use odom_gt to avoid conflict with robot's wheel odometry (odom)
                'base_frame': 'base_link',  # Keep base_link to connect to URDF frames (rplidar_link, etc.)
                'map_frame': 'map',
                'publish_map_odom_tf': True,
                'publish_odom_msg': True,
                'z_offset': 0.0,
            }],
            # Remap TF and odom to namespaced topics to match robot's TF tree
            remappings=[
                ('/tf', f'/{robot_namespace}/tf'),
                ('/tf_static', f'/{robot_namespace}/tf_static'),
                ('/odom', f'/{robot_namespace}/odom'),
            ],
        )
        ld.add_action(node)

    return ld
