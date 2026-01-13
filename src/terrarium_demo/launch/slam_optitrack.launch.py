#!/usr/bin/env python3
"""
SLAM launch file for TurtleBot4 with OptiTrack localization.

This launch file runs on the PC/RPi connected to a specific robot.
It launches slam_toolbox in mapping-only mode, using OptiTrack
for localization instead of wheel odometry.

Based on: https://github.com/turtlebot/turtlebot4/blob/jazzy/turtlebot4_navigation/launch/slam.launch.py

Prerequisites:
- The workstation must be running optitrack_slam_bridge_node
  which publishes TF: odom -> base_link from motion capture data on /<namespace>/tf
- The robot must be publishing laser scan data on /<namespace>/scan

Usage:
    ros2 launch terrarium_demo slam_optitrack.launch.py namespace:=turtle8
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from lifecycle_msgs.msg import Transition


def launch_setup(context, *args, **kwargs):
    """Setup function that configures and launches slam_toolbox node directly."""

    # Get launch argument values
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    sync = LaunchConfiguration('sync').perform(context)
    params_file = LaunchConfiguration('params').perform(context)

    # Normalize namespace (strip slashes)
    if namespace:
        namespace = namespace.strip('/')

    # Load and modify the SLAM params
    with open(params_file, 'r') as f:
        slam_params = yaml.safe_load(f)

    # Rewrite topic names with namespace prefix
    # Frame names stay as defined in params file (odom_gt, base_link)
    # - odom_gt avoids conflict with robot's wheel odometry (odom)
    # - base_link connects to URDF frames (rplidar_link, etc.)
    if namespace and 'slam_toolbox' in slam_params:
        params = slam_params['slam_toolbox']['ros__parameters']

        # Don't overwrite frame names - use values from params file
        # odom_frame should be 'odom_gt' to avoid conflict with wheel odometry
        # base_frame should be 'base_link' to connect to URDF

        # Prefix topic names with namespace
        params['scan_topic'] = f'/{namespace}/scan'
        params['map_name'] = f'/{namespace}/map'

    # Add use_sim_time to params
    slam_params['slam_toolbox']['ros__parameters']['use_sim_time'] = use_sim_time.lower() == 'true'

    # Write modified params to a temporary file
    tmp_params_file = os.path.join(
        tempfile.gettempdir(),
        f'slam_optitrack_params_{namespace or "default"}.yaml'
    )
    with open(tmp_params_file, 'w') as f:
        yaml.dump(slam_params, f)

    # Choose sync or async node
    if sync.lower() == 'true':
        executable = 'sync_slam_toolbox_node'
    else:
        executable = 'async_slam_toolbox_node'

    # Build remappings for namespaced topics
    remappings = []
    if namespace:
        remappings = [
            ('/tf', f'/{namespace}/tf'),
            ('/tf_static', f'/{namespace}/tf_static'),
            # Remap scan topic explicitly (slam_toolbox may ignore scan_topic param)
            ('/scan', f'/{namespace}/scan'),
            # Remap map output
            ('/map', f'/{namespace}/map'),
        ]

    # Create the slam_toolbox lifecycle node directly
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable=executable,
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        parameters=[tmp_params_file],
        remappings=remappings,
    )

    # Configure lifecycle node after a short delay to let it initialize
    configure_event = TimerAction(
        period=1.0,
        actions=[
            LogInfo(msg='[LifecycleLaunch] Slamtoolbox node is configuring.'),
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            ),
        ]
    )

    # Activate after configure has time to complete and TF buffer fills
    activate_event = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[LifecycleLaunch] Slamtoolbox node is activating.'),
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )
            ),
        ]
    )

    return [slam_node, configure_event, activate_event]


# Import LaunchConfiguration here to avoid issues with OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get package directory
    terrarium_demo_dir = get_package_share_directory('terrarium_demo')

    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace (e.g., turtle8)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    sync_arg = DeclareLaunchArgument(
        'sync',
        default_value='false',
        description='Use synchronous SLAM (true) or asynchronous (false). Async recommended for real robots.'
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'params',
        default_value=os.path.join(terrarium_demo_dir, 'config', 'slam_optitrack_params.yaml'),
        description='Path to slam_toolbox params file'
    )

    # Set environment variables for logging
    stdout_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_USE_STDOUT', '1'
    )
    buffered_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Create launch description
    ld = LaunchDescription()

    ld.add_action(namespace_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(sync_arg)
    ld.add_action(slam_params_file_arg)
    ld.add_action(stdout_envvar)
    ld.add_action(buffered_envvar)

    # Use OpaqueFunction to perform param rewriting at launch time
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
