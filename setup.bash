#!/bin/bash

# Source ROS 2 setup
source /opt/ros/jazzy/setup.bash

# Source workspace setup if it exists
if [ -f "/terrarium/install/setup.bash" ]; then
    source /terrarium/install/setup.bash
fi

# ROS 2 DDS Configuration
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_SUPER_CLIENT=True
export ROS_DOMAIN_ID=0

# Robot discovery servers
# Format: IP:PORT (semicolon-separated list)
# Add or modify robot entries as needed
robot_0=""
robot_1=""
robot_2=""
robot_3="192.168.1.161:11811"
robot_4=""
robot_5="192.168.1.132:11811"
robot_6=""
robot_7=""
robot_8=""
robot_9=""

# Build ROS_DISCOVERY_SERVER list (filters out empty entries)
export ROS_DISCOVERY_SERVER="${robot_0};${robot_1};${robot_2};${robot_3};${robot_4};${robot_5};${robot_6};${robot_7};${robot_8};${robot_9};"

echo "Terrarium environment configured:"
echo "  RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "  ROS_SUPER_CLIENT: $ROS_SUPER_CLIENT"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  ROS_DISCOVERY_SERVER: $ROS_DISCOVERY_SERVER"
