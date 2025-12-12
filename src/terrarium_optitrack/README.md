# terrarium_optitrack

OptiTrack integration for Terrarium multi-robot system.

## Overview

Subscribes to OptiTrack rigid body data and publishes individual robot poses.

**Input:** `/rigid_bodies` (mocap4r2_msgs/RigidBodies)
**Output:** `turtleX/pose` (geometry_msgs/Pose) - dynamically created per rigid body

## Nodes

**optitrack_pose_publisher_node**
- Converts rigid body data to individual pose topics
- Creates publishers on-demand for each detected rigid body
- Topic naming: rigid body "3" → `turtle3/pose`

## Launch

```bash
ros2 launch terrarium_optitrack terrarium_optitrack.launch.py
```

Starts:
- `mocap4r2_optitrack_driver_node` - OptiTrack driver (lifecycle node)
- `optitrack_pose_publisher_node` - Pose republisher

## Configuration

Edit `config/optitrack_params.yaml`:
- `server_address` - OptiTrack server IP
- `local_address` - This machine's IP
- `connection_type` - Unicast/Multicast
- `rigid_body_name` - Body to track (default: "ground")
