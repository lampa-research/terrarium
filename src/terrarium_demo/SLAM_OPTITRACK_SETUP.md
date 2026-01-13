# SLAM with OptiTrack Localization

This guide explains how to run slam_toolbox on TurtleBot4 robots using OptiTrack motion capture for localization instead of wheel odometry.

## Architecture

```
                              WORKSTATION (Container)
┌─────────────────────────────────────────────────────────────────────────────┐
│                                                                             │
│  ┌──────────────────────┐    ┌───────────────────────────────────────────┐  │
│  │  OptiTrack Driver    │───►│  optitrack_slam_bridge_all.launch.py     │  │
│  │  (mocap4r2)          │    │                                           │  │
│  │  /rigid_bodies       │    │  Launches 5 bridge nodes, one per robot:  │  │
│  └──────────────────────┘    │    - optitrack_slam_bridge_turtle2        │  │
│                              │    - optitrack_slam_bridge_turtle3        │  │
│                              │    - optitrack_slam_bridge_turtle5        │  │
│                              │    - optitrack_slam_bridge_turtle7        │  │
│                              │    - optitrack_slam_bridge_turtle8        │  │
│                              │                                           │  │
│                              │  Each publishes to /<robotX>/tf:          │  │
│                              │    - TF: map -> odom (static identity)    │  │
│                              │    - TF: odom -> base_link (from mocap)   │  │
│                              │    - /turtleX/odom (nav_msgs/Odometry)    │  │
│                              └───────────────────────────────────────────┘  │
│                                                │                            │
└────────────────────────────────────────────────┼────────────────────────────┘
                                                 │
              ROS2 Discovery Server / DDS network
                                                 │
         ┌───────────────┬───────────────┬───────┴───────┬───────────────┐
         ▼               ▼               ▼               ▼               ▼
    ┌─────────┐     ┌─────────┐     ┌─────────┐     ┌─────────┐     ┌─────────┐
    │ turtle2 │     │ turtle3 │     │ turtle5 │     │ turtle7 │     │ turtle8 │
    │  (RPi)  │     │  (RPi)  │     │  (RPi)  │     │  (RPi)  │     │  (RPi)  │
    │         │     │         │     │         │     │         │     │         │
    │ LIDAR   │     │ LIDAR   │     │ LIDAR   │     │ LIDAR   │     │ LIDAR   │
    │ scan    │     │ scan    │     │ scan    │     │ scan    │     │ scan    │
    │         │     │         │     │         │     │         │     │         │
    │ SLAM    │     │ SLAM    │     │ SLAM    │     │ SLAM    │     │ SLAM    │
    │ toolbox │     │ toolbox │     │ toolbox │     │ toolbox │     │ toolbox │
    └─────────┘     └─────────┘     └─────────┘     └─────────┘     └─────────┘
```

## How It Works

### Standard SLAM (wheel odometry)
- Robot publishes `odom -> base_link` TF from wheel encoders
- slam_toolbox uses laser scans + odometry for mapping
- slam_toolbox publishes `map -> odom` TF to correct drift

### OptiTrack SLAM (this setup)
- **Robot's wheel odometry must be DISABLED** (to avoid conflicting `odom -> base_link` transforms)
- `optitrack_slam_bridge_node` publishes `odom -> base_link` TF from motion capture
- `optitrack_slam_bridge_node` publishes static `map -> odom` identity transform
- slam_toolbox uses laser scans + OptiTrack pose for mapping
- `transform_publish_period: 0.0` disables slam_toolbox's `map -> odom` publishing

**Benefit**: No odometry drift, perfect localization from ground truth

### TF Tree (per robot namespace, e.g., /turtle3/tf)

```
map
 └── odom (static identity from OptiTrack bridge)
      └── base_link (dynamic from OptiTrack rigid body pose)
           ├── rplidar_link (static from robot URDF)
           ├── base_footprint (static from robot URDF)
           └── ... other URDF frames
```

## Prerequisites

### System Packages

**On all machines (workstation, RPis):**
```bash
# NTP for clock synchronization (CRITICAL!)
sudo apt install chrony

# Enable and start
sudo systemctl enable chrony
sudo systemctl start chrony
```

**On workstation:**
```bash
# ROS2 Jazzy base
sudo apt install ros-jazzy-desktop

# OptiTrack driver
# (install mocap4r2 package - see mocap4r2 repository)
```

**On RPi / PC running SLAM:**
```bash
# slam_toolbox
sudo apt install ros-jazzy-slam-toolbox

# TF2 tools for debugging
sudo apt install ros-jazzy-tf2-tools
```

### Clock Synchronization (CRITICAL!)

All devices MUST have synchronized clocks. The TF lookups will fail if timestamps differ by more than ~2 seconds.

```bash
# Check clock sync on each device
date +%s

# Force immediate sync
sudo chronyc makestep

# Or use ntpdate for quick sync
sudo apt install ntpdate
sudo ntpdate pool.ntp.org
```

### Disable Robot's Wheel Odometry TF

The Create3 base publishes `odom -> base_link` and `odom -> base_footprint` TF transforms from wheel encoders. This conflicts with OptiTrack's transform and MUST be disabled on each TurtleBot4.

#### Steps to Disable

1. **Access the Create3 webserver** on each robot:
   - Connect to the robot's network
   - Open browser to `192.168.186.2` (USB-C connection) or the Create3's IP address

2. **Go to Application Configuration** page

3. **Edit the ROS 2 Parameters file** and add:
   ```yaml
   robot_state:
     ros__parameters:
       publish_odom_tfs: false
   ```

   Note: If the Create3 has a namespace configured (e.g., `/turtle3/_do_not_use`), the namespace is automatically prepended to node names, so you don't need to include it in the YAML.

4. **Restart the Create3 application** via the webserver or power cycle the robot

5. **Verify the parameter is set**:
   ```bash
   ros2 param get /turtle3/_do_not_use/robot_state publish_odom_tfs
   # Should return: Boolean value is: False
   ```

6. **Restart the TurtleBot4 service** on the RPi:
   ```bash
   ssh ubuntu@<robot_ip>
   sudo systemctl restart turtlebot4
   ```

#### Verify Only OptiTrack Publishes odom->base_link

After disabling, check that only the OptiTrack bridge publishes to the TF topic:

```bash
ros2 topic info /turtle3/tf -v
```

You should see these publishers:
- `optitrack_slam_bridge_turtle3` - publishes `odom -> base_link` from OptiTrack
- `robot_state_publisher` - publishes static URDF frames (base_link -> sensors)

You should NOT see `create3_repub` publishing dynamic transforms anymore.

### Build the Packages

```bash
# On workstation
cd ~/terrarium_ws
colcon build --packages-select terrarium_optitrack
source install/setup.bash

# On RPi
cd ~/tb4_ws
colcon build --packages-select terrarium_demo
source install/setup.bash
```

## Step-by-Step Setup

### 1. Workstation: Start OptiTrack Driver

```bash
# Terminal 1: OptiTrack driver (mocap4r2)
ros2 launch terrarium_optitrack terrarium_optitrack.launch.py
```

Verify rigid bodies are being published:
```bash
ros2 topic echo /rigid_bodies --once
```

### 2. Workstation: Start SLAM Bridges for All Robots

```bash
# Terminal 2: Launch all 5 bridges at once
ros2 launch terrarium_optitrack optitrack_slam_bridge_all.launch.py
```

This launches one bridge node per robot, each publishing to its namespaced TF topic.

Verify bridges are publishing:
```bash
# Check TF for turtle3
ros2 topic echo /turtle3/tf --once

# Should show odom -> base_link transform
```

### 3. RPi: Start SLAM

On each robot's RPi:

```bash
# Source workspace
source ~/tb4_ws/install/setup.bash

# Launch slam_toolbox with OptiTrack config
ros2 launch terrarium_demo slam_optitrack.launch.py namespace:=turtle3
```

Wait for activation (~10 seconds). Verify it's active:
```bash
ros2 lifecycle get /turtle3/slam_toolbox
# Should show: active [3]
```

### 4. Verify TF Tree

```bash
# On RPi - check complete TF tree
ros2 run tf2_tools view_frames --ros-args -r /tf:=/turtle3/tf -r /tf_static:=/turtle3/tf_static

# Check specific transform
ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/turtle3/tf -r /tf_static:=/turtle3/tf_static
```

Expected TF tree:
- `map -> odom` (static, rate 10000 = static TF)
- `odom -> base_link` (dynamic, ~30-35 Hz from OptiTrack)
- `base_link -> rplidar_link` (static, from URDF)

### 5. Visualize in RViz

On a machine with display:

```bash
ros2 run rviz2 rviz2
```

Add displays:
- **Map**: Topic `/turtle3/map`
- **LaserScan**: Topic `/turtle3/scan`
- **TF**: Enable to see frame tree
- **RobotModel**: Topic `/turtle3/robot_description`

Set Fixed Frame to `map`.

### 6. Drive the Robot

Move the robot to build the map:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/turtle3/cmd_vel
```

The map will update after the robot moves `minimum_travel_distance` (0.1m).

### 7. Save the Map

When mapping is complete:

```bash
# Using slam_toolbox service
ros2 service call /turtle3/slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "name: {data: 'my_map'}"

# Or using map_saver_cli
ros2 run nav2_map_server map_saver_cli -f my_map \
    --ros-args -p map_topic:=/turtle3/map
```

## Configuration Files

### slam_optitrack_params.yaml

Location: `terrarium_demo/config/slam_optitrack_params.yaml`

Key settings:
```yaml
slam_toolbox:
  ros__parameters:
    # Frame configuration
    odom_frame: odom           # Must match OptiTrack bridge odom_frame
    map_frame: map
    base_frame: base_link      # Must match OptiTrack bridge base_frame

    # CRITICAL: Disable map->odom TF publishing (OptiTrack provides this)
    transform_publish_period: 0.0

    # Timing - increase if TF lookups fail
    transform_timeout: 2.0
    tf_buffer_duration: 30.0

    # Movement thresholds
    minimum_travel_distance: 0.1
    minimum_travel_heading: 0.1
```

### optitrack_slam_bridge_all.launch.py

Location: `terrarium_optitrack/launch/optitrack_slam_bridge_all.launch.py`

Robot configuration:
```python
ROBOTS = [
    ('2', 'turtle2'),   # (rigid_body_name, robot_namespace)
    ('3', 'turtle3'),
    ('5', 'turtle5'),
    ('7', 'turtle7'),
    ('8', 'turtle8'),
]
```

Per-robot parameters:
```python
parameters=[{
    'rigid_body_name': rigid_body_name,  # OptiTrack rigid body name
    'robot_namespace': '',                # Empty = no frame prefix
    'odom_frame': 'odom',                 # Odom frame name
    'base_frame': 'base_link',            # Base frame name
    'map_frame': 'map',
    'publish_map_odom_tf': True,          # Publish static map->odom
    'publish_odom_msg': True,             # Publish nav_msgs/Odometry
    'z_offset': 0.0,
}]
```

TF topic remapping (publishes to robot's namespaced TF):
```python
remappings=[
    ('/tf', f'/{robot_namespace}/tf'),
    ('/tf_static', f'/{robot_namespace}/tf_static'),
    ('/odom', f'/{robot_namespace}/odom'),
]
```

## Launch Arguments

### slam_optitrack.launch.py (RPi)

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace` | `` | Robot namespace (e.g., `turtle3`) |
| `sync` | `false` | Use sync SLAM (false=async, recommended) |
| `use_sim_time` | `false` | Use simulation time |
| `params` | `slam_optitrack_params.yaml` | SLAM config file |

### optitrack_slam_bridge_all.launch.py (Workstation)

No arguments - launches all 5 robots as configured in the file.

For single robot, use `optitrack_slam_bridge.launch.py`:

| Argument | Default | Description |
|----------|---------|-------------|
| `rigid_body_name` | (required) | OptiTrack rigid body name |
| `robot_namespace` | `` | Robot namespace for TF topics |
| `publish_map_odom_tf` | `true` | Publish static map->odom |
| `publish_odom_msg` | `true` | Publish nav_msgs/Odometry |
| `z_offset` | `0.0` | Z offset for base_link |

## Troubleshooting

### "Failed to compute odom pose" errors

**Cause**: TF lookup failing - timestamps don't match.

**Solutions**:
1. Sync clocks: `sudo chronyc makestep` on all devices
2. Check timestamps match:
   ```bash
   # Compare these - should be within 0.5s
   ros2 topic echo /turtle3/scan --once --field header.stamp
   ros2 topic echo /turtle3/tf --once --field transforms[0].header.stamp
   ```
3. Increase `transform_timeout` in slam_optitrack_params.yaml

### "Message Filter dropping message: queue is full"

**Cause**: TF buffer not ready or transform lookup too slow.

**Solutions**:
1. Wait longer after launch (TF buffer needs to fill)
2. Increase `tf_buffer_duration` in params
3. Check clock sync

### slam_toolbox stuck in "Activating" state

**Cause**: TF tree incomplete.

**Solutions**:
1. Check TF tree: `ros2 run tf2_tools view_frames --ros-args -r /tf:=/turtle3/tf -r /tf_static:=/turtle3/tf_static`
2. Verify these transforms exist:
   - `map -> odom`
   - `odom -> base_link`
   - `base_link -> rplidar_link`
3. Check OptiTrack bridge is running and tracking the rigid body

### No map being published

**Cause**: slam_toolbox not receiving valid scans or not processing them.

**Solutions**:
1. Check scan topic: `ros2 topic hz /turtle3/scan`
2. Move the robot (needs `minimum_travel_distance` movement)
3. Check lifecycle state: `ros2 lifecycle get /turtle3/slam_toolbox`
4. Check for errors in slam_toolbox terminal output

### Robot pose jumping in RViz

**Cause**: Multiple sources publishing `odom -> base_link`.

**Solutions**:
1. Disable robot's wheel odometry (CRITICAL!)
2. Verify only OptiTrack bridge publishes `odom -> base_link`:
   ```bash
   ros2 topic info /turtle3/tf -v
   # Should show only one source for odom->base_link
   ```

### TF frames not found

**Cause**: Frame name mismatch or TF not being published.

**Solutions**:
1. Check frame names match between:
   - OptiTrack bridge (`odom_frame`, `base_frame` params)
   - slam_toolbox params (`odom_frame`, `base_frame`)
   - Robot URDF
2. Verify TF remapping in launch files

## Debugging Commands

```bash
# Check TF tree
ros2 run tf2_tools view_frames --ros-args -r /tf:=/turtle3/tf -r /tf_static:=/turtle3/tf_static

# Monitor TF timing
ros2 run tf2_ros tf2_monitor --ros-args -r /tf:=/turtle3/tf -r /tf_static:=/turtle3/tf_static

# Echo specific transform
ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/turtle3/tf -r /tf_static:=/turtle3/tf_static

# Check node info
ros2 node info /turtle3/slam_toolbox
ros2 node info /optitrack_slam_bridge_turtle3

# Check lifecycle state
ros2 lifecycle get /turtle3/slam_toolbox

# Check topic rates
ros2 topic hz /turtle3/scan
ros2 topic hz /turtle3/tf
ros2 topic hz /turtle3/map

# Check OptiTrack data
ros2 topic echo /rigid_bodies --once

# Compare timestamps (for clock sync debugging)
ros2 topic echo /turtle3/scan --once --field header.stamp
ros2 topic echo /rigid_bodies --once --field header.stamp
```

## References

- [TurtleBot4 SLAM Tutorial](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html)
- [TurtleBot4 Navigation Package](https://github.com/turtlebot/turtlebot4/tree/jazzy/turtlebot4_navigation)
- [slam_toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [mocap4r2 OptiTrack Driver](https://github.com/MOCAP4ROS2-Project/mocap4r2)
