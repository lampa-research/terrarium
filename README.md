# Terrarium ROS2 Workspace

ROS 2 Jazzy workspace for OptiTrack motion capture with multi-robot support.

## Quick Start

### Clone with Submodules
```bash
git clone --recurse-submodules <repo-url>
```

Or if already cloned:
```bash
git submodule init
git submodule update
```

### Build Podman Container
```bash
cd podman
./build-and-run.sh
```

### Enter Container
```bash
cd podman
./enter.sh
```

### Build Workspace
Inside the container:
```bash
cd /terrarium
source setup.bash
colcon build
source install/setup.bash
```

## Dependencies

Uses git submodules for mocap packages:
- `mocap4r2` - Core mocap4ros2 package
- `mocap4r2_msgs` - Message definitions
- `mocap4ros2_optitrack` - OptiTrack driver

Update submodules:
```bash
git submodule update --remote
```

## Configuration

**Discovery Servers:** Edit `setup.bash` to configure robot discovery servers.

**OptiTrack:** Edit `src/terrarium_optitrack/config/optitrack_params.yaml` for server addresses.

## Launch

```bash
source /terrarium/setup.bash
ros2 launch terrarium_optitrack terrarium_optitrack.launch.py
```
