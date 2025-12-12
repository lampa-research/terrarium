# Terrarium Demo - Turtle Controller Template

This package provides a C++ template for controlling robots using OptiTrack pose data.

## Creating a Similar Package

To create your own ROS2 C++ package:

```bash
cd /terrarium/src
ros2 pkg create --build-type ament_cmake --dependencies rclcpp geometry_msgs -- your_package_name
```

This will create a package with:
- `CMakeLists.txt` - Build configuration
- `package.xml` - Package dependencies
- `src/` - Source code directory
- `include/` - Header files directory

## Adding Your Node

1. Create your C++ file in the `src/` directory:
```bash
cd your_package_name/src
# Create your_node.cpp file here
```

2. Update `CMakeLists.txt` to build your executable:
```cmake
# Add after find_package() calls
add_executable(your_node_name src/your_node.cpp)
ament_target_dependencies(your_node_name
  rclcpp
  geometry_msgs
)

# Install the executable
install(TARGETS
  your_node_name
  DESTINATION lib/${PROJECT_NAME}
)
```

3. Build your package:
```bash
cd /terrarium
colcon build --symlink-install --packages-select your_package_name
```

4. Source the workspace:
```bash
source /terrarium/install/setup.bash
```

## Running the Node

To run the turtle controller template:

```bash
ros2 run terrarium_demo turtle_controller_template turtle0
```

Replace `turtle0` with your robot's name (turtle1, turtle2, etc.)

### Command Breakdown:
- `ros2 run` - ROS2 command to run a node
- `terrarium_demo` - Package name
- `turtle_controller_template` - Executable/node name
- `turtle0` - Command-line argument (turtle name)

## Template Overview

The template subscribes to:
- `turtleX/pose` - Receives robot pose from OptiTrack

And publishes to:
- `turtleX/cmd_vel` - Sends velocity commands to the robot

### Modifying the Control Logic

Edit the `control_loop()` function in `src/turtle_controller_template.cpp`:

```cpp
void control_loop()
{
  if (!pose_received_) {
    return;  // Wait until we receive the first pose
  }

  // Access current pose
  double x = current_pose_.position.x;
  double y = current_pose_.position.y;
  double z = current_pose_.position.z;

  // Create velocity command
  auto cmd_vel_msg = geometry_msgs::msg::Twist();

  // Your control logic here
  cmd_vel_msg.linear.x = 0.5;   // Move forward
  cmd_vel_msg.angular.z = 0.1;  // Turn

  // Publish command
  cmd_vel_publisher_->publish(cmd_vel_msg);
}
```

After modifying, rebuild:
```bash
cd /terrarium
colcon build --symlink-install --packages-select terrarium_demo
source install/setup.bash
```
