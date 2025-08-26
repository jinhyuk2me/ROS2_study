# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System

This is a ROS 2 workspace with multiple projects. Use `colcon` build system for all operations:

```bash
# Build entire workspace
colcon build

# Build specific packages
colcon build --packages-select package_name

# Clean build
rm -rf build install log && colcon build

# Source the workspace after building
source install/setup.bash
```

## Project Structure

The workspace contains several ROS 2 projects organized by functionality:

### Core Projects

- **pinky/**: Main robotics project with a mobile robot named "Pinky"
  - `pinky_description/`: Robot URDF/XACRO descriptions and visualization
  - `pinky_gazebo/`: Gazebo simulation environments and launch files
  - `pinky_navigation/`: Navigation stack with Nav2, AMCL, and Cartographer
  - `pinky_simple_navigator/`: Custom navigation node with PID control and state machine

- **ros2_basic_study/**: Educational ROS 2 fundamentals
  - `my_first_package/`: Python package for basic ROS 2 concepts
  - `my_first_package_msgs/`: Custom message definitions

- **ros2_tf_study/**: Transform (TF) system learning materials
- **ros2_control_study/**: ROS 2 control system tutorials
- **ros2_gazebo_ws/**: Additional Gazebo simulation files

## Common Commands

### Simulation and Visualization
```bash
# Launch Gazebo simulation
ros2 launch pinky_gazebo launch_sim.launch.xml

# Launch robot description in RViz
ros2 launch pinky_description display.launch.xml

# Navigation with mapping
ros2 launch pinky_navigation map_building.launch.xml

# Navigation with existing map
ros2 launch pinky_navigation navigation_launch.xml
```

### Testing Python Packages
```bash
# Run Python tests (for packages like my_first_package)
colcon test --packages-select package_name
colcon test-result --verbose
```

## Architecture Overview

### Pinky Robot System
The main robot (Pinky) follows a modular architecture:

1. **Description Layer**: URDF/XACRO files define robot geometry, sensors (lidar, camera, IMU), and Gazebo integration
2. **Simulation Layer**: Gazebo worlds and launch files for various environments
3. **Navigation Layer**: Nav2 integration with custom parameters for AMCL localization and path planning
4. **Control Layer**: Custom SimpleNavigator implementing PID control with state machine (idle → rotate_to_goal → move_to_goal → rotate_to_final)

### Key Integration Points
- Robot uses `map` → `base_link` TF transformations for localization
- Nav2 parameters are tuned for differential drive robot model
- Bridge configuration connects Gazebo simulation with ROS 2 topics
- Custom navigation accepts `PoseStamped` goals on `/goal_pose` topic

### Development Patterns
- Python packages use standard ROS 2 Python structure with `setup.py` and `package.xml`
- C++ packages use CMake with `CMakeLists.txt` and `package.xml`
- Launch files use XML format with parameterized arguments
- Configuration stored in YAML parameter files under `params/` directories

## Frame Conventions
- `map`: Global reference frame
- `base_link`: Robot center
- `base_footprint`: Ground projection of robot center
- `odom`: Odometry frame for local navigation