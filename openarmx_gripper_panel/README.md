# OpenArm Gripper Panel

[中文文档](README_CN.md) | **English**

## Overview

`openarmx_gripper_panel` is an RViz2 plugin for controlling single or dual grippers in OpenArm robotic systems. It provides an intuitive graphical interface within RViz to control gripper positions through ROS2 action commands.

## Features

- **Dual Gripper Support**: Control left, right, or both grippers simultaneously
- **Intuitive GUI**: Slider-based position control with preset quick buttons
- **Real-time Feedback**: Visual status indicators for connection and command status
- **RViz Integration**: Seamlessly integrates as an RViz panel
- **Configuration Persistence**: Saves gripper selection preferences

## Architecture

### Components

- **RViz Plugin**: Qt5-based GUI panel that integrates into RViz2
- **Action Clients**: Communicates with gripper controllers via `control_msgs/action/GripperCommand`
- **Controller Interfaces**:
  - Left Gripper: `/left_gripper_controller/gripper_cmd`
  - Right Gripper: `/right_gripper_controller/gripper_cmd`

### Control Range

- **Position Range**: 0-44mm
- **Presets**:
  - Closed: 0mm
  - Half-open: 22mm
  - Open: 44mm
- **Max Effort**: 10.0N

## Installation

### Dependencies

This package requires the following dependencies:

- ROS2 (Humble or later)
- `rclcpp`
- `rclcpp_action`
- `control_msgs`
- `rviz_common`
- `rviz_default_plugins`
- `pluginlib`
- Qt5 (Core, Widgets)

### Build

```bash
# Navigate to your workspace
cd ~/openarmx_ws

# Build the package
colcon build --packages-select openarmx_gripper_panel

# Source the workspace
source install/setup.bash
```

## Usage

### 1. Launch with RViz

The gripper panel is automatically available in RViz once the package is built and sourced.

```bash
# Launch RViz
ros2 run rviz2 rviz2

# Or launch with moveit
ros2 launch openarmx_bimanual_moveit_config demo.launch.py can_fd:=false
```

### 2. Add Panel to RViz

1. In RViz, go to **Panels** → **Add New Panel**
2. Select **openarmx_gripper_panel/GripperPanel**
3. The panel will appear in your RViz window

### 3. Control Grippers

1. **Select Target**: Choose from dropdown menu:
   - Left Gripper
   - Right Gripper
   - Dual Grippers (Synchronized)

2. **Set Position**:
   - Use the slider to adjust position (0-44mm)
   - Or click quick preset buttons:
     - **Close (0mm)**: Fully closed position
     - **Half (22mm)**: Half-open position
     - **Open (44mm)**: Fully open position

3. **Execute Command**:
   - Click the green **"Apply - Execute Command"** button
   - Status label will show command feedback

### 4. Status Indicators

The panel displays real-time status information:

- Green: Command sent successfully / Controllers ready
- Yellow: One gripper controller not connected
- Red: All gripper controllers not connected
- Gray: Ready/Idle state

## Technical Details

### ROS2 Interfaces

**Action Type**: `control_msgs/action/GripperCommand`

**Action Goal Structure**:
```cpp
goal_msg.command.position = position;  // Target position in meters (0.0-0.044)
goal_msg.command.max_effort = 10.0;    // Maximum effort in Newtons
```

### Topics and Services

The plugin connects to the following action servers:
- `/left_gripper_controller/gripper_cmd` (control_msgs/action/GripperCommand)
- `/right_gripper_controller/gripper_cmd` (control_msgs/action/GripperCommand)

### Synchronized Control

When "Dual Grippers (Synchronized)" is selected:
1. Command is sent to right gripper first
2. 5ms delay
3. Command is sent to left gripper
4. This ensures near-simultaneous execution with minimal time offset

### Configuration

Panel configuration is automatically saved in RViz config files (.rviz):
- Gripper selection preference
- Panel position and size

## Plugin Registration

The plugin is registered with RViz through the pluginlib mechanism:

```xml
<library path="openarmx_gripper_panel">
  <class name="openarmx_gripper_panel/GripperPanel"
         type="openarmx_gripper_panel::GripperPanel"
         base_class_type="rviz_common::Panel">
    <description>
      OpenArm gripper control panel for controlling single or dual grippers in RViz
    </description>
  </class>
</library>
```

## Troubleshooting

### Panel doesn't appear in RViz

```bash
# Ensure package is built
colcon build --packages-select openarmx_gripper_panel

# Source workspace
source install/setup.bash

# Check plugin registration
ros2 pkg prefix openarmx_gripper_panel

# Verify plugin description file
cat install/openarmx_gripper_panel/share/openarmx_gripper_panel/plugins/plugin_description.xml
```

### Controllers not connecting

1. Verify gripper controllers are running:
```bash
ros2 action list | grep gripper
```

Expected output:
```
/left_gripper_controller/gripper_cmd
/right_gripper_controller/gripper_cmd
```

2. Check controller status:
```bash
ros2 action info /left_gripper_controller/gripper_cmd
ros2 action info /right_gripper_controller/gripper_cmd
```

3. Test action manually:
```bash
ros2 action send_goal /left_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.022, max_effort: 10.0}}"
```

### Panel shows warning messages

- **"Warning: Gripper controllers not connected"**: Neither controller is available
  - Check if robot hardware is connected
  - Verify controllers are launched

- **"Warning: Left gripper controller not connected"**: Only left controller unavailable
  - Right gripper can still be controlled

- **"Warning: Right gripper controller not connected"**: Only right controller unavailable
  - Left gripper can still be controlled

## Development

### File Structure

```
openarmx_gripper_panel/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata
├── README.md                   # English documentation
├── README_zh.md                # Chinese documentation
├── include/
│   └── openarmx_gripper_panel/
│       └── gripper_panel.hpp   # Header file
├── src/
│   └── gripper_panel.cpp       # Implementation
├── plugins/
│   └── plugin_description.xml  # Plugin registration
└── resource/
    └── openarmx_gripper_panel  # Resource marker
```

### Key Classes

**GripperPanel**: Main panel class inheriting from `rviz_common::Panel`
- Qt slots for user interactions
- ROS2 action clients for gripper control
- Configuration save/load functionality

### Building from Source

```bash
# Clone the repository (if not already in workspace)
cd ~/openarmx_ws/src/openarmx_tools/

# Install dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build
cd ~/openarmx_ws
colcon build --packages-select openarmx_gripper_panel --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source and test
source install/setup.bash
ros2 run rviz2 rviz2
```

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

For more details, see the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Author

- **Wei Lindong** (魏林栋)
- Company: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- Website: https://openarmx.com/

## Version

1.0.0

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.

| Contact           | Information                                                                                                  |
| ----------------- | ------------------------------------------------------------------------------------------------------------ |
| 📧 Email          | [openarmrobot@gmail.com](mailto:openarmrobot@gmail.com)                                                      |
| 📱 Phone / WeChat | +86-17746530375                                                                                              |
| 🌐 Website        | [https://openarmx.com/](https://openarmx.com/)                                                               |
| 📍 Address        | Huacheng Machinery Plant, No.11 Xinye 8th Street, West Area, Tianjin Economic-Technological Development Area |
| 👤 Contact Person | Mr. Wang                                                                                                     |
