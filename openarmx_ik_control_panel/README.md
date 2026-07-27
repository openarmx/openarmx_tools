# OpenArmX IK Control Panel

[中文文档](README_CN.md) | **English**

## Overview

`openarmx_ik_control_panel` is an RViz2 panel plugin that switches the manual IK enable override used by `openarmx_teleop_vr`.

This panel is not an IK solver and does not publish robot joint commands. It publishes a Boolean override that tells the VR teleoperation node whether relative IK control should remain enabled without requiring the controller grip value to exceed its configured threshold.

## Behavior

- The initial state is `OFF` unless an RViz configuration explicitly restores another value.
- Clicking `ik逆解开始` switches the override to `ON`.
- Clicking `ik逆解结束` switches the override back to `OFF`.
- `ON`: the VR node treats manual IK override as active regardless of the grip threshold.
- `OFF`: the VR node returns to its normal grip-threshold behavior.
- The selected state is saved in the RViz configuration as `ik_enabled`.

Enabling the override does not generate motion by itself. Robot motion still depends on valid VR pose input and the running teleoperation and controller stack.

## ROS 2 Interface

| Direction | Topic | Type | QoS |
| --- | --- | --- | --- |
| Publisher | `/openarmx_teleop_vr/ik_enable_override` | `std_msgs/msg/Bool` | Reliable, transient local, depth 1 |

The default `openarmx_teleop_vr` configuration subscribes to the same topic with matching QoS settings.

## Build

From the workspace root:

```bash
cd ~/openarmx/openarmx_ws_new
colcon build --packages-select openarmx_ik_control_panel --symlink-install
source install/setup.bash
```

## Usage

### O6 Bringup

The O6 RViz configuration already loads this panel and starts it in the disabled state:

```bash
ros2 launch openarmx_hand_bringup openarmx.bimanual.o6.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=true
```

### Add It Manually in RViz2

1. Build the package and source the workspace.
2. Open RViz2.
3. Select `Panels` -> `Add New Panel`.
4. Select `openarmx_ik_control_panel/IkControlPanel`.

## Verification

Check the published state:

```bash
ros2 topic echo /openarmx_teleop_vr/ik_enable_override std_msgs/msg/Bool
```

Check that ROS 2 can find the package:

```bash
ros2 pkg prefix openarmx_ik_control_panel
```

If the panel is missing from RViz, rebuild the package and source `install/setup.bash` in the terminal that launches RViz.

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

For more details, see the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Author

- Company: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- Website: https://openarmx.com/

## Version

**Current Version**: 6.0.0

## Acknowledgments

This package is part of the OpenArmX robotic platform ecosystem, developed for research and industrial applications in collaborative robotics.

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.

| Contact           | Information                                                                                                  |
| ----------------- | ------------------------------------------------------------------------------------------------------------ |
| 📧 Email          | [openarmrobot@gmail.com](mailto:openarmrobot@gmail.com)                                                      |
| 📱 Phone / WeChat | +86-17746530375                                                                                              |
| 🌐 Website        | [https://openarmx.com/](https://openarmx.com/)                                                               |
| 🌐 Documentation  | [http://docs.openarmx.com/](http://docs.openarmx.com/)                                                               |
| 📍 Address        | Huacheng Machinery Plant, No.11 Xinye 8th Street, West Area, Tianjin Economic-Technological Development Area |
| 👤 Contact Person | Mr. Wang                                                                                                     |
