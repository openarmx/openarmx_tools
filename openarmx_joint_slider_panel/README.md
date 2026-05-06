# OpenArmX Joint Slider Panel

## Package Overview

`openarmx_joint_slider_panel` is an RViz2 panel plugin for directly controlling OpenArmX dual-arm robot joints and grippers using sliders.

This plugin is designed for debugging, demos, and fast system integration. In RViz2, users can adjust dual-arm joint and gripper targets, and large target changes are split into smaller command steps through segmented execution for smoother and more controllable motion.

## Features

- 16 sliders: left/right arm joints (7 + 7) and left/right grippers.
- Continuous real-time synchronization from `/joint_states` when the panel is idle.
- Direct slider control: commands are sent while dragging (no Apply button required).
- Segmented execution with a background thread:
  - `Joint Step`: controls maximum arm delta per cycle (mrad/cycle).
  - `Gripper Step`: controls maximum gripper delta per cycle (mm/cycle).
  - Large slider jumps are automatically split into multiple small command steps.
- Preview model is disabled.
- `Hands Up` button sends both arms to the hands-up preset (joint4=1.8 rad, others 0).
- `Home` button sets both arm and gripper targets to zero and executes segmented return-to-zero.
- Forward position backend only:
  - `/left_forward_position_controller/commands`
  - `/right_forward_position_controller/commands`

## Build

```bash
cd ~/openarmx_ws2
colcon build --packages-select openarmx_joint_slider_panel
source install/setup.bash
```

## Usage in RViz2

1. Start the robot stack (`demo.launch.py` / `demo_sim.launch.py` / bringup).
2. Open RViz2.
3. Go to `Panels` -> `Add New Panel` -> `openarmx_joint_slider_panel/JointSliderPanel`.
4. Set `Joint Step` and `Gripper Step` based on the desired smoothness.
5. Drag sliders to directly control the robot with segmented commands.
6. Click `Hands Up` to execute the hands-up preset.
7. Click `Home` to perform segmented return-to-zero.

**Or use `openarmx_preview_bringup` for one-click startup.**

## Typical Controller Topic Mapping

- `/left_forward_position_controller/commands`
- `/right_forward_position_controller/commands`

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.

For details, see [LICENSE_CN.md](LICENSE_CN.md) or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Author

- **Li QingRan**
- Company: Chengdu Changshu Robot Co., Ltd.
- Website: https://openarmx.com/

## Version

1.0.0

---

## Contact Us

### Chengdu Changshu Robotics Co., Ltd.

| Contact           | Information                                                                                                  |
| ----------------- | ------------------------------------------------------------------------------------------------------------ |
| 📧 Email          | [openarmrobot@gmail.com](mailto:openarmrobot@gmail.com)                                                      |
| 📱 Phone / WeChat | +86-17746530375                                                                                              |
| 🌐 Website        | [https://openarmx.com/](https://openarmx.com/)                                                               |
| 📍 Address        | Huacheng Machinery Plant, No.11 Xinye 8th Street, West Area, Tianjin Economic-Technological Development Area |
| 👤 Contact Person | Mr. Wang                                                                                                     |
