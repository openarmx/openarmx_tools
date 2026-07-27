# openarmx_tools

`openarmx_tools` is the OpenArmX utility collection, mainly for **debugging, teaching, parameter tuning, and rapid integration**.  
Each subpackage in this directory can be built and used independently, covering common engineering workflows from RViz visual control to trajectory recording and playback.

> ⚠️ It is recommended to confirm robot controllers are running normally before using these tools for tuning or teaching.

## 🧰 Included Tools

1. `openarmx_joint_slider_panel`
- RViz2 joint slider panel (dual-arm + dual-gripper).
- Suitable for quick pose adjustment, demos, and integration testing.
- Supports segmented step execution to reduce motion shock from large jumps.

![openarmx_joint_slider_panel demo](./assets/openarmx_joint_slider_panel.gif)

2. `openarmx_gripper_panel`
- RViz2 gripper control panel.
- Supports left gripper, right gripper, or synchronized dual-gripper control.
- Sends commands via `GripperCommand` action.

![openarmx_gripper_panel demo](./assets/openarmx_gripper_panel.gif)

3. `openarmx_kp_kd_panel`
- RViz2 KP/KD parameter tuning panel.
- Enables real-time stiffness and damping adjustment for arm and gripper.
- Supports right-arm, left-arm, or dual-arm mode for real hardware tuning.

![openarmx_kp_kd_panel demo](./assets/openarmx_kp_kd_panel.gif)

4. `openarmx_ik_control_panel`
- RViz2 switch for the VR teleoperation IK override.
- Publishes a latched enable state to `/openarmx_teleop_vr/ik_enable_override`.
- Starts in the disabled state for predictable robot startup.

5. `openarmx_teach`
- Trajectory teaching tools (record + playback).
- Records YAML trajectories from `/joint_states`, then replays to arm and gripper controllers.
- Supports joint filtering, rate scaling, and gripper synchronization strategies.

![openarmx_teach demo](./assets/openarmx_teach.gif)

## 🚀 Recommended Workflow (Typical Real-Robot Flow)

1. Start robot base services (bringup/moveit, controllers online)
2. Use `openarmx_kp_kd_panel` to tune stiffness to a suitable level
3. Use `openarmx_joint_slider_panel` or `openarmx_gripper_panel` for motion debugging
4. Use `openarmx_teach` for trajectory recording and playback verification

> ✅ Following this order usually reduces troubleshooting cost for issues like “topics exist but robot does not move”.

## 🔧 Quick Build

Run in your workspace root:

```bash
colcon build --packages-select \
  openarmx_joint_slider_panel \
  openarmx_gripper_panel \
  openarmx_kp_kd_panel \
  openarmx_ik_control_panel \
  openarmx_teach
source install/setup.bash
```

## 📚 Documentation Entry Points

- `openarmx_joint_slider_panel/README_CN.md`
- `openarmx_gripper_panel/README_CN.md`
- `openarmx_kp_kd_panel/README_CN.md`
- `openarmx_ik_control_panel/README.md`
- `openarmx_ik_control_panel/README_CN.md`
- `openarmx_teach/README_CN.md`

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
