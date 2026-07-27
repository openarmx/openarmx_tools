# OpenArmX KP/KD Panel

English | [简体中文](README_CN.md)

### Overview
`openarmx_kp_kd_panel` is an RViz2 panel plugin that lets you tune KP (stiffness) and KD (damping) for 8 joints (7 arm joints + gripper) in real time. Slider values are automatically mapped to each motor’s valid range and applied through ROS 2 parameter services.

### Highlights
- Grouped sliders: arm (Joint 1–7) and gripper (Joint 8) each have KP/KD sliders and “Reset to default” buttons
- Auto range mapping: RS04/RS03/RS00 limits are handled for you
- Multi-arm control: choose Right, Left, or Both; connects to `/openarmx_right_hardware_params` and `/openarmx_left_hardware_params`
- Live status: connection state, simulation detection, and apply results shown inline
- Config persistence: sliders and mode are stored in the RViz config
- UI-friendly: scrollable layout, color-coded status, compact hints

### Motor ranges
| Joints | Motor | KP range | KD range |
| --- | --- | --- | --- |
| Joint 1–2 | RS04 | 0–5000 | 0–100 |
| Joint 3–4 | RS03 | 0–5000 | 0–100 |
| Joint 5–8 | RS00 | 0–500  | 0–5 |

Slider ranges: arm KP 0–1000, arm KD 0–100; gripper KP 0–1000, gripper KD 0–100. The panel shows mapped numeric values in real time.

### Default values (Reset buttons)
- Arm KP: 10  → RS04/RS03=50.0, RS00=5.0
- Arm KD: 3   → RS04/RS03=3.0, RS00=0.15
- Gripper KP: 100 → RS00=50.0
- Gripper KD: 50  → RS00=2.5

### Build & install
Standard ROS 2 ament package depending on `rclcpp`, `rviz_common`, `rviz_rendering`, `pluginlib`, `Qt5 Widgets`. Build inside your workspace:
```bash
colcon build --packages-select openarmx_kp_kd_panel
```
Source the workspace (e.g., `source install/setup.bash`) and load the plugin in RViz.

### How to use
1) **Ensure hardware nodes are running**  
   - Right arm: `/openarmx_right_hardware_params`  
   - Left arm: `/openarmx_left_hardware_params`  
   The panel reports connected/partially connected/disconnected.
2) **Add the panel in RViz**: `Panels` → `Add New Panel` → `openarmx_kp_kd_panel` → `KpKdPanel`.
3) **Pick control mode**: Right / Left / Both. Mode changes re-check the parameter services.
4) **Adjust sliders & observe mapped values** shown above each group.
5) **Apply**: click “Apply KP/KD to all joints” to send all 8 parameters at once. Status colors indicate success/failure/waiting.
6) **Simulation detection**: if `fake_hardware` is detected (controller_manager without *_hardware_params), the apply button is disabled and a warning is shown.

### Safety notes
- Increase KP/KD gradually to avoid vibration and noise
- Both-arms mode sends the same slider values to left and right
- Apply is blocked when parameter services are unreachable; a dialog explains the issue

### CLI equivalents
```bash
# Set Joint1 KP to 100 (right arm example)
ros2 param set /openarmx_right_hardware_params kp_joint1 100.0

# Inspect all right-arm parameters
ros2 param list /openarmx_right_hardware_params
```

## Author

- **Wei Lindong** (魏林栋)
- Company: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- Website: https://openarmx.com/

## Version

v2.0.0

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

For more details, see the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

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
