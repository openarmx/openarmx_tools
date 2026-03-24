# openarmx_tools

`openarmx_tools` is the OpenArmX utility collection, mainly for **debugging, teaching, parameter tuning, and rapid integration**.  
Each subpackage in this directory can be built and used independently, covering common engineering workflows from RViz visual control to trajectory recording and playback.

> ⚠️ It is recommended to confirm robot controllers are running normally before using these tools for tuning or teaching.

## 🧰 Included Tools

1. `openarmx_joint_slider_panel`
- RViz2 joint slider panel (dual-arm + dual-gripper).
- Suitable for quick pose adjustment, demos, and integration testing.
- Supports segmented step execution to reduce motion shock from large jumps.

2. `openarmx_gripper_panel`
- RViz2 gripper control panel.
- Supports left gripper, right gripper, or synchronized dual-gripper control.
- Sends commands via `GripperCommand` action.

3. `openarmx_kp_kd_panel`
- RViz2 KP/KD parameter tuning panel.
- Enables real-time stiffness and damping adjustment for arm and gripper.
- Supports right-arm, left-arm, or dual-arm mode for real hardware tuning.

4. `openarmx_teach`
- Trajectory teaching tools (record + playback).
- Records YAML trajectories from `/joint_states`, then replays to arm and gripper controllers.
- Supports joint filtering, rate scaling, and gripper synchronization strategies.

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
  openarmx_teach
source install/setup.bash
```

## 📚 Documentation Entry Points

- `openarmx_joint_slider_panel/README_CN.md`
- `openarmx_gripper_panel/README_CN.md`
- `openarmx_kp_kd_panel/README_CN.md`
- `openarmx_teach/README_CN.md`

## ⚖️ License

Subpackages in this directory follow the licenses declared in the repository (see each subpackage `LICENSE` and `README`).
