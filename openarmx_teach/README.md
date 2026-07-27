# openarmx_teach | Trajectory Recording & Playback

English | [简体中文](README_CN.md)

This package ships two scripts for OpenArmX dual arms + dual grippers: a **recorder** that samples `/joint_states` into a single YAML file, and a **player** that auto-splits that YAML into left/right arm `FollowJointTrajectory` goals plus left/right gripper `GripperCommand` goals. Gripper timing can follow arm feedback for better synchronization.

---

## Highlights
- **Continuous recording**: `record_joint_states_always` samples `/joint_states` at a fixed rate with interactive start/pause/save.
- **Multi-controller playback**: `play_joint_trajectory` groups joints by name and drives arms and grippers simultaneously.
- **Joint filtering**: choose a custom list, left only, right only, both arms, or all joints.
- **Rate scaling**: `--rate-scale` speeds up or slows down the whole motion.
- **Gripper scheduling**: averages finger joints to a scalar, compresses tiny/close points, and can trigger by arm feedback time (`--sync-feedback` / `--sync-margin`).
- **Name-based grouping**: expects joint prefixes `openarmx_left_joint*`, `openarmx_right_joint*`, `openarmx_left_finger*`, `openarmx_right_finger*`.

## Dependencies & Build
Build inside your workspace:
```bash
colcon build --packages-select openarmx_teach
source install/setup.bash
```
Requires `rclpy`, `control_msgs`, `trajectory_msgs`, `PyYAML` (installed with ROS 2/apt).

## Typical workflow
1. Launch hardware/simulation and corresponding controllers (bringup or moveit).
2. Adjust PID: Lower KP/KD to zero for easy manual dragging during recording.
3. Launch a new terminal and start record: move the robot → `SPACE` start → `SPACE` pause → `w` save.
4. Adjust PID: Restore default or higher PID values for accurate playback.
5. Play back: verify with `--all-joints`, then narrow to a specific arm or gripper; add `--sync-feedback` if gripper timing matters.
6. If you see "Warning: Joint 'xxx' not found", ensure `joint_names` match current controller joint names.

## Step 1: Launch Moveit or Bringup
```bash
# Enable CAN interfaces
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

# Launch Moveit
ros2 launch openarmx_bimanual_moveit_config demo.launch.py

# Or launch bringup
ros2 launch openarmx_bringup openarmx.bimanual.launch.py
```

## Step 2: Adjust PID (Before Recording)

Use our KP KD adjustment panel to easily adjust the robot's PID to reduce stiffness for easier recording.

Click button **Panels** --> **Add New Panel** --> **KPKDPanel** --> **OK**

Adjust all joints' KP and KD to zero (default is already 0, so no operation needed)

Click the button **应用KP/KD到所有关节** (green button) to apply the PID settings

## Step 3: Recording - record_joint_states_always
Examples:
```bash
ros2 run openarmx_teach record_joint_states_always --rate 20
# Custom output name
ros2 run openarmx_teach record_joint_states_always --rate 10 --outfile demo.yaml
```
Default output: `joint_states_stream_YYYYMMDD_HHMMSS.yaml`. Sampling rate `--rate` (Hz) determines `time_from_start` increment (`(i+1)*dt`).

Keyboard controls:
- `SPACE` / `p`: start / pause toggle
- `c`: clear buffer (confirm)
- `w`: save and quit (confirm)
- `q`: quit without saving

`joint_names` follow the first received message order; each snapshot is reordered to that layout.

## Step 4: Adjust PID (Before Playback)

After completing the recording step, you can proceed to playback. However, since the previous step set PID to 0, the robot won't move during playback (motors have no torque).

We need to restore PID to default or higher values (higher stiffness means the robot will follow the recorded path more closely).

Click all **恢复默认** (Restore Default) buttons, then click **应用KP/KD到所有关节** (green button) to apply the PID settings. Now you can proceed with playback.

**Note: With low PID values (including default KP/KD, which are set relatively low for safety), the robot won't follow the recorded path very precisely because the motors don't have enough torque. You can try increasing the PID, or use position mode directly (more precise mode, but the robot has lower payload capacity (~3kg), which may damage motors during loaded motion).**

## Step 5: Playback - play_joint_trajectory

Multi-controller (default):
```bash
ros2 run openarmx_teach play_joint_trajectory <record.yaml> --all-joints --rate-scale 0.5
```
Default action names:
- Left arm: `/left_joint_trajectory_controller/follow_joint_trajectory`
- Right arm: `/right_joint_trajectory_controller/follow_joint_trajectory`
- Left gripper: `/left_gripper_controller/gripper_cmd`
- Right gripper: `/right_gripper_controller/gripper_cmd`

Key options:
- `--left-arm` / `--right-arm` / `--both-arms` / `--all-joints`
- `--joints <list>` for a custom subset
- `--rate-scale f` to speed up (>1) or slow down (<1)
- `--sync-feedback` to trigger grippers from arm feedback time
- `--sync-margin m` to allow slightly early gripper triggers
- `--action <name>` single-controller mode; names containing `gripper` use `GripperCommand`

Single-controller example (left gripper only):
```bash
ros2 run openarmx_teach play_joint_trajectory <record.yaml> \
  --action /left_gripper_controller/gripper_cmd \
  --joints openarmx_left_finger_joint1 openarmx_left_finger_joint2
```

## YAML format
```yaml
joint_names: [openarmx_left_joint1, openarmx_left_joint2, ...]
points:
  - positions: [0.1, 0.2, ...]
    time_from_start: 0.1   # seconds, derived from sample rate
```

## Operational notes
- `time_from_start` is generated from the sampling rate, not from real execution timestamps; adjust `--sync-margin` or `--rate-scale` if controllers start slowly or lag.
- Too low sampling makes sparse trajectories; too high grows the file and adds little for grippers.
- Gripper points are compressed; make clear open/close motions when recording.
- Playback fails fast if action servers are not running or names differ from defaults.

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

For more details, see the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Author

- **Zhang Li** (张力)
- Company: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- Website: https://openarmx.com/

## Version

**Current Version**: 1.0.0

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
