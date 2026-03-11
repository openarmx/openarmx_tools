# openarmx_teach | 轨迹录制与回放工具

[English](README.md) | 简体中文

本包提供**录制**与**回放**两类脚本，面向 OpenArmX 双臂 + 双夹爪场景。录制阶段从 `/joint_states` 采样生成统一 YAML；回放阶段自动按关节命名拆分，分别下发到左右臂的 `FollowJointTrajectory` 与夹爪的 `GripperCommand` Action，可选基于反馈的夹爪同步。

---

## 功能概览
- **长时间录制**：`record_joint_states_always` 按频率采样 `/joint_states`，手动开始/暂停/保存。
- **多控制器并行回放**：`play_joint_trajectory` 自动分组左右臂与左右夹爪，同时执行。
- **关节过滤**：支持指定关节列表、左臂、右臂、双臂或全部关节。
- **速率缩放**：`--rate-scale` 让轨迹整体加速/减速。
- **夹爪智能调度**：位置均值映射为标量，去噪/压缩微小变化；可用臂的反馈时间触发夹爪动作 (`--sync-feedback`/`--sync-margin`)。
- **命名约定自动识别**：默认依赖 `openarmx_left_joint*` / `openarmx_right_joint*` / `openarmx_left_finger*` / `openarmx_right_finger*`。

## 依赖与构建
工作区内执行：
```bash
colcon build --packages-select openarmx_teach
source install/setup.bash
```
需要：`rclpy`、`control_msgs`、`trajectory_msgs`、`PyYAML` 等（随 ROS 2/依赖包安装）。

## 典型流程
1. 启动硬件/仿真和对应控制器，如 bringup 或 moveit。
2. 打开一个新终端进行录制：进入动作 → `SPACE` 开始 → `SPACE` 暂停 → `w` 保存。
3. 回放：先 `--all-joints` 验证，再按需要过滤到单臂或单夹爪；必要时加 `--sync-feedback` 提升夹爪同步。
4. 若出现 “Warning: Joint 'xxx' not found”，检查 YAML 的 `joint_names` 与当前控制器关节名是否一致。

## 第一步：启动 Moveit 或 bringup
```bash
# 开启 can 通道
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

# 启动 Moveit
ros2 launch openarmx_bimanual_moveit_config demo.launch.py

# 启动 bringup
ros2 launch openarmx_bringup openarmx.bimanual.launch.py
```

## 第二步：调整 pid 

使用我们的 KP KD 调整面板即可轻松调整机器人的 pid ，以降低机器人硬度，方便后续录制。

点击按钮 **Panels** --> **Add New Panel** --> **KPKDPanel** --> **OK**

将所有关节的 KP KD 均调整到零（默认就是 0 ，因此可以不用任何操作）

点击按钮 **应用KP/KD到所有关节**（绿色按钮）即可完成 pid 的设置

## 第三步：录制：record_joint_states_always
命令示例：
```bash
ros2 run openarmx_teach record_joint_states_always --rate 20
# 自定义输出名：
ros2 run openarmx_teach record_joint_states_always --rate 10 --outfile demo.yaml
```
默认输出名：`joint_states_stream_YYYYMMDD_HHMMSS.yaml`。采样频率 `--rate`（Hz）决定 `time_from_start` 递增步长（`(i+1)*dt`）。

键盘控制：
- `SPACE` / `p`：开始/暂停
- `c`：清空当前缓存（需确认）
- `w`：保存后退出（需确认）
- `q`：不保存退出

注意：`joint_names` 取首条消息的顺序，之后每帧按此顺序补齐。

## 第四步：调整 pid

在完成了录制步骤的情况下即可进行后续的播放环节，但是上一步已经将 pid 调整到了 0 ，因此如果直接播放机器人不会作任何操作（因为电机没力）

那么我们需要将 pid 调整回默认状态或更高（具有更高的硬度，机器人会更接近录制的路径）

点击所有的 **恢复默认** 按钮然后再点击按钮 **应用KP/KD到所有关节**（绿色按钮）即可完成 pid 的设置，接下来就可以进行播放了

**注意：在 pid 较小时（包括默认kp kd，为保证安全，机器人的默认kp kd设置的比较小）机器人并不会特别精准地沿着录制的路径运行，因为电机的力气不够，可以尝试调大 pid，或者直接使用位置模式（更加精准的模式，但机器人负载较低（大概3kg），在作负载运动时可能损坏电机）**

## 回放：play_joint_trajectory

多控制器（默认）：
```bash
ros2 run openarmx_teach play_joint_trajectory <record.yaml> --all-joints --rate-scale 0.5
```
默认使用的 action 名称：
- 左臂：`/left_joint_trajectory_controller/follow_joint_trajectory`
- 右臂：`/right_joint_trajectory_controller/follow_joint_trajectory`
- 左夹爪：`/left_gripper_controller/gripper_cmd`
- 右夹爪：`/right_gripper_controller/gripper_cmd`

常用过滤/调度参数：
- `--left-arm` / `--right-arm` / `--both-arms` / `--all-joints`
- `--joints <list>`：自定义关节子集
- `--rate-scale f`：时间缩放（>1 加速，<1 减速）
- `--sync-feedback`：用臂反馈时间驱动夹爪调度
- `--sync-margin m`：反馈时间 + m ≥ 目标时间即触发夹爪，可实现略提前
- `--action <name>`：单控制器模式；若名称包含 `gripper` 则发送 `GripperCommand`

单控制器示例（仅左夹爪）：
```bash
ros2 run openarmx_teach play_joint_trajectory <record.yaml> \
  --action /left_gripper_controller/gripper_cmd \
  --joints openarmx_left_finger_joint1 openarmx_left_finger_joint2
```

## YAML 格式示例
```yaml
joint_names: [openarmx_left_joint1, openarmx_left_joint2, ...]
points:
  - positions: [0.1, 0.2, ...]
    time_from_start: 0.1   # 秒，按录制频率递增
```

## 使用注意
- 录制的 `time_from_start` 由采样频率推导，并非真实执行时间戳；若控制器启动/延迟较大，需适当放宽 `--sync-margin` 或调整 `--rate-scale`。
- 录制频率过低会让轨迹稀疏；过高会增大文件且对夹爪意义有限。
- 夹爪发送前会压缩微小/过密变化；保持清晰的开合动作更易重现。
- 回放前确保 action server 已启动且名称匹配，否则会直接失败。

## 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE_CN.md](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 作者

- **Zhang Li** (张力)
- 公司: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 网站: https://openarmx.com/

## 版本

**当前版本**：1.0.0

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | <https://openarmx.com/> |
| 📍 地址 | 天津经济技术开发区西区新业八街11号华诚机械厂 |
| 👤 联系人 | 王先生 |