# OpenArmX IK 控制面板

**中文** | [English](README.md)

## 概述

`openarmx_ik_control_panel` 是一个 RViz2 面板插件，用于切换 `openarmx_teleop_vr` 使用的手动 IK 强制启用状态。

这个面板本身不是 IK 求解器，也不会直接发布机械臂关节命令。它只发布一个布尔覆盖量，用于决定 VR 遥操作节点是否可以在手柄 grip 值未超过配置阈值时仍保持相对 IK 控制启用。

## 工作方式

- 除非 RViz 配置明确恢复其他值，否则面板初始状态为 `OFF`。
- 点击“ik逆解开始”后，覆盖状态切换为 `ON`。
- 点击“ik逆解结束”后，覆盖状态切换回 `OFF`。
- `ON`：VR 节点不再依赖 grip 阈值，持续认为手动 IK 覆盖已启用。
- `OFF`：VR 节点恢复正常的 grip 阈值判断方式。
- 当前状态会以 `ik_enabled` 字段保存在 RViz 配置中。

开启覆盖状态本身不会产生运动。机械臂是否运动仍取决于有效的 VR 位姿输入，以及 VR 遥操作节点和控制器是否正常运行。

## ROS 2 接口

| 方向 | 话题 | 类型 | QoS |
| --- | --- | --- | --- |
| 发布 | `/openarmx_teleop_vr/ik_enable_override` | `std_msgs/msg/Bool` | Reliable、Transient Local、深度 1 |

`openarmx_teleop_vr` 的默认配置订阅同一话题，并使用相匹配的 QoS 设置。

## 编译

在工作空间根目录执行：

```bash
cd ~/openarmx/openarmx_ws_new
colcon build --packages-select openarmx_ik_control_panel --symlink-install
source install/setup.bash
```

## 使用方法

### 随 O6 Bringup 启动

O6 的 RViz 配置已经默认加载该面板，并将初始状态设为关闭：

```bash
ros2 launch openarmx_hand_bringup openarmx.bimanual.o6.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=true
```

### 在 RViz2 中手动添加

1. 编译该包并加载工作空间环境。
2. 打开 RViz2。
3. 选择 `Panels` -> `Add New Panel`。
4. 选择 `openarmx_ik_control_panel/IkControlPanel`。

## 验证方法

查看面板发布的状态：

```bash
ros2 topic echo /openarmx_teleop_vr/ik_enable_override std_msgs/msg/Bool
```

检查 ROS 2 是否能够找到该包：

```bash
ros2 pkg prefix openarmx_ik_control_panel
```

如果 RViz 中找不到该面板，请重新编译该包，并确保启动 RViz 的终端已经执行 `source install/setup.bash`。

## 作者

- 公司: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 网站: https://openarmx.com/

## 版本

**当前版本**：6.0.0

## 致谢

本包是 OpenArmX 机器人平台生态系统的一部分，专为协作机器人领域的研究和工业应用而开发。

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | <https://openarmx.com/> |
| 🌐 文档 | <http://docs.openarmx.com/> |
| 📍 地址 | 天津经济技术开发区西区新业八街11号华诚机械厂 |
| 👤 联系人 | 王先生 |
