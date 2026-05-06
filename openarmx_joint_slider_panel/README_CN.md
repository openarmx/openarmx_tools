# OpenArmX 关节滑块面板

## 包简介

`openarmx_joint_slider_panel` 是一个 RViz2 面板插件，用于通过滑块直接控制 OpenArmX 双臂机器人的关节与夹爪。

该插件面向调试、演示和快速联调场景：在 RViz2 中即可进行双臂关节与夹爪目标值调节，并通过分段执行机制将大幅度目标变化拆分为小步进命令，以获得更平滑、可控的运动过程。

## 功能特性

- 提供 16 个滑块：左右机械臂关节（7 + 7）以及左右夹爪。
- 面板启动后自动进入 `/joint_states` 实时同步显示，无需手动同步。
- 滑块拖动即发命令，无需额外点击“应用”按钮。
- 支持后台线程分段执行：
  - `Joint Step`：控制每周期机械臂最大步进（mrad/cycle）。
  - `Gripper Step`：控制每周期夹爪最大步进（mm/cycle）。
  - 大幅滑块跳变会自动拆分为多次小步命令。
- 不启用预览模型（Preview model disabled）。
- `Hands Up` 按钮可将双臂切换到举手预设位（双臂 joint4=1.8 rad，其余为 0）。
- `Home` 按钮可将双臂与夹爪目标置零，并按分段模式执行回零。
- 仅支持前向位置控制后端：
  - `/left_forward_position_controller/commands`
  - `/right_forward_position_controller/commands`

## 编译

```bash
cd ~/openarmx_ws2
colcon build --packages-select openarmx_joint_slider_panel
source install/setup.bash
```

## RViz2 使用方法

1. 启动机器人系统（`demo.launch.py` / `demo_sim.launch.py` / bringup）。
2. 打开 RViz2。
3. 进入 `Panels` -> `Add New Panel` -> `openarmx_joint_slider_panel/JointSliderPanel`。
4. 根据期望平滑度设置 `Joint Step` 与 `Gripper Step`。
5. 拖动滑块后，机器人将按分段命令直接执行运动。
6. 点击 `Hands Up` 可执行举手预设动作。
7. 点击 `Home` 可执行分段回零。

**或者直接使用openarmx_preview_bringup一键启动**

## 典型控制器话题映射

- `/left_forward_position_controller/commands`
- `/right_forward_position_controller/commands`


## 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE_CN.md](LICENSE_CN.md) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 作者

- **李青燃** (Li QingRan)
- 公司：成都长数机器人有限公司
- 网站：https://openarmx.com/

## 版本

1.0.0

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
