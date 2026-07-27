# openarmx_tools

`openarmx_tools` 是 OpenArmX 的工具集合包，主要用于 **调试、示教、参数整定和快速联调**。  
该目录下每个子包都可独立编译与使用，覆盖了从 RViz 可视化控制到轨迹录制回放的常见工程需求。

> ⚠️ 建议先确认机器人控制器已正常启动，再使用本目录工具进行联调或示教。

## 🧰 包含哪些工具

1. `openarmx_joint_slider_panel`
- RViz2 关节滑块面板（双臂 + 双夹爪）。
- 适合快速调姿、演示、联调。
- 支持分段步进执行，减少大跳变带来的运动冲击。

![openarmx_joint_slider_panel 演示](./assets/openarmx_joint_slider_panel.gif)

2. `openarmx_gripper_panel`
- RViz2 夹爪控制面板。
- 支持左夹爪、右夹爪或双夹爪同步控制。
- 基于 `GripperCommand` action 下发命令。

![openarmx_gripper_panel 演示](./assets/openarmx_gripper_panel.gif)

3. `openarmx_kp_kd_panel`
- RViz2 的 KP/KD 参数调节面板。
- 可对手臂/夹爪进行实时刚度与阻尼调整。
- 支持左右臂或双臂模式，适用于实机参数整定。

![openarmx_kp_kd_panel 演示](./assets/openarmx_kp_kd_panel.gif)

4. `openarmx_ik_control_panel`
- RViz2 中用于切换 VR 遥操作 IK 强制启用状态的面板。
- 向 `/openarmx_teleop_vr/ik_enable_override` 发布可保持的启用状态。
- 默认关闭，避免机器人启动后立即进入 IK 强制启用状态。

5. `openarmx_teach`
- 轨迹示教工具（录制 + 回放）。
- 从 `/joint_states` 录制 YAML 轨迹，并回放到双臂与夹爪控制器。
- 支持关节筛选、速率缩放、夹爪同步策略。

![openarmx_teach 演示](./assets/openarmx_teach.gif)

## 🚀 推荐使用顺序（典型实机流程）

1. 启动机器人底层（bringup/moveit，控制器在线）
2. 用 `openarmx_kp_kd_panel` 调整到合适刚度
3. 用 `openarmx_joint_slider_panel` 或 `openarmx_gripper_panel` 进行动作联调
4. 用 `openarmx_teach` 进行轨迹录制与回放验证

> ✅ 按上述顺序操作，通常可以显著降低“有话题但不动作”的排查成本。

## 🔧 快速编译

在工作空间根目录执行：

```bash
colcon build --packages-select \
  openarmx_joint_slider_panel \
  openarmx_gripper_panel \
  openarmx_kp_kd_panel \
  openarmx_ik_control_panel \
  openarmx_teach
source install/setup.bash
```

## 📚 文档入口

- `openarmx_joint_slider_panel/README_CN.md`
- `openarmx_gripper_panel/README_CN.md`
- `openarmx_kp_kd_panel/README_CN.md`
- `openarmx_ik_control_panel/README.md`
- `openarmx_ik_control_panel/README_CN.md`
- `openarmx_teach/README_CN.md`

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
