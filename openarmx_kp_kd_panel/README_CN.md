# 关节 KP/KD 控制面板

[English](README.md) | 简体中文

### 概述
`openarmx_kp_kd_panel` 是一个 RViz2 面板插件，用滑轨实时调整机器人 8 个关节（7 轴手臂 + 夹爪）的 KP（刚度）与 KD（阻尼）。滑轨值会按电机类型自动映射到真实参数范围，并通过 ROS 2 参数服务一次性下发到硬件。

### 主要特性
- 手臂/夹爪分组滑轨：手臂 (Joint 1-7) 与夹爪 (Joint 8) 各有独立 KP/KD 滑轨与“恢复默认”按钮
- 自动范围映射：RS04/RS03/RS00 电机的 KP/KD 上限自动换算，无需手算比例
- 多控制模式：可选右臂、左臂或双臂，分别连接 `/openarmx_right_hardware_params` 与 `/openarmx_left_hardware_params`
- 实时状态反馈：状态条显示连接情况、仿真检测结果与应用是否成功
- 配置持久化：滑轨与控制模式写入 RViz 配置，重开 RViz 自动恢复
- 适配 UI：面板可滚动，适合小分辨率；按钮/提示颜色区分状态

### 电机与映射范围
| 关节 | 电机类型 | KP 映射范围 | KD 映射范围 |
| --- | --- | --- | --- |
| Joint 1-2 | RS04 | 0–5000 | 0–100 |
| Joint 3-4 | RS03 | 0–5000 | 0–100 |
| Joint 5-8 | RS00 | 0–500  | 0–5 |

滑轨范围：手臂 KP 0–1000、手臂 KD 0–100；夹爪 KP 0–1000、夹爪 KD 0–100。面板实时显示映射后的实际值。

### 默认值（面板“恢复默认”按钮）
- 手臂 KP: 10  → RS04/RS03=50.0，RS00=5.0
- 手臂 KD: 3   → RS04/RS03=3.0，RS00=0.15
- 夹爪 KP: 100 → RS00=50.0
- 夹爪 KD: 50  → RS00=2.5

### 构建与安装
此包为标准 ROS 2 ament 插件，依赖 `rclcpp`、`rviz_common`、`rviz_rendering`、`pluginlib`、`Qt5 Widgets`。在工作区执行：
```bash
colcon build --packages-select openarmx_kp_kd_panel
```
编译后 source 工作区（例如 `source install/setup.bash`），RViz 即可加载插件。

### 使用步骤
1) **确认硬件节点**：确保真实硬件参数节点在运行  
   - 右臂：`/openarmx_right_hardware_params`  
   - 左臂：`/openarmx_left_hardware_params`  
   面板会检测服务是否就绪；若只连上其中一侧会提示“部分连接”。
2) **在 RViz 添加面板**：`Panels` → `Add New Panel` → 选择 `openarmx_kp_kd_panel` → `KpKdPanel`。
3) **选择控制模式**：右臂 / 左臂 / 双臂。模式切换会重新检查参数服务器。
4) **拖动滑轨并查看映射值**：顶部标签实时显示各关节映射后的 KP/KD 数字。
5) **应用**：点击“应用KP/KD到所有关节”一次性下发 8 轴参数。状态条颜色提示成功/失败/等待。
6) **仿真模式提示**：若检测到 `fake_hardware`（有 controller_manager 但无 *_hardware_params），按钮会自动禁用并提示 KP/KD 不可用。

### 安全提示
- 优先从较小的 KP/KD 开始逐步上调，防止振动与噪声
- 双臂模式下两侧使用同一组滑轨值同时下发
- 若服务未连通，面板会阻止应用并弹窗提示

### 命令行等效示例
```bash
# 将 Joint1 KP 设为 100（右臂示例）
ros2 param set /openarmx_right_hardware_params kp_joint1 100.0

# 查看右臂所有参数
ros2 param list /openarmx_right_hardware_params
```

## 作者

- **魏林栋** (Wei Lindong)
- 公司：成都长数机器人有限公司
- 网站：https://openarmx.com/

## 版本

v1.0.0

## 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE_CN.md](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

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