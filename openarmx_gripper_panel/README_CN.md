# OpenArm 夹爪控制面板

**中文** | [English](README.md)

## 概述

`openarmx_gripper_panel` 是一个用于 OpenArm 机器人系统的 RViz2 插件，用于控制单个或双夹爪。它在 RViz 中提供了一个直观的图形界面，通过 ROS2 动作命令来控制夹爪位置。

## 功能特性

- **双夹爪支持**：可控制左夹爪、右夹爪或同时控制双夹爪
- **直观的图形界面**：基于滑块的位置控制，带有预设快捷按钮
- **实时反馈**：连接状态和命令状态的可视化指示器
- **RViz 集成**：无缝集成为 RViz 面板
- **配置持久化**：保存夹爪选择偏好设置

## 系统架构

### 组件

- **RViz 插件**：基于 Qt5 的 GUI 面板，集成到 RViz2
- **动作客户端**：通过 `control_msgs/action/GripperCommand` 与夹爪控制器通信
- **控制器接口**：
  - 左夹爪：`/left_gripper_controller/gripper_cmd`
  - 右夹爪：`/right_gripper_controller/gripper_cmd`

### 控制范围

- **位置范围**：0-44mm
- **预设值**：
  - 关闭：0mm
  - 半开：22mm
  - 打开：44mm
- **最大力度**：10.0N

## 安装

### 依赖项

本软件包需要以下依赖项：

- ROS2（Humble 或更高版本）
- `rclcpp`
- `rclcpp_action`
- `control_msgs`
- `rviz_common`
- `rviz_default_plugins`
- `pluginlib`
- Qt5（Core、Widgets）

### 编译

```bash
# 导航到工作空间
cd ~/openarmx_ws

# 编译软件包
colcon build --packages-select openarmx_gripper_panel

# 加载工作空间
source install/setup.bash
```

## 使用方法

### 1. 启动 RViz

软件包编译并加载后，夹爪面板会自动在 RViz 中可用。

```bash
# 启动 RViz
ros2 run rviz2 rviz2

# 或使用 moveit 启动
ros2 launch openarmx_bimanual_moveit_config demo.launch.py can_fd:=false
```

### 2. 添加面板到 RViz

1. 在 RViz 中，点击 **Panels** → **Add New Panel**
2. 选择 **openarmx_gripper_panel/GripperPanel**
3. 面板将出现在 RViz 窗口中

### 3. 控制夹爪

1. **选择目标**：从下拉菜单中选择：
   - 左夹爪
   - 右夹爪
   - 双夹爪（同步）

2. **设置位置**：
   - 使用滑块调整位置（0-44mm）
   - 或点击快捷预设按钮：
     - **关闭 (0mm)**：完全闭合位置
     - **半开 (22mm)**：半开位置
     - **打开 (44mm)**：完全打开位置

3. **执行命令**：
   - 点击绿色的**"应用 - 执行命令"**按钮
   - 状态标签会显示命令反馈

### 4. 状态指示器

面板显示实时状态信息：

- 绿色：命令发送成功 / 控制器就绪
- 黄色：一个夹爪控制器未连接
- 红色：所有夹爪控制器未连接
- 灰色：就绪/空闲状态

## 技术细节

### ROS2 接口

**动作类型**：`control_msgs/action/GripperCommand`

**动作目标结构**：
```cpp
goal_msg.command.position = position;  // 目标位置（米）(0.0-0.044)
goal_msg.command.max_effort = 10.0;    // 最大力度（牛顿）
```

### 话题和服务

插件连接到以下动作服务器：
- `/left_gripper_controller/gripper_cmd` (control_msgs/action/GripperCommand)
- `/right_gripper_controller/gripper_cmd` (control_msgs/action/GripperCommand)

### 同步控制

当选择"双夹爪（同步）"时：
1. 首先发送命令到右夹爪
2. 延迟 5 毫秒
3. 发送命令到左夹爪
4. 这确保了近乎同时的执行，时间偏移最小

### 配置

面板配置会自动保存在 RViz 配置文件（.rviz）中：
- 夹爪选择偏好
- 面板位置和大小

## 插件注册

插件通过 pluginlib 机制注册到 RViz：

```xml
<library path="openarmx_gripper_panel">
  <class name="openarmx_gripper_panel/GripperPanel"
         type="openarmx_gripper_panel::GripperPanel"
         base_class_type="rviz_common::Panel">
    <description>
      OpenArm夹爪控制面板 - 在RViz中控制单个或双个夹爪的开合
    </description>
  </class>
</library>
```

## 故障排除

### 面板未出现在 RViz 中

```bash
# 确保软件包已编译
colcon build --packages-select openarmx_gripper_panel

# 加载工作空间
source install/setup.bash

# 检查插件注册
ros2 pkg prefix openarmx_gripper_panel

# 验证插件描述文件
cat install/openarmx_gripper_panel/share/openarmx_gripper_panel/plugins/plugin_description.xml
```

### 控制器连接失败

1. 验证夹爪控制器正在运行：
```bash
ros2 action list | grep gripper
```

预期输出：
```
/left_gripper_controller/gripper_cmd
/right_gripper_controller/gripper_cmd
```

2. 检查控制器状态：
```bash
ros2 action info /left_gripper_controller/gripper_cmd
ros2 action info /right_gripper_controller/gripper_cmd
```

3. 手动测试动作：
```bash
ros2 action send_goal /left_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.022, max_effort: 10.0}}"
```

### 面板显示警告信息

- **"警告：夹爪控制器未连接"**：两个控制器都不可用
  - 检查机器人硬件是否连接
  - 验证控制器是否已启动

- **"警告：左夹爪控制器未连接"**：仅左控制器不可用
  - 右夹爪仍可控制

- **"警告：右夹爪控制器未连接"**：仅右控制器不可用
  - 左夹爪仍可控制

## 开发

### 文件结构

```
openarmx_gripper_panel/
├── CMakeLists.txt              # 编译配置
├── package.xml                 # 软件包元数据
├── README.md                   # 英文文档
├── README_zh.md                # 中文文档
├── include/
│   └── openarmx_gripper_panel/
│       └── gripper_panel.hpp   # 头文件
├── src/
│   └── gripper_panel.cpp       # 实现文件
├── plugins/
│   └── plugin_description.xml  # 插件注册
└── resource/
    └── openarmx_gripper_panel  # 资源标记
```

### 关键类

**GripperPanel**：主面板类，继承自 `rviz_common::Panel`
- 用户交互的 Qt 槽函数
- 夹爪控制的 ROS2 动作客户端
- 配置保存/加载功能

### 从源代码编译

```bash
# 克隆仓库（如果工作空间中还没有）
cd ~/openarmx_ws/src/openarmx_tools/

# 安装依赖项
rosdep install --from-paths . --ignore-src -r -y

# 编译
cd ~/openarmx_ws
colcon build --packages-select openarmx_gripper_panel --cmake-args -DCMAKE_BUILD_TYPE=Release

# 加载并测试
source install/setup.bash
ros2 run rviz2 rviz2
```

## 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE_CN.md](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 作者

- **魏林栋** (Wei Lindong)
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