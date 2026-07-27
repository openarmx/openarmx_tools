# openarmx_teach | 轨迹录制与回放工具

[English](README.md) | 简体中文

本包提供**录制**与**回放**两类脚本，面向 OpenArmX 双臂 + 双夹爪场景。录制阶段从 `/joint_states` 采样生成统一 YAML；回放阶段自动按关节命名拆分，分别下发到左右臂的 `FollowJointTrajectory` 与夹爪的 `GripperCommand` Action，支持多机器人同步广播。

---

## 功能概览
- **长时间录制**：`record_joint_states_always` 按频率采样 `/joint_states`，手动开始/暂停/保存，支持命名空间。
- **多控制器并行回放**：`play_joint_trajectory` 自动分组左右臂与左右夹爪，同时执行。
- **多机器人同步广播**：`--arm-prefix robot1 robot2 ...` 一条命令同时驱动多台机器人，通过同一 `asyncio.gather` 保证同步触发。
- **关节过滤**：支持指定关节列表、左臂、右臂、双臂或全部关节。
- **速率缩放**：`--rate-scale` 让轨迹整体加速/减速。
- **夹爪智能调度**：位置均值映射为标量，去噪/压缩微小变化；可用臂的反馈时间触发夹爪动作 (`--sync-feedback`/`--sync-margin`)。

## 依赖与构建
```bash
colcon build --packages-select openarmx_teach
source install/setup.bash
```

---

## 第一步：开启 CAN 通道

每次开机后需要初始化 CAN 接口：

```bash
sudo ip link set can0 down && sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
sudo ip link set can1 down && sudo ip link set can1 type can bitrate 1000000 && sudo ip link set can1 up
# 多机器人时继续初始化 can2、can3...
sudo ip link set can2 down && sudo ip link set can2 type can bitrate 1000000 && sudo ip link set can2 up
sudo ip link set can3 down && sudo ip link set can3 type can bitrate 1000000 && sudo ip link set can3 up
```

---

## 第二步：启动机器人

### 单机器人（不带重力补偿）
```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can0 left_can_interface:=can1 \
    control_mode:=mit
```

### 单机器人（带重力补偿）
重力补偿可消除 MIT 模式下因重力引起的约 6° 稳态误差，推荐在精度要求较高时开启。
```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can0 left_can_interface:=can1 \
    control_mode:=mit \
    enable_forward_effort:=true
```

### 多机器人（不带重力补偿）
每台机器人在独立终端启动，通过 `arm_prefix` 区分命名空间：
```bash
# 机器人 1（终端 1）
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can0 left_can_interface:=can1 \
    arm_prefix:=robot1 control_mode:=mit

# 机器人 2（终端 2）
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can2 left_can_interface:=can3 \
    arm_prefix:=robot2 control_mode:=mit
```

### 多机器人（带重力补偿）
```bash
# 机器人 1
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can0 left_can_interface:=can1 \
    arm_prefix:=robot1 control_mode:=mit enable_forward_effort:=true

# 机器人 2
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can2 left_can_interface:=can3 \
    arm_prefix:=robot2 control_mode:=mit enable_forward_effort:=true
```

---

## 第三步：调节 KP/KD（示教前必做）

示教录制前需将 KP/KD 调为 0，使机器人进入零阻抗状态，方便手动拖动。

在 RViz 中添加面板：**Panels → Add New Panel → KPKDPanel → OK**

### 单机器人
命名空间输入框**留空**，选择"双臂"，将 KP/KD 滑轨拖到最左（0），点击**应用KP/KD到所有关节**。

### 多机器人
在命名空间输入框填入对应机器人的前缀（如 `robot1`），分别对每台机器人操作。

### 关于前馈补偿与示教
启用了重力补偿（`enable_forward_effort:=true`）时，KP/KD 置零后机器人拖动会过于"跟手"。可在不重启节点的情况下临时关闭前馈：

```bash
# 单机器人
ros2 param set /gravity_comp_node enable_compensation false

# 多机器人（robot1）
ros2 param set /robot1/gravity_comp_node enable_compensation false
```

示教完成后恢复：
```bash
ros2 param set /gravity_comp_node enable_compensation true
# 或
ros2 param set /robot1/gravity_comp_node enable_compensation true
```

---

## 第四步：录制动作

### 单机器人
```bash
ros2 run openarmx_teach record_joint_states_always --rate 10 --outfile demo.yaml
```

### 多机器人（指定前缀）
```bash
# 录制 robot1 的动作
ros2 run openarmx_teach record_joint_states_always --arm-prefix robot1 --rate 10 --outfile demo_robot1.yaml
```

键盘控制：

| 按键 | 功能 |
|------|------|
| `SPACE` / `p` | 开始 / 暂停录制 |
| `c` | 清空缓存（需确认） |
| `w` | 保存并退出（需确认） |
| `q` | 不保存退出 |

录制参数：

| 参数 | 说明 |
|------|------|
| `--rate` | 采样频率（Hz），决定轨迹时间步长，默认 10 |
| `--outfile` | 输出文件路径，默认自动生成带时间戳的文件名 |
| `--arm-prefix` | 机器人命名空间前缀（多机器人时使用，如 `robot1`） |
| `--topic` | 直接指定话题名，优先级高于 `--arm-prefix` |

---

## 第五步：恢复 KP/KD

录制完成后，播放前需将 KP/KD 恢复，否则电机无力无法执行轨迹。

在 KpKdPanel 中点击所有**恢复默认**按钮，再点击**应用KP/KD到所有关节**。

> 默认 KP/KD 较小以保证安全。若需要更精准地跟随录制路径，可适当调大 KP/KD，或切换为 CSP 位置模式（`control_mode:=csp`，精度更高但负载约 3 kg，超载可能损坏电机）。

---

## 第六步：播放动作

### 单机器人
```bash
ros2 run openarmx_teach play_joint_trajectory demo.yaml --all-joints
```

### 多机器人同步播放（一个终端，同时触发）
```bash
ros2 run openarmx_teach play_joint_trajectory demo.yaml --all-joints --arm-prefix robot1 robot2
```

所有机器人的轨迹通过同一个 `asyncio.gather` 同时发出，保证多机器人动作整齐同步。

### 常用参数

| 参数 | 说明 |
|------|------|
| `--all-joints` | 自动分组所有关节，同时发给左右臂和夹爪 |
| `--left-arm` | 仅播放左臂 |
| `--right-arm` | 仅播放右臂 |
| `--both-arms` | 仅播放双臂（不含夹爪） |
| `--joints <list>` | 自定义关节子集 |
| `--arm-prefix` | 命名空间前缀，支持多个（如 `robot1 robot2`） |
| `--rate-scale` | 速率缩放，>1 加速，<1 减速，默认 1.0 |
| `--sync-feedback` | 用臂轨迹反馈时间驱动夹爪调度，提升同步精度 |
| `--sync-margin` | 反馈时间 + margin ≥ 目标时间即触发夹爪（秒） |
| `--action` | 单控制器模式，直接指定 action 名称 |

### 播放示例

```bash
# 减速到 50% 播放，带夹爪同步
ros2 run openarmx_teach play_joint_trajectory demo.yaml \
    --all-joints --rate-scale 0.5 --sync-feedback

# 仅播放左臂
ros2 run openarmx_teach play_joint_trajectory demo.yaml --left-arm

# 三台机器人同步播放
ros2 run openarmx_teach play_joint_trajectory demo.yaml \
    --all-joints --arm-prefix robot1 robot2 robot3
```

---

## YAML 格式

```yaml
joint_names: [openarmx_left_joint1, openarmx_left_joint2, ..., openarmx_right_joint1, ...]
points:
  - positions: [0.1, 0.2, ...]
    time_from_start: 0.1   # 秒，按录制频率递增
  - positions: [0.15, 0.25, ...]
    time_from_start: 0.2
```

---

## 使用注意

- 录制的 `time_from_start` 由采样频率推导，并非真实执行时间戳；若控制器延迟较大，可适当调整 `--rate-scale` 或 `--sync-margin`。
- 多机器人播放时，`--arm-prefix` 的顺序不影响同步性，所有机器人通过同一 `asyncio.gather` 同时触发。
- 录制频率过低会让轨迹稀疏；过高会增大文件且对夹爪意义有限，推荐 10~20 Hz。
- 夹爪发送前会自动压缩微小/过密变化，保持清晰的开合动作更易重现。
- 回放前确保 action server 已启动且名称匹配，否则会超时报错。

---

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

---