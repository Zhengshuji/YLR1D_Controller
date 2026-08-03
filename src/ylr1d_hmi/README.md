# ylr1d_hmi

基于 Qt5 的人机交互界面，用于 YLR1D 机器人调试。按职责拆分为三个独立节点：

| 节点 | 可执行文件 | Launch 文件 | 面向层级 | 输出 |
|------|-----------|-------------|----------|------|
| **控制层 HMI** | `ylr1d_hmi_control` | `hmi.launch.py` | `ylr1d_control_sim` | `/desired_joint_states` |
| **传感器 HMI** | `ylr1d_hmi_sensor` | `sensor_panel.launch.py` | 各传感器话题 | 只读观测 |
| **传输层 HMI** | `ylr1d_hmi_translate` | `hmi_translate.launch.py` | `ylr1d_translate` | action goal |

代码结构：节点入口直接放 `src/`，面板实现放 `src/panels/`，头文件放
`include/ylr1d_hmi/panels/`（参考 `ylr1d_control_sim` 的文件管理原则，不照搬其目录名）。

---

## 控制层 HMI（ylr1d_hmi_control）— 界面布局

观测 `/joint_states`，向控制层发送 `/desired_joint_states`。

```
┌──────────────────────────────────────────────────┐
│  Toolbar: [Units: rad|m] [Rate: 5 Hz] [● Auto] [Send Now]│
│  [Chassis] [Torso] [Left Arm] [Right Arm]  ← 标签页 │
│  ╔═ Chassis (8) ══════════════════════════════╗   │
│  ║ Observe                                    ║   │
│  ║  RF_Steer   0.000 rad   0.000 rad/s        ║   │
│  ║  RF_Wheel   0.000 rad   0.000 rad/s        ║   │
│  ║  ...                                       ║   │
│  ║ ────────────────────────────────────────── ║   │
│  ║ Control                                    ║   │
│  ║  RF_Steer  [========]  0.000 rad           ║   │
│  ║  RF_Wheel  [========]  0.000 rad/s         ║   │
│  ║  ...                                       ║   │
│  ╚════════════════════════════════════════════╝   │
└──────────────────────────────────────────────────┘
   状态栏: ● /joint_states · 2 pub · 123 msg | ● /desired_joint_states · 1 sub · sent 45 | Send: auto 5 Hz
```

### 1. 布局（四标签页）
- 主区域为 `QTabWidget`，四个标签页：**Chassis**（蓝）、**Torso**（绿）、**Left Arm**（橙）、**Right Arm**（紫）
- 每个标签页只显示一个板块，避免单窗口信息过载；标签页标题即板块名（英文，避免 WSL 中文字体乱码）
- 板块内仍以彩色卡片区分

### 2. 标签页内部：上观测 / 下控制
- **观测区（上方，只读表格）**：本组一个 `QTableWidget`，每行 `[名称] [Position] [Velocity]`
  - 值后带单位：**平移关节（Lift、Finger）显示 `m` / `m/s`，旋转关节显示 `rad` / `rad/s`**
  - 50Hz 刷新（与 ROS spin 定时器同步），订阅 `/joint_states`
- **控制区（下方，可交互）**：每关节一行 `[名称] [Slider] [SpinBox]`
  - 位置关节：Slider 范围对应关节限位，SpinBox 显示 3 位小数并带单位后缀
  - 速度关节（轮子）：Slider ±5.0，单位 `rad/s`
  - Slider / SpinBox 双向同步，操作只打"待发送"标记

### 3. 状态栏
- 左侧：`● /joint_states` 发布者数 + 累计接收帧数（绿点=有发布者，红点=无）
- 中部：`● /desired_joint_states` 订阅者数 + 累计发送次数（绿点=有订阅者）
- 右侧：当前发送模式 `Send: auto 5 Hz`

### 4. 发送方式（GUI 触发 + 低频合并发送）
- 拖动 Slider / 修改 SpinBox **不立即发布**，仅置脏标记
- 由发送定时器（默认 5 Hz / 200 ms）合并发送一次到 `/desired_joint_states`
- 避免高频拖动时对话题洪泛

### 5. 工具栏
- **Units**：角度单位切换 `rad / deg`，长度单位切换 `m / mm`（仅影响显示，内部存储保持 SI）
- **Rate**：发送频率选择 `1 / 2 / 5 / 10 / 20 Hz`（改变发送定时器间隔）
- **Auto**：持续发送总开关（`● Auto: ON` / `○ Auto: OFF` 带指示）；关闭后停止定时发送，仅剩手动发送
- **Send Now**：手动发送按钮，立即发布当前期望值一次

---

## 传输层 HMI（ylr1d_hmi_translate）— 界面布局

向 `ylr1d_translate` 发送 action goal，由转译层再转成控制层命令。
**本节点的输出不给控制层，而是给传输层。**

```
┌──────────────────────────────────────────────────┐
│  [Chassis] [Arm] [Gripper]  ← 标签页             │
│  ╔═ Chassis ═══════════════════════════════════╗ │
│  ║ Mode: [平移|旋转|停车]                       ║ │
│  ║ Direction: [      ] rad   Speed: [     ]    ║ │
│  ║ Duration: [      ] s   [ 发送 ]             ║ │
│  ║ Status: ● ready                             ║ │
│  ╚═════════════════════════════════════════════╝ │
└──────────────────────────────────────────────────┘
```

### 1. Chassis 标签页（action: `/chassis_move`）
- **Mode**：`平移 / 旋转 / 停车`（对应 action 常量 `MODE_TRANSLATE=0 / MODE_ROTATE=1 / MODE_STOP=2`）
- **Direction**：平移时的运动方向角（rad）；旋转/停车忽略
- **Speed**：平移=轮速（rad/s），旋转=车体角速度（rad/s）
- **Duration**：执行时长（s），0=持续执行直到被新目标替换/取消
- 点击发送即 `async_send_goal`；状态栏显示服务端连接、goal 接受、feedback 阶段（steering/moving/stopped）与 result

### 2. Arm 标签页（action: `/arm_move`）
- **Part**：`躯干 / 左臂 / 右臂`（对应 `PART_TORSO=0 / PART_LEFT_ARM=1 / PART_RIGHT_ARM=2`）
- 选中部位后，用 `QStackedWidget` 切换显示该部位的关节滑杆行（躯干 4 个，左右臂各 7 个）
- **机械臂与夹爪解耦**：左右臂面板只含臂关节（Arm1~Arm7），**不含夹爪两指**；夹爪由 Gripper 标签页单独控制
- 关节名称/顺序与 `ylr1d_translate` 的关节表完全一致，`positions` 按此顺序打包
- 点击发送即发送该部位全部关节的期望位置；状态栏显示 feedback 进度与 result

### 3. Gripper 标签页（action: `/gripper_move`）
- **Part**：`左夹爪 / 右夹爪`（对应 `GRIPPER_LEFT=0 / GRIPPER_RIGHT=1`）
- **Open**：`开 / 关` 布尔量（对应 `GRIPPER_OPEN / GRIPPER_CLOSE`）
- 语义由转译层解算：左夹爪开→两指 -0.014，关→0；右夹爪开→两指 +0.014，关→0（左右不对称）

> **action 语义详见** `ylr1d_translate/action/*.action` 文件头注释，本面板的常量与输出顺序与之对齐。

---

## 传感器 HMI（ylr1d_hmi_sensor）

只读观测面板，订阅机器人全部传感器话题并分别以最合适的形式展示：

| 传感器 | 数量 | 展示形式 |
|--------|------|----------|
| 相机（RGB / 深度 / 红外） | 3 台 | 图像 + camera_info 参数；WSL 软渲染下仅渲染选中的相机 |
| 深度相机点云 | 3 台 | 2D 投影 |
| 雷达 | 1 | 360° 极坐标激光扫描视图 + 统计 |
| 超声波 | 4 | 窄扇形激光扫描视图 + 统计 |
| IMU | 1 | 数值读出 |

每个视图都带 `TopicStatus` 行（收到帧数 · 距上次更新秒数 · 估算频率），
独立于渲染判断话题是否存活。

---

## 关节限位

限位以 `ylr1d_control_sim/config/position_control_limits.yaml` 为基准（Finger 按需求取 0.015）：

| 分组 | 关节数 | 位置限位 | 单位 | 说明 |
|------|--------|----------|------|------|
| 底盘-转向 | 4 | ±3.14 | rad | 全向转向 |
| 底盘-轮子 | 4 | ±5.0 | rad/s | 速度控制，连续旋转 |
| 躯干 | 4 | Lift: ±0.30 / Yaw: ±3.14 / Pitch1/2: ±1.57 | Lift 为 m，其余 rad | |
| 左臂 | 9 | Shoulder1: ±2.62 / Shoulder2: -1.57~1.83 / Shoulder3: ±2.62 / Elbow1: ±1.57 / Elbow2: ±2.62 / Wrist1: ±2.09 / Wrist2: ±6.28 / Finger1/2: -0.015~0 | Finger 为 m，其余 rad | |
| 右臂 | 9 | 同左臂 | 同左臂 | 右手 Finger1/2: 0~0.015 m |

> 平移关节（Lift、四个 Finger）单位 m；旋转关节单位 rad；速度单位 rad/s（平移关节 m/s）。
> 夹爪（Finger）限位：左指 `[-0.015, 0]`，右指 `[0, 0.015]`（HMI 展示/操作口径，比
> `ylr1d_control_sim/config/position_control_limits.yaml` 的 ±0.014 略宽松；越界目标会被
> 仿真层硬钳制到实际限位处）。
> 传输层 HMI 的夹爪语义取 ±0.014（与转译层常量一致），见 Gripper 标签页说明。

---

## 话题与 Action

### 控制层 HMI 话题

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | `/joint_states` | `sensor_msgs/JointState` | 关节反馈（来自 Gazebo） |
| 发布 | `/desired_joint_states` | `sensor_msgs/JointState` | 用户设定的期望值 |

`/desired_joint_states` 发布规则：
- **转向关节**（`Joint_Base_to_*WheelF` ×4）：`position` 字段
- **轮子关节**（`Joint_*WheelF_to_*Wheel` ×4）：`velocity` 字段
- **躯干/臂关节**（其余 22 关节）：`position` 字段
- 发布方式：**GUI 操作触发 + 定时合并发送**（默认 5 Hz，可由工具栏 Rate 调整；Auto 开关控制是否持续发送）

### 传输层 HMI Action 客户端

| Action | 服务端 | Goal 摘要 |
|--------|--------|-----------|
| `/chassis_move` | `ylr1d_translate` | mode(平移/旋转/停车) + direction + speed + duration |
| `/arm_move` | `ylr1d_translate` | part(躯干/左臂/右臂) + 全部关节 positions |
| `/gripper_move` | `ylr1d_translate` | part(左/右) + open(布尔) |

---

## 使用

```bash
# 控制层 HMI（配合 position 方案完整闭环，推荐）
ros2 launch ylr1d_bringup bringup.launch.py

# 或分步：
# 终端 1: 物理层（position 接口方案）
ros2 launch ylr1d_plant gazebo.launch.py
# 终端 2: 控制层软仿真
ros2 launch ylr1d_control_sim position_simulate.launch.py
# 终端 3: 控制层 HMI
ros2 launch ylr1d_hmi hmi.launch.py

# 传输层 HMI（需要 ylr1d_translate 节点在运行）
ros2 launch ylr1d_hmi hmi_translate.launch.py

# 传感器 HMI
ros2 launch ylr1d_hmi sensor_panel.launch.py
```

> WSL 下需 X server（WSLg 或 VcXsrv）支持 GUI 显示。
> 环境变量 `LIBGL_ALWAYS_SOFTWARE=1` 已在各 launch 文件中自动设置。
> **注意**：控制层 HMI 只发布 `/desired_joint_states`，若只启动 HMI + `gazebo_effort.launch.py`
> （力控方案），`/desired_joint_states` 无人订阅，滑动不会产生任何运动——必须经过
> `ylr1d_control_sim` 过渡。

---

## 依赖

- **ROS2**: rclcpp, rclcpp_action（仅传输层）, sensor_msgs, std_msgs, ylr1d_translate（仅传输层）
- **Qt5**: Widgets, Core
- 系统: qtbase5-dev, libqt5widgets5

---

## 构建

```bash
source /opt/ros/humble/setup.bash
cd <workspace>
colcon build --packages-select ylr1d_hmi
source install/setup.bash
```

---

## 关键文件

```
ylr1d_hmi/
├── include/ylr1d_hmi/panels/
│   ├── hmi_window.hpp          # HmiWindow：控制层面板（观测表 + 控制行）
│   ├── sensor_panel.hpp        # SensorPanel：传感器观测面板
│   └── translate_panel.hpp     # TranslatePanel：传输层 action 客户端面板
├── src/
│   ├── main_control.cpp        # ylr1d_hmi_control 入口
│   ├── main_sensor.cpp         # ylr1d_hmi_sensor 入口
│   ├── main_translate.cpp      # ylr1d_hmi_translate 入口
│   └── panels/
│       ├── hmi_window.cpp      # 控制层面板实现（ROS 通信 + 单位换算）
│       ├── sensor_panel.cpp    # 传感器面板实现（渲染 + TopicStatus）
│       └── translate_panel.cpp # 传输层面板实现（action 发送 + feedback/result）
├── launch/
│   ├── hmi.launch.py           # 控制层 HMI
│   ├── sensor_panel.launch.py  # 传感器 HMI
│   └── hmi_translate.launch.py # 传输层 HMI
├── CMakeLists.txt              # 三个可执行文件
├── package.xml
└── README.md
```

## 已知限制

- **单线程设计**：各 Qt 主线程同时处理 GUI 和 ROS spin（`rclcpp::spin_some` 在 QTimer 中调用），重负载下可能卡顿
- **未做代码层硬限幅**：控制层 Slider/SpinBox 的 range 约束了输入范围，但代码层面未对发布值做二次限幅
- **WSL 渲染性能**：WSL 下 GUI 响应较慢（软件渲染）
- **传感器 HMI 渲染**：三台相机在 WSL 软渲染下不能全量渲染，只渲染选中的相机（其余仅缓存）
