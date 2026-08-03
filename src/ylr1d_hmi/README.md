# ylr1d_hmi — 人机界面（Qt5）

基于 Qt5 的人机交互界面，用于 YLR1D 机器人调试。按职责拆分为**四个面板**，覆盖控制、转译、传感器观测与仿真监视。

---

## 一、功能定位

- **架构位置**：最顶层人机交互，面向 `ylr1d_control_sim` / `ylr1d_translate` 与各传感器话题。
- **四个面板**：

| 面板 | 可执行 | Launch | 面向 | 用途 |
|------|--------|--------|------|------|
| 控制层 HMI | `ylr1d_hmi_control` | `hmi.launch.py` | `ylr1d_control_sim` | 观测 `/joint_states`，发 `/desired_joint_states` |
| 转译层 HMI | `ylr1d_hmi_translate` | `hmi_translate.launch.py` | `ylr1d_translate` | 发 action goal（底盘 / 机械臂 / 夹爪） |
| 传感器 HMI | `ylr1d_hmi_sensor` | `sensor_panel.launch.py` | 各传感器话题 | 只读观测相机 / 点云 / 雷达 / 超声波 / IMU |
| 监视 HMI | rviz2 插件 或 `ylr1d_hmi_monitor` | `monitor.launch.py` / `ros2 run` | 仿真全局 | 监视控制器 / 节点 / rosout / 传感器 / 关节 |

- **不做什么**：不参与控制闭环（控制层 HMI 只发期望值，运动由 `control_sim` 过渡），只做观测与指令下发。

---

## 二、包结构

```
ylr1d_hmi/
├── include/ylr1d_hmi/panels/
│   ├── hmi_window.hpp        # HmiWindow：控制层面板（观测表 + 控制行）
│   ├── translate_panel.hpp   # TranslatePanel：转译层 action 客户端面板
│   ├── sensor_panel.hpp      # SensorPanel：传感器观测面板
│   ├── monitor_panel.hpp     # MonitorWidget：监视面板核心（rviz2 插件与独立窗口共用）
│   ├── monitor_rviz_panel.hpp# MonitorRvizPanel：rviz2 插件宿主
│   └── topic_status.hpp      # TopicStatus：话题存活/频率统计
├── src/
│   ├── main_control.cpp      # ylr1d_hmi_control 入口
│   ├── main_translate.cpp    # ylr1d_hmi_translate 入口
│   ├── main_sensor.cpp       # ylr1d_hmi_sensor 入口
│   ├── main_monitor.cpp      # ylr1d_hmi_monitor 入口（独立窗口）
│   └── panels/               # 各面板实现
├── launch/                   # hmi / hmi_translate / sensor_panel / monitor 四个 launch
├── rviz/monitor.rviz         # 监视面板的 rviz2 配置
├── plugin_description.xml    # rviz2 插件声明
├── CMakeLists.txt            # 5 个 target（4 可执行 + 1 插件库）
└── package.xml
```

> 面板实现放 `src/panels/`，头文件放 `include/ylr1d_hmi/panels/`（参考 `ylr1d_control_sim` 的文件管理原则）。

---

## 三、使用方法

```bash
# 控制层 HMI（配合 position 方案完整闭环）
ros2 launch ylr1d_hmi hmi.launch.py

# 转译层 HMI（需要 ylr1d_translate 节点在运行）
ros2 launch ylr1d_hmi hmi_translate.launch.py

# 传感器 HMI
ros2 launch ylr1d_hmi sensor_panel.launch.py

# 监视 HMI：rviz2 版（预加载 YLR1D Monitor 面板）
ros2 launch ylr1d_hmi monitor.launch.py
# 或独立窗口版（无 3D 视图，便于无头验证）
ros2 run ylr1d_hmi ylr1d_hmi_monitor

# 完整链路（推荐直接走 bringup，已聚合对应 HMI + 传感器面板）
ros2 launch ylr1d_bringup bringup_control.launch.py
ros2 launch ylr1d_bringup bringup_translate.launch.py
```

> - **无头验证**：监视独立窗口与传感器面板可在 `QT_QPA_PLATFORM=offscreen` 下运行。
> - WSL 下需 X server（WSLg 或 VcXsrv）支持 GUI 显示；`LIBGL_ALWAYS_SOFTWARE=1` 已在各 launch 中自动设置。
> - **注意**：控制层 HMI 只发布 `/desired_joint_states`，若只启动 HMI + `gazebo_effort.launch.py`（力控方案），该话题无人订阅，滑动不会产生任何运动——必须经过 `ylr1d_control_sim` 过渡。

---

## 四、接口

### 话题

| 面板 | 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|------|
| 控制层 HMI | 订阅 | `/joint_states` | `sensor_msgs/JointState` | 关节反馈（来自 Gazebo） |
| 控制层 HMI | 发布 | `/desired_joint_states` | `sensor_msgs/JointState` | 用户设定的期望值 |
| 传感器 HMI | 订阅 | 相机 / 点云 / 雷达 / 超声波 / IMU 各话题 | 各传感器消息 | 只读观测 |
| 监视 HMI | 订阅 | `/joint_states`、`/rosout`、`/clock`、各传感器话题 | — | 存活/频率统计与展示 |

`/desired_joint_states` 发布规则：转向关节（×4）用 `position`；轮子关节（×4）用 `velocity`；躯干/臂/夹爪（其余 22）用 `position`。发布方式为 **GUI 操作触发 + 定时合并发送**（默认 5 Hz，可由工具栏调整；Auto 开关控制是否持续发送）。

### Action 客户端（转译层 HMI）

| Action | 服务端 | Goal 摘要 |
|--------|--------|-----------|
| `/chassis_move` | `ylr1d_translate` | mode（平移/旋转/停车）+ direction + speed + duration |
| `/arm_move` | `ylr1d_translate` | part（躯干/左臂/右臂）+ 全部关节 positions |
| `/gripper_move` | `ylr1d_translate` | part（左/右）+ open（布尔） |

---

## 五、配置

- **无独立参数 yaml**，运行参数均为头文件/代码常量（见[关键机制](#六关键机制选读)）。
- **关节限位**：以 `ylr1d_description/config/limits.yaml` 为基准，控制层 HMI 各分组限位范围：转向 ±3.14 rad；轮子 ±5.0 rad/s（连续旋转）；躯干 Lift（棱柱）±0.30 m、其余 ±3.14 / ±1.57 rad；左右臂各关节 ±1.57 ~ ±6.28 rad（非对称肩 2 为 -1.57 ~ +1.83）；夹爪（棱柱）左指 [-0.015, 0]、右指 [0, 0.015] m。
  - 平移关节（Lift、Finger）单位 m，旋转关节 rad，速度 rad/s（平移关节 m/s）。
  - ⚠️ 夹爪展示口径取 ±0.015，比仿真层 `position_control_limits.yaml` 的 ±0.014 **略宽松**；越界目标会被仿真层硬钳制到实际限位。转译层 HMI 的夹爪语义取 ±0.014（与转译层常量一致）。
- **监视面板**：`rviz/monitor.rviz` 预加载 "YLR1D Monitor" 面板。

---

## 六、关键机制（选读）

### 6.1 控制层 HMI（ylr1d_hmi_control）

- **四标签页**：Chassis / Torso / Left Arm / Right Arm（标签页标题英文，避免 WSL 中文字体乱码）
- **上观测 / 下控制**：观测区为只读表格（`[名称] [Position] [Velocity]`，50Hz 刷新）；控制区每关节一行 `[Slider] [SpinBox]`，Slider 范围对应关节限位、SpinBox 带单位后缀；Slider / SpinBox 双向同步
- **发送方式**：拖动不立即发布，仅置脏标记，由发送定时器（默认 5 Hz）合并发送一次到 `/desired_joint_states`，避免高频拖动洪泛
- **工具栏**：Units（rad/deg、m/mm 显示切换，内部存储保持 SI）；Rate（1/2/5/10/20 Hz）；Auto（持续发送总开关）；Send Now（手动立即发送）

### 6.2 转译层 HMI（ylr1d_hmi_translate）

- **Chassis 标签页**：Mode（平移/旋转/停车）联动 Direction / Speed 的单位与可用性；点击发送即 `async_send_goal`，状态栏显示连接、goal 接受、feedback 阶段（steering/moving/stopped）与 result
- **Arm 标签页**：Part（躯干/左臂/右臂）用 `QStackedWidget` 切换对应关节滑杆行（躯干 4、左右臂各 7）；**机械臂与夹爪解耦**，左右臂面板不含夹爪两指；关节名称/顺序与 `ylr1d_translate` 关节表一致
- **Gripper 标签页**：Part（左/右）+ Open（开/关）布尔，语义由转译层解算（左指 -0.014↔0、右指 0↔+0.014，左右不对称）
- 面板底部有 action 通信状态区：三个 server 连接指示（1s 刷新）+ 事件日志

### 6.3 传感器面板（ylr1d_hmi_sensor）

| 传感器 | 数量 | 展示形式 |
|--------|------|----------|
| 相机（RGB/深度/红外） | 3 台 | 图像 + camera_info 参数；WSL 软渲染下仅渲染选中相机 |
| 深度相机点云 | 3 台 | 2D 投影 |
| 雷达 | 1 | 360° 极坐标激光扫描视图 + 统计 |
| 超声波 | 4 | 窄扇形激光扫描视图 + 统计 |
| IMU | 1 | 数值读出 |

每个视图带 `TopicStatus` 行（收到帧数 · 距上次更新秒数 · 估算频率），独立于渲染判断话题是否存活。

### 6.4 监视面板（MonitorWidget）

rviz2 插件（`monitor.launch.py`）与独立窗口（`ylr1d_hmi_monitor`）共用同一核心。四个标签页：
- **Overview**：仿真时钟（`/clock`）、控制器状态（`controller_manager` ListControllers）、预期节点存活、异常汇总
- **Log**：`/rosout` 流 + 过滤 + 里程碑/异常聚合
- **Sensors**：全部传感器话题的存活状态
- **Joints**：按关节分组下拉（与 `ylr1d_control_sim` 分组一致）

环境事实（实测）：`/rosout` 消息类型是 `rcl_interfaces/msg/Log`；`/clock` 用 **BEST_EFFORT** QoS（默认 RELIABLE 会收不到）；rviz2 插件内自建节点必须 `NodeOptions.use_global_arguments(false)` 才能保留自身节点名（否则被重命名成 `rviz2`）。

### 6.5 单线程模型

各面板 ROS 回调运行在 GUI 线程（QTimer 调 `rclcpp::spin_some`），回调只缓存最新消息，缓存数据无需加锁。

---

## 七、已知限制与注意事项

- **勿混用两套 HMI**：控制层 HMI 与转译层 HMI 同时使用会**竞争同一批关节**（各自下游都发期望值），完整链路应走 `bringup_control` / `bringup_translate` 之一
- **单线程卡顿**：GUI 与 ROS spin 同线程，重负载下可能卡顿
- **WSL 渲染性能**：WSL 下 GUI 响应较慢（软件渲染）；三台相机在软渲染下不能全量渲染，只渲染选中的相机（其余仅缓存）
- **监视面板需仿真在跑**：`monitor.launch.py` 需在仿真（`gazebo.launch.py` 或 bringup）运行时另行启动
- **依赖**：rclcpp / rclcpp_action（仅转译层）/ sensor_msgs / std_msgs / ylr1d_translate（仅转译层）/ rviz_common + pluginlib + class_loader（监视插件）/ Qt5 Widgets, Core
