# ylr1d_bringup — 一键启动

将 YLR1D 仿真栈的三个功能包 launch 聚合为**单一启动入口**，避免逐个手动拉起。
本包不含节点，仅做 launch 编排。

---

## 一、包介绍

| 内容 | 说明 |
|------|------|
| `launch/bringup.launch.py` | 聚合启动物理层 + 控制层 + 人机界面 |

聚合的三个 launch：

| 来源包 | launch 文件 | 作用 |
|--------|------------|------|
| `ylr1d_plant` | `gazebo.launch.py` | 物理层：Gazebo + ros2_control + rviz2（position 接口方案） |
| `ylr1d_control_sim` | `position_simulate.launch.py` | 控制层软仿真：`chassis_simulate` + `arm_simulate` |
| `ylr1d_hmi` | `hmi.launch.py` | Qt5 人机界面 |

启动后形成闭环：`HMI → /desired_joint_states → control_sim → 5组命令话题 → plant (Gazebo) → /joint_states 反馈`。

---

## 二、使用方法

```bash
ros2 launch ylr1d_bringup bringup.launch.py
```

WSL 下 Gazebo 加载慢（30-60s 控制器才 active），用下面命令轮询，全部 active 后再发命令：

```bash
ros2 control list_controllers
# 期望全部 active：
#   chassis_steering_controller
#   chassis_wheels_controller
#   torso_controller
#   left_arm_controller
#   right_arm_controller
#   joint_state_broadcaster
```

> 等价于依次执行：
> ```bash
> ros2 launch ylr1d_plant gazebo.launch.py
> ros2 launch ylr1d_control_sim position_simulate.launch.py
> ros2 launch ylr1d_hmi hmi.launch.py
> ```

---

## 三、补充说明

- **环境自足**：被包含的 launch 已自带 `LIBGL_ALWAYS_SOFTWARE=1`、`GAZEBO_MODEL_PATH`、
  `LD_LIBRARY_PATH` 设置，以及 Gazebo spawner 的 `TimerAction` 时序，本包无需重复处理
- **position 接口**：本包默认聚合 position 方案（`gazebo.launch.py`），与 `position_simulate.launch.py`
  输出的 `*_controller/commands` 话题匹配
- **切换 effort 力控**：如改用 effort 方案，把 include 的 `gazebo.launch.py` 换成
  `gazebo_effort.launch.py`（此时 control_sim 的命令话题将无法与 effort 控制器对接，仅适合直接力控调试）
- **模型资产**：全部来自 `ylr1d_description`，由 `ylr1d_plant` 在内部加载，本包不直接接触

---

## 四、相关文档

- 根 README：`README.md`（工作空间根目录）
- 各被聚合包 README：
  - [ylr1d_plant](../ylr1d_plant/README.md)
  - [ylr1d_control_sim](../ylr1d_control_sim/README.md)
  - [ylr1d_hmi](../ylr1d_hmi/README.md)
