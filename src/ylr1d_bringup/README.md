# ylr1d_bringup — 一键启动

将 YLR1D 仿真栈各功能包的 launch 聚合为**单一启动入口**，避免逐个手动拉起。
本包不含节点，仅做 launch 编排。

按"入口所在层"提供两条完整栈：

| Launch 文件 | 链路 | 适用场景 |
|-------------|------|----------|
| `bringup_control.launch.py` | **控制层 → 物理层** | 用控制层 HMI 直接调关节 |
| `bringup_translate.launch.py` | **转译层 → 物理层** | 用转译层 HMI 发语义动作 |

---

## 一、控制层完整栈 `bringup_control.launch.py`

```
control HMI → /desired_joint_states → control_sim → 5 组命令话题 → plant(Gazebo)
```

聚合的 launch：

| 来源包 | launch 文件 | 作用 |
|--------|------------|------|
| `ylr1d_plant` | `gazebo.launch.py` | 物理层：Gazebo + ros2_control（position 接口方案） |
| `ylr1d_control_sim` | `position_simulate.launch.py` | 控制层软仿真：`chassis_simulate` + `arm_simulate` |
| `ylr1d_hmi` | `hmi.launch.py` | 控制层 HMI：观测 + 直接发 `/desired_joint_states` |
| `ylr1d_hmi` | `sensor_panel.launch.py` | 传感器观测面板 |

```bash
ros2 launch ylr1d_bringup bringup_control.launch.py
```

## 二、转译层完整栈 `bringup_translate.launch.py`

```
translate HMI → action goal → translate_server → /desired_joint_states
              → control_sim → 5 组命令话题 → plant(Gazebo)
```

聚合的 launch：

| 来源包 | launch 文件 | 作用 |
|--------|------------|------|
| `ylr1d_plant` | `gazebo.launch.py` | 物理层：Gazebo + ros2_control（position 接口方案） |
| `ylr1d_control_sim` | `position_simulate.launch.py` | 控制层软仿真：`chassis_simulate` + `arm_simulate` |
| `ylr1d_translate` | `translate.launch.py` | 转译层：`translate_server`（chassis/arm/gripper 三个 action server） |
| `ylr1d_hmi` | `hmi_translate.launch.py` | 转译层 HMI：发语义级 action goal |
| `ylr1d_hmi` | `sensor_panel.launch.py` | 传感器观测面板 |

```bash
ros2 launch ylr1d_bringup bringup_translate.launch.py
```

> **注意**：两条栈各带各的 HMI，**不要同时启动**两个 bringup（或混开 control HMI 与
> translate HMI）——两者都会向下游下发期望值（control 直接发 `/desired_joint_states`，
> translate 经 translate_server 再发），会对 control_sim 竞争同一批关节。

## 三、通用说明

- **参数 `world`**：两条栈都透传 `world:=<文件名>` 给 `ylr1d_plant gazebo.launch.py`
  （世界文件在 `ylr1d_description/worlds/` 下），默认 `empty.world`
- **环境自足**：被包含的 launch 已自带 `LIBGL_ALWAYS_SOFTWARE=1`、`GAZEBO_MODEL_PATH`、
  `LD_LIBRARY_PATH` 设置，以及 Gazebo spawner 的 `TimerAction` 时序，本包无需重复处理
- **position 接口**：本包默认聚合 position 方案（`gazebo.launch.py`），与 `position_simulate.launch.py`
  输出的 `*_controller/commands` 话题匹配
- **切换 effort 力控**：如改用 effort 方案，把 include 的 `gazebo.launch.py` 换成
  `gazebo_effort.launch.py`（此时 control_sim 的命令话题将无法与 effort 控制器对接，仅适合直接力控调试）
- **模型资产**：全部来自 `ylr1d_description`，由 `ylr1d_plant` 在内部加载，本包不直接接触

### 启动后验证（WSL 下 Gazebo 加载慢，30-60s 控制器才 active）

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

转译层栈额外确认 action server 已就绪：

```bash
ros2 action list -t
# 期望出现 /chassis_move /arm_move /gripper_move（server = translate_server）
```

---

## 四、相关文档

- 根 README：`README.md`（工作空间根目录）
- 各被聚合包 README：
  - [ylr1d_plant](../ylr1d_plant/README.md)
  - [ylr1d_control_sim](../ylr1d_control_sim/README.md)
  - [ylr1d_translate](../ylr1d_translate/README.md)
  - [ylr1d_hmi](../ylr1d_hmi/README.md)
