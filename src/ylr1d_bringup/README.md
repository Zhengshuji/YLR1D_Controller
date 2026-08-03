# ylr1d_bringup — 一键启动编排

聚合 `ylr1d_plant` / `ylr1d_control_sim` / `ylr1d_translate` / `ylr1d_hmi` 的 launch 为**单一启动入口**，避免逐个手动拉起。本包**不含节点**，仅做 launch 编排。

---

## 一、功能定位

- **架构位置**：最顶层，纯启动编排。
- **职责**：把物理层（Gazebo）、控制层（软仿真）、转译层（可选）、HMI 聚合为单个 `ros2 launch` 入口。
- **提供两套完整链路**（按"入口所在层"区分）：

| Launch 文件 | 链路 | 适用场景 |
|-------------|------|----------|
| `bringup_control.launch.py` | **控制层 → 物理层** | 用控制层 HMI 直接调关节 |
| `bringup_translate.launch.py` | **转译层 → 物理层** | 用转译层 HMI 发语义动作 |

- **不做什么**：不含节点、不定义任何接口，只 include 其他包的 launch 并透传参数。

---

## 二、包结构

```
ylr1d_bringup/
├── launch/
│   ├── bringup_control.launch.py    # 控制层完整栈
│   └── bringup_translate.launch.py  # 转译层完整栈
└── package.xml
```

> 两个 launch 共用同一套 `_include()` 辅助函数，通过 `get_package_share_directory` 定位各包 share 目录下的 launch 文件。

---

## 三、使用方法

```bash
# 控制层链路：Gazebo + 软仿真 + 控制层 HMI + 传感器面板
ros2 launch ylr1d_bringup bringup_control.launch.py

# 转译层链路：Gazebo + 软仿真 + translate + 转译层 HMI + 传感器面板
ros2 launch ylr1d_bringup bringup_translate.launch.py

# 可选：指定 Gazebo 世界（ylr1d_description/worlds/ 下）
ros2 launch ylr1d_bringup bringup_control.launch.py world:=sensors_test.world
```

### 启动后验证

WSL 下 Gazebo 加载慢（30-60s 控制器才 active），等待控制器全部 active：

```bash
ros2 control list_controllers
# 期望全部 active：chassis_steering / chassis_wheels / torso / left_arm / right_arm / joint_state_broadcaster
```

转译层栈额外确认 action server 已就绪：

```bash
ros2 action list -t
# 期望出现 /chassis_move /arm_move /gripper_move（server = translate_server）
```

---

## 四、接口

- **无运行期接口**：本包不启动任何节点，不发布/订阅话题，不起 action server。
- **参数透传**：`world` 透传给 `ylr1d_plant gazebo.launch.py`。

---

## 五、配置

| 参数 | 默认 | 说明 |
|------|------|------|
| `world` | `empty.world` | Gazebo 世界文件名（`ylr1d_description/worlds/` 下），透传给 plant 的 `gazebo.launch.py` |

- **position 接口**：本包默认聚合 position 方案（`gazebo.launch.py`），与 `position_simulate.launch.py` 输出的 `*_controller/commands` 话题匹配。
- **环境自足**：被包含的 launch 已自带 `LIBGL_ALWAYS_SOFTWARE=1`、`GAZEBO_MODEL_PATH`、`LD_LIBRARY_PATH` 设置及 Gazebo spawner 的 `TimerAction` 时序，本包无需重复处理。

---

## 六、关键机制（选读）

### 聚合关系

| 链路 | 聚合的 launch |
|------|--------------|
| `bringup_control` | `plant/gazebo.launch.py` + `control_sim/position_simulate.launch.py` + `hmi/hmi.launch.py` + `hmi/sensor_panel.launch.py` |
| `bringup_translate` | `plant/gazebo.launch.py` + `control_sim/position_simulate.launch.py` + `translate/translate.launch.py` + `hmi/hmi_translate.launch.py` + `hmi/sensor_panel.launch.py` |

### 链路

```
控制层：control HMI → /desired_joint_states → control_sim → 5 组命令话题 → plant(Gazebo)
转译层：translate HMI → action goal → translate_server → /desired_joint_states → control_sim → 5 组命令话题 → plant(Gazebo)
```

### 模型资产

全部来自 `ylr1d_description`，由 `ylr1d_plant` 在内部加载，本包不直接接触。

### 切换 effort 力控

如改用 effort 方案，需把 include 的 `gazebo.launch.py` 换成 `gazebo_effort.launch.py`。此时 `control_sim` 的命令话题无法与 effort 控制器对接，该组合仅适合直接力控调试。

---

## 七、已知限制与注意事项

- **两条栈不要同时启动**：两栈各带各的 HMI（control 直发 `/desired_joint_states`，translate 经 `translate_server` 再发），混开会**竞争同一批关节**
- **WSL 下启动慢**：从 launch 到控制器 active 需 30-60s，用 `ros2 control list_controllers` 轮询，不能抢跑
- **依赖顺序**：两链路都依赖 `ylr1d_plant` 正常启动；中途任何一环失败需整栈重启（先清理残留：`pkill -f gzserver; pkill -f gzclient`）
- **sensor_panel 总是被聚合**：两链路都含传感器面板，无需单独再启动
