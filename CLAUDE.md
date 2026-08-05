# CLAUDE.md — YLR1D 项目备忘录

> **路径约定**：本文中 `<WS_ROOT>` 指工作空间根目录（如 `/home/zsj/WorkSpace/ros2_ylr1d_controller_ws`）。
> 工作空间重命名后，将 `<WS_ROOT>` 替换为新路径即可，代码与文档均不依赖具体路径。

---

## 一、项目概述

双机械臂 + 全向四轮底盘 + 升降躯干的复合移动机器人仿真，基于 **ROS2 Humble + Gazebo Classic**，全模型 30 关节。六个功能包分层协作：

| 包 | 职责 |
|----|------|
| `ylr1d_description` | 模型资产单一来源（xacro / mesh / config / rviz / world） |
| `ylr1d_plant` | 物理层（中控）：Gazebo + ros2_control |
| `ylr1d_control_sim` | 控制层（软仿真）：PID 过渡 → 5 组命令 |
| `ylr1d_translate` | 转译层：上层 action → `/desired_joint_states` |
| `ylr1d_hmi` | 人机界面：Qt5 四面板观测 + 控制 |
| `ylr1d_bringup` | 一键启动：聚合各包 launch |

> **语言约定**：项目主体基于 C++（各包节点均为 C++）。Python 仅限 launch 与 test 层
> （`launch/*.py`、`launch/python_utils/*.py`、`test/*.py`），核心逻辑禁止用 Python。

> **详细说明见 [README.md](README.md)**：总体架构、数据流、包一览、快速开始、接口速查。
> 各包功能细节见对应包 README（根 README"文档导航"有索引）。本文件仅保留备忘录性质的注意项与工作流程。

---

## 二、注意事项

### 环境与执行

#### WSL 执行命令
本项目在 WSL Ubuntu-22.04 中运行。从 Windows CLI 调用 WSL 时必须添加 `MSYS2_ARG_CONV_EXCL="*"` 防止 Git Bash/MSYS2 路径转换：

```bash
MSYS2_ARG_CONV_EXCL="*" wsl.exe -d Ubuntu-22.04 bash -c 'source <WS_ROOT>/install/setup.bash; ros2 ...'
```

#### 环境初始化
```bash
cd <WS_ROOT>
source install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/src
```

脚本 `colcon_build.sh` 中有完整的环境配置参考（脚本会自动定位工作空间根目录）。

#### 注意
- WSL 下 GPU 渲染受限，Gazebo 需 `LIBGL_ALWAYS_SOFTWARE=1`
- Gazebo Classic 在 WSL 下启动很慢（~30-60s 控制器才完全加载），务必耐心等待 `ros2 control list_controllers` 全部显示 `active`

### 功能测试约定
- 功能测试统一收敛到 `ylr1d_test` 包（`src/ylr1d_tools/ylr1d_test`），入口脚本 `src/ylr1d_tools/scripts/run_tests.sh`，结果落盘 `test_results/`（已加入 `.gitignore`）。详见 [ylr1d_test README](src/ylr1d_tools/ylr1d_test/README.md)。
- 新增/改动功能包时，同步补充/更新对应测试（Tier0 `env` 环境检查、Tier1 各层冒烟、Tier2 集成/全流程）。
- 需 ROS2 的测试必须**独立 launch**（`ylr1d_test` 自带 `plant_stack.launch.py` / `test_stack.launch.py`），**不得并入 bringup**（bringup 一行不改）。
- 测试互不干扰：独立进程组 + 测试前后清理 + 结束核查无残留；失败诊断只给「问题信息 + 解决方案建议」。

### 仿真验证（常用命令）

#### 一键启动（完整闭环，推荐）
```bash
ros2 launch ylr1d_bringup bringup_control.launch.py
```
等价于依次启动 `gazebo.launch.py`（position 方案）+ `position_simulate.launch.py` + `hmi.launch.py` + `sensor_panel.launch.py`。

#### 力控测试（effort 方案，单独启动）
```bash
ros2 launch ylr1d_plant gazebo_effort.launch.py
```

#### 转译层链路（上层 action 驱动）
```bash
ros2 launch ylr1d_bringup bringup_translate.launch.py
```

#### 验证控制器激活（等待输出全部为 active）
```bash
ros2 control list_controllers
```

#### 获取广义坐标
```bash
ros2 topic echo /joint_states --once
```
所有 30 个关节的 position / velocity / effort 均在 `/joint_states` 中。prismatic 关节的 NaN 已被 `joint_state_filter` 处理为 0.0，过滤后数据在 `/joint_states_filtered`。

### 已知弯路 / 常见陷阱

#### 1. WSL 路径转换（MSYS2_ARG_CONV_EXCL）
**问题**: Git Bash 自动将 `/home/...` 转换为 `C:/Program Files/Git/home/...`
**解决**: 所有 `wsl.exe` 调用前加 `MSYS2_ARG_CONV_EXCL="*"`

#### 2. gzserver 生命周期
**问题**: `ros2 launch` 退出后 `gzserver` 仍在后台运行，再次启动会冲突（`Entity already exists`）
**解决**: 重启前清理：
```bash
pkill -f gzserver; pkill -f gzclient
```

#### 3. 控制器加载慢（WSL 性能）
**问题**: WSL 下 Gazebo 启动到控制器 active 需要 30-60s，而非文档写的 10-15s
**解决**: 启动后使用 `ros2 control list_controllers` 轮询直到全部 active，不能抢跑

#### 4. Joint_Base_to_Body1 限位
**问题**: 限位 [-0.3, 0.3] rad，初始位置就是 0.3，正向力矩推不动
**解决**: 调试时先用负向力矩离开限位

#### 5. effort 命令需要持续发送
**问题**: ForwardCommandController 虽然保持最后值，但单次 `--once` 在重力/碰撞下可能不足以产生可见运动
**解决**: 用 `--rate 20` 持续发送

#### 6. JointState 解析
**问题**: `ros2 topic echo /joint_states` 输出格式为 YAML-like 多行文本，pipe 给 grep 不容易提取
**解决**: 用 Python 写 rclpy 节点直接订阅解析，或使用 `ros2 topic echo --field data`

#### 7. 棱柱关节 NaN
**问题**: GazeboSystem 在 spawn 暂停阶段将 prismatic joint 位置初始化为 NaN，导致 `TF_NAN` 刷屏
**解决**: `joint_state_filter` 节点将 NaN → 0.0，已集成到 `gazebo_effort.launch.py`

#### 8. ros2 topic echo 输出被文件重定向截断
**问题**: `timeout 3 ros2 topic echo /joint_states --once > file.txt 2>&1` 可能输出空文件，因为 timeout 在消息到达前就结束了
**解决**: timeout 给足 5s，或使用 `ros2 topic echo --once` 不加 timeout（在后台运行时使用）

#### 9. gzserver 与 spawner 的时序
**问题**: spawner 在 Gazebo 完全加载前启动会连不上 controller_manager 服务
**解决**: launch 文件中使用 `TimerAction(period=8.0)` 延迟 spawner 启动

#### 10. control_sim 建议经 launch 启动
**问题**: 直接 `ros2 run`（不经 launch）时 `pid.yaml` 未加载，pid 退化为预设默认值（kp=4/kd=0.2，非 yaml 的 150/20），控制响应偏软（limit 已编译进头文件，不再受此影响）
**解决**: 用 `ros2 launch ylr1d_control_sim position_simulate.launch.py` 启动；改 pid 需改 `config/pid.yaml` 后重启节点（运行中 `ros2 param set` 不生效），改 limit 需改头文件后重新编译

#### 11. pkill -f 会自匹配（WSL bash）
**问题**: `pkill -f chassis_simulate` 会匹配到 bash 自身命令行里的同名模式，把执行 shell 杀掉（exit 15）
**解决**: 用字符类技巧 `pkill -f "[c]hassis_simulate"`，或先 `ps` 确认 PID 再精确 kill

#### 12. ylr1d_hmi 静态配置单一来源
**约定**: `ylr1d_hmi` 的关节定义统一在 `include/ylr1d_hmi/config/joint_defs.hpp`（30 关节原子 + 控制/监视/转译三组视图），传感器话题统一在 `config/sensor_topics.hpp`。改限位/话题只改这两处，并对照 `ylr1d_description/config/limits.yaml` 语境；action 发送公共逻辑在 `common/action_sender.hpp`。

#### 13. ylr1d_control_sim 静态配置单一来源
**约定**: `ylr1d_control_sim` 的 limit 与关节分组唯一在 `include/ylr1d_control_sim/config/joint_config.hpp`（`kPositionLimits`[26] / `kVelocityLimits`[4] + `jointLimitFor(name)` + 底盘/臂分组常量），pid 在 `config/pid.yaml`（经 launch 加载为 `<关节>/pid/*` 参数）。改限位/关节分组只改头文件后重新编译；改 pid 只改 yaml 后重启节点；改限位前对照 `ylr1d_description/config/limits.yaml` 语境。

---

## 三、工作流程

以下为本项目约定俗成的协作方式，适用于任何修改类任务（代码 / 文档 / 配置）。

### 1. 了解当前情况
- **首先阅读 README**（根 README + 相关包 README），而不是通读代码。
- README 已覆盖项目的使用方式、接口与限制，读完即可对当前状态有整体把握。
- 需要更深处（类结构、内部机制）时，再由 README 指明的文件进入代码。

### 2. 分析理解任务
- 有问题**尽管提**，有不清楚的要求**尽管问**，在开始执行前把任务边界确认清楚。
- 不要在执行中途才发现理解偏差——前期多问一句的代价远小于返工。

### 3. 执行任务
- 遵守 CLAUDE.md 中的注意事项与已知陷阱，避免重蹈覆辙。
- 涉及文档内容时，务必对照代码实证，不臆造（参考用户"文档准确性"偏好）。

### 4. 任务完成后：先文档、后备忘
- **先更新 README**：让 README 保持"读了就能用"——若本次改动改变了使用方式 / 接口 / 配置 / 限制，同步更新根 README 与对应包 README。
- **再结合具体情况更新 CLAUDE.md**：若产生了新的坑 / 注意事项 / 工作流经验，沉淀到这里。

### 5. 汇报
汇报时不仅说明做了什么，还要覆盖：
- **做了什么**：改动范围与关键内容
- **结果如何**：验证情况、是否达到预期
- **下一步打算**：有无遗留问题、待办事项
- **意见建议**：对项目的观察与改进建议
