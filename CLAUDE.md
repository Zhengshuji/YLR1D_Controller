# CLAUDE.md — YLR1D 项目备忘录

> **路径约定**：本文中 `<WS_ROOT>` 指工作空间根目录（如 `/home/zsj/WorkSpace/test_ylr1d`）。
> 工作空间重命名后，将 `<WS_ROOT>` 替换为新路径即可，代码与文档均不依赖具体路径。

## 环境与执行

### WSL 执行命令
本项目在 WSL Ubuntu-22.04 中运行。从 Windows CLI 调用 WSL 时必须添加 `MSYS2_ARG_CONV_EXCL="*"` 防止 Git Bash/MSYS2 路径转换：

```bash
MSYS2_ARG_CONV_EXCL="*" wsl.exe -d Ubuntu-22.04 bash -c 'source <WS_ROOT>/install/setup.bash; ros2 ...'
```

### 环境初始化
```bash
cd <WS_ROOT>
source install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/src
```

脚本 `colcon_build.sh` 中有完整的环境配置参考（脚本会自动定位工作空间根目录）。

### 注意
- WSL 下 GPU 渲染受限，Gazebo 需 `LIBGL_ALWAYS_SOFTWARE=1`
- Gazebo Classic 在 WSL 下启动很慢（~30-60s 控制器才完全加载），务必耐心等待 `ros2 control list_controllers` 全部显示 `active`

---

## 包结构速览

五个包，职责分层（模型资产 → 物理层 → 控制层 → 人机界面），外加一键启动编排：

```
ylr1d_description  ylr1d_plant  ylr1d_control_sim  ylr1d_hmi   ylr1d_bringup
      │                │               │                │              │
  模型资产(单一来源)  物理层(中控)    控制层(软仿真)    人机界面      一键启动
  xacro/mesh/config  Gazebo+ros2_control  PID过渡→5组命令   Qt5观测+控制  聚合三者launch
```

- **`ylr1d_description`**: 模型资产单一来源（xacro、meshes、模型 config yaml、sensors、rviz、world）。`ylr1d_plant` 与展示 launch 均从这里取资产
- **`ylr1d_plant`**: 物理层/中控，管理 Gazebo + ros2_control 的配置与启动。提供 `gazebo.launch.py`（position 接口）与 `gazebo_effort.launch.py`（effort 力控）两套方案；内含 `joint_state_filter`（NaN → 0.0）
- **`ylr1d_control_sim`**: 控制层/软仿真，模拟硬件位置/速度闭环。订阅 `/desired_joint_states` + `/joint_states`，PID 过渡后发布 5 组 ForwardCommandController 命令话题。参数从 `config/` 三个 yaml 加载（`<关节名>/limit/*`、`<关节名>/pid/*`）。**必须通过 `position_simulate.launch.py` 启动**。无头验证：`test/position_simulate_smoke_test.py`
- **`ylr1d_hmi`**: Qt5 人机交互界面，关节状态观测 + 关节控制器。Lite 版（`hmi.launch.py`）正常，RViz2 版存在构建/运行问题
- **`ylr1d_bringup`**: 一键启动编排（无节点）。`bringup.launch.py` 聚合 `gazebo.launch.py` + `position_simulate.launch.py` + `hmi.launch.py`，默认走 position 方案

> 注：`joint_state_filter`（NaN → 0.0）集成在 `ylr1d_plant` 的 `gazebo_effort.launch.py` 中（见陷阱 7）。
> 旧包名 `ylr1d_mid_control` → `ylr1d_plant`，`ylr1d_position_simulate` → `ylr1d_control_sim`。

---

## 仿真验证

### 一键启动（完整闭环，推荐）
```bash
ros2 launch ylr1d_bringup bringup.launch.py
```
等价于依次启动 `gazebo.launch.py`（position 方案）+ `position_simulate.launch.py` + `hmi.launch.py`。

### 力控测试（effort 方案，单独启动）
```bash
ros2 launch ylr1d_plant gazebo_effort.launch.py
```

### 验证控制器激活（等待输出全部为 active）
```bash
ros2 control list_controllers
```

### 发力矩命令（ForwardCommandController 保持最后值）
```bash
# steering — 30Nm 就能看到明显转动
ros2 topic pub --rate 20 /chassis_steering_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [30.0, 30.0, 30.0, 30.0]}"

# torso — 注意 Joint_Base_to_Body1 上限 0.3 rad！
ros2 topic pub --rate 20 /torso_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [-100.0, 0.0, 0.0, 0.0]}"
```

### 获取广义坐标
```bash
ros2 topic echo /joint_states --once
```
所有 30 个关节的 position / velocity / effort 均在 `/joint_states` 中。prismatic 关节的 NaN 已被 `joint_state_filter` 处理为 0.0，过滤后数据在 `/joint_states_filtered`。

---

## 已知弯路 / 常见陷阱

### 1. WSL 路径转换（MSYS2_ARG_CONV_EXCL）
**问题**: Git Bash 自动将 `/home/...` 转换为 `C:/Program Files/Git/home/...`
**解决**: 所有 `wsl.exe` 调用前加 `MSYS2_ARG_CONV_EXCL="*"`

### 2. gzserver 生命周期
**问题**: `ros2 launch` 退出后 `gzserver` 仍在后台运行，再次启动会冲突（`Entity already exists`）
**解决**: 重启前清理：
```bash
pkill -f gzserver; pkill -f gzclient
```

### 3. 控制器加载慢（WSL 性能）
**问题**: WSL 下 Gazebo 启动到控制器 active 需要 30-60s，而非文档写的 10-15s
**解决**: 启动后使用 `ros2 control list_controllers` 轮询直到全部 active，不能抢跑

### 4. Joint_Base_to_Body1 限位
**问题**: 限位 [-0.3, 0.3] rad，初始位置就是 0.3，正向力矩推不动
**解决**: 调试时先用负向力矩离开限位

### 5. effort 命令需要持续发送
**问题**: ForwardCommandController 虽然保持最后值，但单次 `--once` 在重力/碰撞下可能不足以产生可见运动
**解决**: 用 `--rate 20` 持续发送

### 6. JointState 解析
**问题**: `ros2 topic echo /joint_states` 输出格式为 YAML-like 多行文本，pipe 给 grep 不容易提取
**解决**: 用 Python 写 rclpy 节点直接订阅解析，或使用 `ros2 topic echo --field data`

### 7. 棱柱关节 NaN
**问题**: GazeboSystem 在 spawn 暂停阶段将 prismatic joint 位置初始化为 NaN，导致 `TF_NAN` 刷屏
**解决**: `joint_state_filter` 节点将 NaN → 0.0，已集成到 `gazebo_effort.launch.py`

### 8. ros2 topic echo 输出被文件重定向截断
**问题**: `timeout 3 ros2 topic echo /joint_states --once > file.txt 2>&1` 可能输出空文件，因为 timeout 在消息到达前就结束了
**解决**: timeout 给足 5s，或使用 `ros2 topic echo --once` 不加 timeout（在后台运行时使用）

### 9. gzserver 与 spawner 的时序
**问题**: spawner 在 Gazebo 完全加载前启动会连不上 controller_manager 服务
**解决**: launch 文件中使用 `TimerAction(period=8.0)` 延迟 spawner 启动

### 10. control_sim 必须经 launch 启动
**问题**: 直接 `ros2 run`（不经 launch）时 `config/*.yaml` 未加载，位置关节按默认限位 [0,0] 且限位开启，所有位置关节被钳死到 0
**解决**: 始终用 `ros2 launch ylr1d_control_sim position_simulate.launch.py` 启动；改参数需改 `config/*.yaml` 后重启节点（运行中 `ros2 param set` 不生效）

### 11. pkill -f 会自匹配（WSL bash）
**问题**: `pkill -f chassis_simulate` 会匹配到 bash 自身命令行里的同名模式，把执行 shell 杀掉（exit 15）
**解决**: 用字符类技巧 `pkill -f "[c]hassis_simulate"`，或先 `ps` 确认 PID 再精确 kill
