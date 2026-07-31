# CLAUDE.md — YLR1D 项目备忘录

## 环境与执行

### WSL 执行命令
本项目在 WSL Ubuntu-22.04 中运行。从 Windows CLI 调用 WSL 时必须添加 `MSYS2_ARG_CONV_EXCL="*"` 防止 Git Bash/MSYS2 路径转换：

```bash
MSYS2_ARG_CONV_EXCL="*" wsl.exe -d Ubuntu-22.04 bash -c 'source /home/zsj/WorkSpace/test_ylr1d/install/setup.bash; ros2 ...'
```

### 环境初始化
```bash
source /home/zsj/WorkSpace/test_ylr1d/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/zsj/WorkSpace/test_ylr1d/src
```

脚本 `colcon_build.sh` 中有完整的环境配置参考。

### 注意
- WSL 下 GPU 渲染受限，Gazebo 需 `LIBGL_ALWAYS_SOFTWARE=1`
- Gazebo Classic 在 WSL 下启动很慢（~30-60s 控制器才完全加载），务必耐心等待 `ros2 control list_controllers` 全部显示 `active`

---

## 包结构速览

- **`ylr1d_mid_control`**: 中间层控制器，管理 Gazebo + ros2_control 的配置/启动。提供三种控制方案：position/velocity（原始）、effort（力控）、velocity（速度控制）
- **`ylr1d_description`**: URDF 模型文件、mesh 文件
- **`ylr1d_position_simulate`**: 软件仿真层，模拟硬件位置/速度闭环。订阅 `/desired_joint_states` + `/joint_states`，PID 过渡后发布 5 组 ForwardCommandController 命令话题。参数从 `config/` 三个 yaml 加载（`<关节名>/limit/*`、`<关节名>/pid/*`）。**必须通过 `position_simulate.launch.py` 启动**。无头验证：`test/position_simulate_smoke_test.py`
- **`ylr1d_hmi`**: Qt5 人机交互界面，关节状态观测 + 关节控制器。Lite 版（`hmi.launch.py`）正常，RViz2 版存在构建/运行问题

> 注：旧的 `ylr1d_base_control`（PD 节点 + `joint_state_filter`）已不在 `src/` 下；`joint_state_filter`（NaN → 0.0）现集成在 `ylr1d_mid_control` 的 `gazebo_effort.launch.py` 中（见陷阱 7）。

---

## 仿真验证 — 力控测试

### 启动
```bash
ros2 launch ylr1d_mid_control gazebo_effort.launch.py
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
**问题**: Git Bash 自动将 `/home/zsj/...` 转换为 `C:/Program Files/Git/home/zsj/...`
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

### 10. position_simulate 必须经 launch 启动
**问题**: 直接 `ros2 run`（不经 launch）时 `config/*.yaml` 未加载，位置关节按默认限位 [0,0] 且限位开启，所有位置关节被钳死到 0
**解决**: 始终用 `ros2 launch ylr1d_position_simulate position_simulate.launch.py` 启动；改参数需改 `config/*.yaml` 后重启节点（运行中 `ros2 param set` 不生效）

### 11. pkill -f 会自匹配（WSL bash）
**问题**: `pkill -f chassis_simulate` 会匹配到 bash 自身命令行里的同名模式，把执行 shell 杀掉（exit 15）
**解决**: 用字符类技巧 `pkill -f "[c]hassis_simulate"`，或先 `ps` 确认 PID 再精确 kill
