# ylr1d_test — YLR1D 功能测试包

统一的功能测试管理：串行调度各层冒烟测试，结果落盘 `test_results/`，失败时给出「问题信息 + 解决方案建议」并支持**分层定位**（全流程失败自动补跑对应层测试）。独立 launch、独立运行，**不并入 bringup**。

## 测试清单

| ID | Tier | 依赖 Gazebo | 说明 | 耗时量级 |
|----|------|:-----------:|------|----------|
| `env` | Tier0 | 否 | 环境检查：工具链 / Python 依赖 / 工作空间 / 配置完整性 / 残留进程 / WSL / 软渲染 | 秒级 |
| `desc` | Tier1 | 否 | description 静态测试：xacro → URDF 导入、30 受控关节、关键限位、ray/imu 传感器插件 | 秒级 |
| `control_sim` | Tier1 | 否 | 控制层冒烟（迁移自 position_simulate_smoke_test）：pid 参数、命令话题长度、Body1 钳制、轮速限幅 | ~30s |
| `hmi` | Tier1 | 否 | HMI 三面板 offscreen 启动存活（monitor / sensor / control） | ~20s |
| `plant` | Tier2 | 是 | 物理层：plant_stack launch → 6 控制器 active + /joint_states 有数据 + gzserver 存活 | ~40s |
| `translate` | Tier2 | 是 | 转译层：3 action server 在线 + 发 gripper_move goal → /desired_joint_states 收到夹指目标 | ~10s（不含启动） |
| `sensor` | Tier2 | 是 | 传感器探测（sensors_test.world）：5 个 LaserScan 话题 60s 内收到数据。**默认集合不含**，显式 `--tests sensor` 或 `--all` 才跑 | ≤60s |
| `full_flow` | Tier2 | 是 | 全流程端到端 + 内置分层隔离诊断（见下文） | ~70s |

## 使用方法

```bash
# 方式一：入口脚本（自动定位 WS 根并 source，推荐）
src/ylr1d_tools/scripts/run_tests.sh --tests env,desc,control_sim,hmi
src/ylr1d_tools/scripts/run_tests.sh --all          # 全部（含默认跳过的 sensor）
src/ylr1d_tools/scripts/run_tests.sh --skip-gazebo  # 跳过需 Gazebo 的测试

# 方式二：ros2 run（需已 source install/setup.bash）
ros2 run ylr1d_test ylr1d-test --tests env

# 方式三：python 模块（需已 source install/setup.bash）
python3 -m ylr1d_test.run_all --tests env
```

**参数**：
- `--tests <id,...>`：逗号分隔的测试 id（默认 `env,desc,control_sim,hmi`）
- `--all`：运行全部测试；显式 `--tests sensor` 亦可单独跑（sensor 仅默认集合时不跑）
- `--skip-gazebo`：跳过需 Gazebo 的测试（plant / translate / full_flow / sensor）
- `--out <dir>`：结果目录（默认 `<WS_ROOT>/test_results/`）

## 结果落盘

```
<WS_ROOT>/test_results/<YYYYmmdd_HHMMSS>/
├── summary.json        # 结构化汇总（tests / skipped / cleanup_check）
├── summary.txt         # 人类可读汇总
└── logs/<test>.log     # 每个测试的完整日志

<WS_ROOT>/test_results/latest/   # 固定指向最近一次运行
```

任一测试 FAIL → 入口返回码非 0。`test_results/` 已加入根 `.gitignore`。

## 失败诊断

每个测试日志与汇总只给 **PASS / FAIL / WARN** 逐项结果；FAIL 时 runner 自动对照 `known_issues.py` 输出「可能原因 + 解决方案建议」（如 gzserver 残留、控制器加载慢、X server 缺失等，对应 CLAUDE.md 已知陷阱）。建议的解决方案可执行，但排查优先看 `logs/<test>.log`。

## 分层定位（full_flow）

`full_flow` 按证据链逐层排查，失败时返回具体故障层，runner **自动补跑**该层对应测试：

| 证据 | 条件 | 失败定位层 → 补跑 |
|------|------|-------------------|
| E1 | 6 控制器 active | `plant` → plant |
| E2 | /joint_states 有数据 | `plant` → plant |
| E3 | 3 action server 在线 | `translate` → translate |
| E4 | chassis_move goal 后 desired 有期望 | `translate` → translate |
| E5 | 5 命令话题在发 | `control_sim` → control_sim |
| E6 | 运动断言（转向/轮速） | E1-E5 全过时提示「物理/时序/环境层」深查 |

## 清理核查

- 每个测试**独立进程组**启动、结束即 kill（`os.setsid` + `os.killpg`）；
- 每个测试**前后** `cleanup_residual` 清理残留（字符类 `pkill` 避免自匹配）；
- 全部跑完后**最终核查**：`pgrep` 确认无 gzserver / 控制器 / translate / hmi 等残留，`summary` 给出 `cleanup_check: PASS（无残留）/ WARN（已清理 N 个）/ FAIL（仍有残留）`。

## 独立 launch

`ylr1d_test` 自带测试 launch（不复用、不修改 bringup）：
- `launch/plant_stack.launch.py`：plant 单层（复制 gazebo.launch.py 结构、去掉 rviz2、world 参数透传）
- `launch/test_stack.launch.py`：全栈（plant_stack + position_simulate + translate），供 translate / full_flow 使用

## 约定

- 新增/改动功能包时，同步在本包补充或更新对应测试（Tier0 env 检查项、Tier1 冒烟、Tier2 集成）。
- 需 ROS2 的测试**独立 launch，不并入 bringup**（bringup 保持一行不改）。
- 测试之间互不干扰：独立进程组 + 前后清理 + 结束核查无残留。
