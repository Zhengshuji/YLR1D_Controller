# ylr1d_tools 工具目录

辅助工具容器（**非 ROS2 package**）：收敛本项目开发辅助工具，当前含**功能测试包 `ylr1d_test`** 与统一入口脚本。未来新工具（如控制器解耦、模型参数提取）也放于此目录。

## 目录结构

```
ylr1d_tools/
├── README.md           # 本总览：定位 + 结构 + 子 README 索引
├── scripts/
│   └── run_tests.sh    # 一键入口：自动定位 WS 根 → source install/setup.bash → 跑测试
└── ylr1d_test/         # 唯一真正的 ROS2 package（ament_python）
```

## 快速上手

```bash
# 环境测试（秒级，无需启动仿真）
src/ylr1d_tools/scripts/run_tests.sh --tests env

# 全部 Tier1 静态/冒烟测试
src/ylr1d_tools/scripts/run_tests.sh --tests desc,control_sim,hmi

# 全部测试（含需 Gazebo 的 Tier2，较慢）
src/ylr1d_tools/scripts/run_tests.sh --all
```

## 子 README 索引

- **[ylr1d_test/README.md](ylr1d_test/README.md)** —— 功能测试包详细手册：测试清单（8 个测试的 ID / Tier / 依赖 / 说明）、使用方法、结果落盘位置、失败诊断（known_issues）、分层定位机制。
