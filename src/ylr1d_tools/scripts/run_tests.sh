#!/usr/bin/env bash
# ylr1d_test 一键入口：自动定位工作空间根 → source install/setup.bash → 运行测试。
#
# 用法（在工作空间任意目录）:
#   src/ylr1d_tools/scripts/run_tests.sh --tests env
#   src/ylr1d_tools/scripts/run_tests.sh --all
#   src/ylr1d_tools/scripts/run_tests.sh --skip-gazebo
set -eo pipefail

# 脚本自身定位 WS 根（src/ylr1d_tools/scripts/run_tests.sh -> WS_ROOT）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
cd "${WS_ROOT}"

if [[ ! -f install/setup.bash ]]; then
    echo "错误: ${WS_ROOT}/install/setup.bash 不存在。请先构建工作空间:" >&2
    echo "  cd ${WS_ROOT} && source /opt/ros/humble/setup.bash && colcon build" >&2
    exit 1
fi

source install/setup.bash
exec python3 -m ylr1d_test.run_all "$@"
