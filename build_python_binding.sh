#!/bin/bash
# build_python_binding.sh — 构建 Python 绑定的 .so 文件
#
# 用法:
#   ./build_python_binding.sh [MUJOCO_DIR]
#
# 默认 MUJOCO_DIR 为 ~/Softwares/mujoco-3.11.0
#
# 前提:
#   1. 已运行 uv sync 安装 Python 依赖
#   2. 已构建 C++ 库 (cd build && cmake .. -DMUJOCO_DIR=... && make)
#   3. 系统中已安装 pybind11 (pip install pybind11)

set -euo pipefail

MUJOCO_DIR="${1:-$HOME/Softwares/mujoco-3.11.0}"
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"

# 查找 Python 3.12 路径（uv 管理）
PYTHON_BIN="$(uv run python3 -c 'import sys; print(sys.executable)')"
PYTHON_VER="$(uv run python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
PYTHON_INCLUDE="$(uv run python3 -c 'import sysconfig; print(sysconfig.get_config_var("INCLUDEPY"))')"
PYTHON_LIBDIR="$(uv run python3 -c 'import sysconfig; print(sysconfig.get_config_var("LIBDIR"))')"
PYBIND11_INCLUDE="$(uv run python3 -c 'import pybind11; print(pybind11.get_include())')"

echo "Python:    $PYTHON_BIN ($PYTHON_VER)"
echo "Include:   $PYTHON_INCLUDE"
echo "Lib dir:   $PYTHON_LIBDIR"
echo "pybind11:  $PYBIND11_INCLUDE"
echo "MuJoCo:    $MUJOCO_DIR"

# 构建 .so
SO_FILE="src/mujoco_camrender/_mujoco_camrender.cpython-${PYTHON_VER/./}-x86_64-linux-gnu.so"

g++ -std=c++17 -O2 -shared -fPIC \
    -I "$PROJECT_ROOT/include" \
    -I "$MUJOCO_DIR/include" \
    -I "$PYTHON_INCLUDE" \
    -I "$PYBIND11_INCLUDE" \
    "$PROJECT_ROOT/src/pybind/mujoco_camrender_pybind.cpp" \
    -L "$PROJECT_ROOT/build/src" -lmujoco_camrender \
    -L "$MUJOCO_DIR/lib" -lmujoco \
    -L "$PYTHON_LIBDIR" -lpython${PYTHON_VER} \
    -lglfw -lOpenGL -lpthread \
    -Wl,-rpath,"$PROJECT_ROOT/build/src:$MUJOCO_DIR/lib:$PYTHON_LIBDIR" \
    -o "$PROJECT_ROOT/$SO_FILE"

echo ""
echo "✅ Python binding built: $SO_FILE"
echo ""
echo "Verify: uv run python3 -c 'from mujoco_camrender import GLRenderer; print(\"OK\")'"
