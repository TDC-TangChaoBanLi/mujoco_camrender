# mujoco-camrender

MuJoCo 多相机并行离屏渲染引擎。支持 OpenGL 后端，提供 C++ 库和 Python 绑定。

## 特性

- 🎥 **多相机并行渲染** — 每个相机独立线程，4 相机 4 线程可达 300+ FPS
- 🖥️ **GPU 加速** — 基于 GLFW 隐藏窗口 + NVIDIA/AMD OpenGL 驱动
- 🔄 **异步模式** — 后台持续渲染，主线程非阻塞获取最新帧
- 🐍 **Python 绑定** — pybind11 绑定，numpy 数组输出
- 📦 **零依赖安装** — `uv sync` 一键安装，`pip install` 也可用

## 依赖

| 依赖 | 版本 | 用途 |
|------|------|------|
| MuJoCo | ≥ 3.11 | 物理引擎 + 渲染核心 |
| GLFW | ≥ 3.3 | OpenGL 上下文管理 |
| OpenGL | — | GPU 渲染 |
| CMake | ≥ 3.16 | C++ 构建 |
| Python | ≥ 3.12 | Python 绑定 |
| pybind11 | ≥ 3.0 | Python 绑定 |
| numpy | ≥ 2.0 | 数组输出 |

### Fedora

```bash
sudo dnf install cmake gcc-c++ glfw-devel mesa-libGL-devel
```

### Ubuntu/Debian

```bash
sudo apt install cmake g++ libglfw3-dev libgl-dev
```

## 快速开始

### 1. 安装 MuJoCo

```bash
# 下载预编译版本
wget https://github.com/google-deepmind/mujoco/releases/download/3.11.0/mujoco-3.11.0-linux-x86_64.tar.gz
# 解压到 ~/Softwares/ 目录下
tar -xzf mujoco-3.11.0-linux-x86_64.tar.gz -C ~/Softwares/
```

### 2. 克隆项目

```bash
git clone <repo-url>
cd mujoco-camrender
```

### 3. 安装 Python 依赖

```bash
uv sync
# 或: pip install -e .[dev]
```

### 4. 构建 C++ 库

```bash
mkdir -p build && cd build
cmake .. -DMUJOCO_DIR=$HOME/Softwares/mujoco-3.11.0
make -j$(nproc)
```

### 5. 构建 Python 绑定

```bash
./build_python_binding.sh $HOME/Softwares/mujoco-3.11.0
```

### 6. 运行测试

```bash
# C++ 测试
LD_LIBRARY_PATH=$HOME/Softwares/mujoco-3.11.0/lib ./build/test_render

# Python 测试
LD_LIBRARY_PATH=$HOME/Softwares/mujoco-3.11.0/lib:$PWD/build/src \
  uv run python tests/test_render.py
```

## 架构

```
┌──────────────────────────────────────────┐
│               Python API                  │
│  MultiCameraRenderer  (高层封装)          │
├──────────────────────────────────────────┤
│          _mujoco_camrender.so             │
│  GLRenderer / RenderPool / AsyncEngine    │
├──────────────────────────────────────────┤
│         libmujoco_camrender.so            │
│  GLContext / GLRenderer / RenderPool      │
├──────────────────────────────────────────┤
│       libmujoco.so + GLFW + OpenGL        │
└──────────────────────────────────────────┘
```

## Python API

```python
import mujoco
from mujoco_camrender import (
    GLRenderer, RenderPool, AsyncEngine,
    CameraParams, RenderMode, RenderBackend, _ptr,
)

# 加载模型
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

# --- 方式 1: 单相机同步渲染 ---
renderer = GLRenderer()
renderer.initialize(_ptr(model), _ptr(data))
renderer.update_scene(_ptr(data))
output = renderer.render(0, RenderMode.All)
rgb = output["rgb"]      # numpy (H, W, 3) uint8
depth = output["depth"]  # numpy (H, W) float32

# --- 方式 2: 多相机并行渲染 ---
pool = RenderPool(num_threads=4, backend=RenderBackend.OpenGL)
pool.initialize(_ptr(model), _ptr(data))
pool.update_scene(_ptr(data))
results = pool.render_all([0, 1, 2, 3], RenderMode.RGB)

# --- 方式 3: 异步后台渲染 ---
engine = AsyncEngine()
engine.start(_ptr(model), _ptr(data), framerate=0,   # 0 = max FPS
             backend=RenderBackend.OpenGL, num_threads=4)
for _ in range(100):
    engine.step_simulation()          # 推进物理
    frame = engine.get_latest_frame(0)  # 非阻塞获取
engine.stop()
```

## 在其他项目中使用

### 通过 uv（推荐）

在目标项目的 `pyproject.toml` 中添加：

```toml
[tool.uv.sources]
mujoco-camrender = { path = "/path/to/mujoco-camrender" }

[project]
dependencies = ["mujoco-camrender"]
```

### 通过 pip

```bash
pip install -e /path/to/mujoco-camrender
```

### 配置 LD_LIBRARY_PATH

Python 绑定需要能找到 `libmujoco.so` 和 `libmujoco_camrender.so`：

```bash
export LD_LIBRARY_PATH=/path/to/mujoco/lib:/path/to/mujoco-camrender/build/src:$LD_LIBRARY_PATH
```

或者写入 `.env`：
```
LD_LIBRARY_PATH=/opt/mujoco/lib:/opt/mujoco-camrender/build/src
```

## 类型提示

项目提供了 `.pyi` 类型存根文件，IDE 可自动补全：

```python
# 已内置在 src/mujoco_camrender/_mujoco_camrender.pyi

# 如需重新生成（需要安装 pybind11-stubgen）:
uv add --dev pybind11-stubgen
uv run python generate_stubs.py
```

## CMake 选项

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `MUJOCO_DIR` | **必填** | MuJoCo 安装路径 |
| `CMAKE_BUILD_TYPE` | — | `Release` / `Debug` |

## 项目结构

```
.
├── CMakeLists.txt              # C++ 构建
├── pyproject.toml              # Python 包配置
├── build_python_binding.sh     # Python 绑定构建脚本
├── generate_stubs.py           # 类型提示生成
├── include/mujoco_camrender/   # C++ 头文件
├── src/
│   ├── mujoco_camrender/       # C++ 源码 + Python 包
│   └── pybind/                 # pybind11 绑定
├── tests/                      # C++ 和 Python 测试
└── assets/                     # MuJoCo 场景文件
```

## 许可证

MIT
