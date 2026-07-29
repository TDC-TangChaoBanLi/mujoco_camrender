"""
generate_stubs.py — 为 mujoco_camrender C++ 绑定生成 .pyi 类型提示文件

用法:
    uv run python generate_stubs.py

依赖:
    uv add --dev pybind11-stubgen
"""

import sys
import os

# 确保能找到 _mujoco_camrender.so
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src", "mujoco_camrender"))

try:
    from pybind11_stubgen import ModuleStubsGenerator

    module_name = "_mujoco_camrender"
    generator = ModuleStubsGenerator(module_name)
    generator.parse()
    generator.write()

    # 重命名输出
    out_path = os.path.join("src", "mujoco_camrender", f"{module_name}.pyi")
    print(f"✅ Type stubs written to: {out_path}")

except ImportError:
    print("pybind11-stubgen not installed. Run: uv add --dev pybind11-stubgen")
    print()
    print("Manual stub (minimal) — save this as src/mujoco_camrender/_mujoco_camrender.pyi:")
    print("""
from typing import Tuple, List, Optional, Dict
import numpy as np

class RenderMode:
    RGB: RenderMode
    Depth: RenderMode
    All: RenderMode

class RenderBackend:
    OpenGL: RenderBackend
    Filament: RenderBackend

class CameraParams:
    name: str
    width: int
    height: int
    fov: float
    orthographic: bool
    near_clip: float
    far_clip: float
    position: Tuple[float, float, float]
    lookat: Tuple[float, float, float]

class GLRenderer:
    def initialize(self, model_ptr: int, data_ptr: int) -> None: ...
    def update_scene(self, data_ptr: int) -> None: ...
    def render(self, camera_id: int, mode: RenderMode = ...) -> Dict[str, any]: ...
    def camera_count(self) -> int: ...
    def set_camera_params(self, cam_id: int, params: CameraParams) -> None: ...
    def reset_camera_params(self, cam_id: int) -> None: ...

class RenderPool:
    def __init__(self, num_threads: int, backend: RenderBackend = ...) -> None: ...
    def initialize(self, model_ptr: int, data_ptr: int) -> None: ...
    def render_all(self, camera_ids: List[int], mode: RenderMode = ...) -> List[Dict[str, any]]: ...
    def update_scene(self, data_ptr: int) -> None: ...
    def set_camera_params(self, cam_id: int, params: CameraParams) -> None: ...
    def thread_count(self) -> int: ...

class AsyncEngine:
    def start(self, model_ptr: int, data_ptr: int, framerate: float = ..., backend: RenderBackend = ..., num_threads: int = ...) -> None: ...
    def stop(self) -> None: ...
    def step_simulation(self) -> None: ...
    def get_latest_frame(self, camera_id: int) -> Optional[Dict[str, any]]: ...
    def set_camera_params(self, cam_id: int, params: CameraParams) -> None: ...
    def running(self) -> bool: ...
    def frame_count(self) -> int: ...
""")
