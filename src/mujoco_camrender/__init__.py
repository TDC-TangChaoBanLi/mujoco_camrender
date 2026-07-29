"""
mujoco_camrender — MuJoCo 多相机并行渲染 Python API
"""

import os as _os
from typing import Optional, List

_module_dir = _os.path.dirname(_os.path.abspath(__file__))

try:
    from . import _mujoco_camrender
    _HAS_NATIVE = True
except ImportError:
    _HAS_NATIVE = False

if _HAS_NATIVE:
    CameraParams  = _mujoco_camrender.CameraParams
    RenderMode    = _mujoco_camrender.RenderMode
    RenderBackend = _mujoco_camrender.RenderBackend
    GLRenderer    = _mujoco_camrender.GLRenderer
    RenderPool    = _mujoco_camrender.RenderPool
    AsyncEngine   = _mujoco_camrender.AsyncEngine


def _ptr(obj) -> int:
    """获取 mujoco 结构体的 C 指针"""
    if hasattr(obj, '_address'):
        return obj._address
    # pybind11 对象：通过 ctypes 读取对象内存起始处的指针
    import ctypes
    return ctypes.cast(id(obj), ctypes.POINTER(ctypes.c_void_p))[0]  # pybind11 vtable → value_ptr


class MultiCameraRenderer:
    """多相机并行渲染器"""

    def __init__(self, model, data, num_threads: int = 2, backend: str = "opengl"):
        if not _HAS_NATIVE:
            raise RuntimeError("C++ 绑定未构建，请用 CMake 编译")
        be = {"opengl": RenderBackend.OpenGL, "filament": RenderBackend.Filament}
        self._pool = RenderPool(num_threads, be.get(backend, RenderBackend.OpenGL))
        self._pool.initialize(_ptr(model), _ptr(data))

    def update_scene(self, data): self._pool.update_scene(_ptr(data))
    def render(self, cam_id, mode=None):
        if mode is None: mode = RenderMode.All
        r = self._pool.render_all([cam_id], mode)
        return r[0] if r else {}
    def render_all(self, cam_ids=None, mode=None):
        if cam_ids is None: cam_ids = list(range(4))
        if mode is None: mode = RenderMode.RGB
        return self._pool.render_all(cam_ids, mode)
    def set_camera_params(self, cam_id, params): self._pool.set_camera_params(cam_id, params)
    @property
    def thread_count(self): return self._pool.thread_count()
