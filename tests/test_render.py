"""
test_render.py — mujoco_camrender Python 集成测试
使用 C++ pybind11 绑定进行渲染
"""

import os, sys, time
from pathlib import Path
import mujoco, numpy as np
from PIL import Image

from mujoco_camrender import (
    GLRenderer, RenderPool, AsyncEngine,
    CameraParams, RenderMode, RenderBackend, _ptr,
)

def save_image(rgb: np.ndarray, path: str):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    Image.fromarray(rgb).save(path)
    print(f"  Saved: {path} ({rgb.shape[1]}x{rgb.shape[0]})")

class Timer:
    def __init__(self, label: str):
        self.label = label
        self._start = time.perf_counter()
    def elapsed_ms(self) -> float:
        return (time.perf_counter() - self._start) * 1000
    def stop(self, suffix: str = ""):
        ms = self.elapsed_ms()
        print(f"[TIMER] {self.label}: {ms:.1f} ms{suffix}")
        return ms

def main():
    xml_path = sys.argv[1] if len(sys.argv) > 1 else "assets/test_scene.xml"
    output_dir = Path("output")
    output_dir.mkdir(exist_ok=True)

    print("=" * 48)
    print("  mujoco_camrender Python 集成测试")
    print("=" * 48)
    print()

    print(f"[1/5] Loading model: {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ncam = model.ncam
    print(f"  Model loaded: {ncam} cameras\n")

    # --- 测试 1：单相机同步渲染 ---
    print("[2/5] Single-camera sync render (GLRenderer)")
    renderer = GLRenderer()
    renderer.initialize(_ptr(model), _ptr(data))
    for _ in range(10): mujoco.mj_step(model, data)
    renderer.update_scene(_ptr(data))
    t = Timer("Single render x4")
    for i in range(ncam):
        renderer.render(i, RenderMode.All)
    t.stop()
    for i in range(ncam):
        out = renderer.render(i, RenderMode.All)
        save_image(out["rgb"], str(output_dir / f"cam{i}_py_rgb.png"))
        d = out["depth"]
        d_vis = ((d - d.min()) / max(d.max() - d.min(), 1e-6) * 255).astype(np.uint8)
        save_image(d_vis, str(output_dir / f"cam{i}_py_depth.png"))
    print()

    # --- 测试 2：并行多相机渲染 ---
    print("[3/5] Parallel multi-camera render (RenderPool, 4 threads)")
    pool = RenderPool(4, RenderBackend.OpenGL)
    pool.initialize(_ptr(model), _ptr(data))
    t = Timer("Parallel render x100")
    for it in range(100):
        mujoco.mj_step(model, data)
        pool.update_scene(_ptr(data))
        results = pool.render_all(list(range(ncam)), RenderMode.RGB)
        if it == 0:
            for i, out in enumerate(results):
                save_image(out["rgb"], str(output_dir / f"cam{i}_py_multi.png"))
    t.stop()
    print()

    # --- 测试 3：异步后台渲染 ---
    print("[4/5] Async background rendering (AsyncEngine, max FPS)")
    engine = AsyncEngine()
    engine.start(_ptr(model), _ptr(data), 0.0, RenderBackend.OpenGL, 4)
    frames_before = engine.frame_count()
    t = Timer("Async render 1000 steps")
    for _ in range(1000):
        engine.step_simulation()
        time.sleep(0.001)
    frames_after = engine.frame_count()
    delta = frames_after - frames_before
    t.stop(f", {delta} frames ({delta * 1000 / max(t.elapsed_ms(), 1):.1f} FPS)")
    for i in range(ncam):
        frame = engine.get_latest_frame(i)
        if frame:
            save_image(frame["rgb"], str(output_dir / f"cam{i}_py_async.png"))
        else:
            print(f"  Camera {i}: no frame available")
    engine.stop()
    print()

    # --- 测试 4：相机参数覆盖 ---
    print("[5/5] Camera parameter override")
    r2 = GLRenderer()
    r2.initialize(_ptr(model), _ptr(data))
    params = CameraParams()
    params.width = 320; params.height = 240; params.fov = 90.0
    params.position = (0.0, -2.0, 3.0); params.lookat = (0.0, 0.0, 0.5)
    r2.set_camera_params(0, params)
    r2.update_scene(_ptr(data))
    out = r2.render(0, RenderMode.RGB)
    save_image(out["rgb"], str(output_dir / "cam0_py_overridden.png"))
    print()

    print("=" * 48)
    print("  All Python tests passed!")
    print(f"  Images saved to {output_dir}/")
    print("=" * 48)

if __name__ == "__main__":
    main()
