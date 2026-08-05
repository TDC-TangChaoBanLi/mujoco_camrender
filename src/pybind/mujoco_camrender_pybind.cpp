#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "mujoco_camrender/gl_renderer.hpp"
#include "mujoco_camrender/render_pool.hpp"
#include "mujoco_camrender/async_engine.hpp"
#include "mujoco_camrender/types.hpp"

namespace py = pybind11;
using namespace mujoco_camrender;

// =============================================================================
// 辅助：RenderOutput → Python dict（含 numpy 数组，拷贝数据保证生命周期）
// =============================================================================
static py::dict render_output_to_dict(const RenderOutput& out) {
    py::dict d;
    d["width"]     = out.width;
    d["height"]    = out.height;
    d["camera_id"] = out.camera_id;

    if (!out.rgb.empty()) {
        py::array_t<uint8_t> arr({out.height, out.width, 3});
        std::memcpy(arr.mutable_data(), out.rgb.data(), out.rgb.size());
        d["rgb"] = std::move(arr);
    }
    if (!out.depth.empty()) {
        py::array_t<float> arr({out.height, out.width});
        std::memcpy(arr.mutable_data(), out.depth.data(),
                    out.depth.size() * sizeof(float));
        d["depth"] = std::move(arr);
    }
    return d;
}

// =============================================================================
// 模块定义
// =============================================================================
PYBIND11_MODULE(_mujoco_camrender, m) {
    m.doc() = "MuJoCo multi-camera parallel rendering engine";

    // --- 枚举 ---
    py::enum_<RenderMode>(m, "RenderMode")
        .value("RGB",   RenderMode::RGB)
        .value("Depth", RenderMode::Depth)
        .value("All",   RenderMode::All)
        .export_values();

    py::enum_<RenderBackend>(m, "RenderBackend")
        .value("OpenGL",   RenderBackend::OpenGL)
        .value("Filament", RenderBackend::Filament)
        .export_values();

    // --- CameraParams ---
    py::class_<CameraParams>(m, "CameraParams")
        .def(py::init<>())
        .def_readwrite("name",         &CameraParams::name)
        .def_readwrite("width",        &CameraParams::width)
        .def_readwrite("height",       &CameraParams::height)
        .def_readwrite("fov",          &CameraParams::fov)
        .def_readwrite("orthographic", &CameraParams::orthographic)
        .def_readwrite("near_clip",    &CameraParams::near_clip)
        .def_readwrite("far_clip",     &CameraParams::far_clip)
        .def_readwrite("cam_type",     &CameraParams::cam_type)
        .def_readwrite("fixedcamid",   &CameraParams::fixedcamid)
        .def_readwrite("trackbodyid",  &CameraParams::trackbodyid)
        .def_readwrite("far_clip",     &CameraParams::far_clip)
        .def_property("position",
            [](const CameraParams& p) { return py::make_tuple(p.position[0], p.position[1], p.position[2]); },
            [](CameraParams& p, py::tuple t) {
                p.position[0] = t[0].cast<float>();
                p.position[1] = t[1].cast<float>();
                p.position[2] = t[2].cast<float>();
            })
        .def_property("lookat",
            [](const CameraParams& p) { return py::make_tuple(p.lookat[0], p.lookat[1], p.lookat[2]); },
            [](CameraParams& p, py::tuple t) {
                p.lookat[0] = t[0].cast<float>();
                p.lookat[1] = t[1].cast<float>();
                p.lookat[2] = t[2].cast<float>();
            });

    // --- GLRenderer ---
    py::class_<GLRenderer>(m, "GLRenderer")
        .def(py::init<>())
        .def("initialize", [](GLRenderer& self, uintptr_t model_ptr, uintptr_t data_ptr) {
            self.initialize(reinterpret_cast<mjModel_*>(model_ptr),
                            reinterpret_cast<mjData_*>(data_ptr));
        }, py::arg("model_ptr"), py::arg("data_ptr"))
        .def("update_scene", [](GLRenderer& self, uintptr_t data_ptr) {
            self.update_scene(reinterpret_cast<mjData_*>(data_ptr));
        }, py::arg("data_ptr"))
        .def("render", [](GLRenderer& self, int cam_id, RenderMode mode) {
            return render_output_to_dict(self.render(cam_id, mode));
        }, py::arg("camera_id"), py::arg("mode") = RenderMode::All)
        .def("camera_count", &GLRenderer::camera_count)
        .def("set_camera_params", &GLRenderer::set_camera_params)
        .def("reset_camera_params", &GLRenderer::reset_camera_params);

    // --- RenderPool ---
    py::class_<RenderPool>(m, "RenderPool")
        .def(py::init<int, RenderBackend>(),
             py::arg("num_threads"), py::arg("backend") = RenderBackend::OpenGL)
        .def("initialize", [](RenderPool& self, uintptr_t model_ptr, uintptr_t data_ptr) {
            self.initialize(reinterpret_cast<mjModel_*>(model_ptr),
                            reinterpret_cast<mjData_*>(data_ptr));
        }, py::arg("model_ptr"), py::arg("data_ptr"))
        .def("render_all", [](RenderPool& self, const std::vector<int>& cam_ids, RenderMode mode) {
            auto results = self.render_all(cam_ids, mode);
            py::list out;
            for (auto& r : results)
                out.append(render_output_to_dict(r));
            return out;
        }, py::arg("camera_ids"), py::arg("mode") = RenderMode::RGB)
        .def("update_scene", [](RenderPool& self, uintptr_t data_ptr) {
            self.update_scene(reinterpret_cast<mjData_*>(data_ptr));
        }, py::arg("data_ptr"))
        .def("set_camera_params", &RenderPool::set_camera_params)
        .def("thread_count", &RenderPool::thread_count);

    // --- AsyncEngine ---
    py::class_<AsyncEngine>(m, "AsyncEngine")
        .def(py::init<>())
        .def("start", [](AsyncEngine& self, uintptr_t model_ptr, uintptr_t data_ptr,
                         float framerate, RenderBackend backend, int num_threads) {
            self.start(reinterpret_cast<mjModel_*>(model_ptr),
                       reinterpret_cast<mjData_*>(data_ptr),
                       framerate, backend, num_threads);
        }, py::arg("model_ptr"), py::arg("data_ptr"),
           py::arg("framerate") = 0.0f, py::arg("backend") = RenderBackend::OpenGL,
           py::arg("num_threads") = 2)
        .def("stop", &AsyncEngine::stop)
        .def("step_simulation", &AsyncEngine::step_simulation)
        .def("get_latest_frame", [](AsyncEngine& self, int cam_id) -> py::object {
            auto frame = self.get_latest_frame(cam_id);
            if (!frame || frame->rgb.empty()) return py::none();
            return render_output_to_dict(*frame);
        }, py::arg("camera_id"))
        .def("set_camera_params", &AsyncEngine::set_camera_params)
        .def("running", &AsyncEngine::running)
        .def("frame_count", &AsyncEngine::frame_count);
}
