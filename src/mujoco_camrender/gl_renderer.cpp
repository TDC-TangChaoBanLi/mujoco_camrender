#include "mujoco_camrender/gl_renderer.hpp"

// MuJoCo C headers
#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace mujoco_camrender {

// =============================================================================
// PIMPL: 隐藏 MuJoCo 内部类型
// =============================================================================
struct GLRenderer::Impl {
    mjrContext con;             // MuJoCo OpenGL 渲染上下文
    mjvScene scn;               // 抽象场景
    mjvOption opt;              // 可视化选项
    std::vector<mjvCamera> cams; // 每个相机一个 mjvCamera
    std::vector<CameraParams> cam_params;     // 当前相机参数
    std::vector<CameraParams> cam_defaults;   // 默认相机参数（来自 XML）
    int max_geom = 2000;
    int offscreen_width = 640;
    int offscreen_height = 480;
    bool initialized = false;
    mjModel* model = nullptr;
    mjData* last_data = nullptr;  // 用于 render() 中按相机重更新场景

    ~Impl() {
        if (initialized) {
            mjv_freeScene(&scn);
            mjr_freeContext(&con);
        }
    }
};

// =============================================================================
// 辅助：垂直翻转图像数据（OpenGL bottom-left → 标准 top-left）
// =============================================================================
template <typename T>
static void flip_vertical(T* data, int w, int h, int channels) {
    int row_bytes = w * channels * static_cast<int>(sizeof(T));
    std::vector<uint8_t> row_buf(row_bytes);
    for (int i = 0; i < h / 2; i++) {
        auto* top = reinterpret_cast<uint8_t*>(data + i * w * channels);
        auto* bot = reinterpret_cast<uint8_t*>(data + (h - 1 - i) * w * channels);
        std::memcpy(row_buf.data(), top, row_bytes);
        std::memcpy(top, bot, row_bytes);
        std::memcpy(bot, row_buf.data(), row_bytes);
    }
}

// =============================================================================
// 辅助：将 CameraParams 应用到 mjvCamera
// =============================================================================
static void params_to_mjv_camera(const CameraParams& p, mjvCamera& cam) {
    // 先恢复默认值
    mjv_defaultCamera(&cam);

    cam.orthographic = p.orthographic ? 1 : 0;

    if (p.cam_type == mjCAMERA_FIXED) {
        // 固定相机：mjv_updateScene 会从模型读取 cam_pos/cam_quat
        cam.type = mjCAMERA_FIXED;
        cam.fixedcamid = p.fixedcamid;
    } else if (p.cam_type == mjCAMERA_TRACKING) {
        // 跟踪相机：mjv_updateScene 会跟踪目标 body
        cam.type = mjCAMERA_TRACKING;
        cam.trackbodyid = p.trackbodyid;
        cam.lookat[0] = p.lookat[0];
        cam.lookat[1] = p.lookat[1];
        cam.lookat[2] = p.lookat[2];
        // 相对位置通过 distance/azimuth/elevation
        double dx = p.position[0] - p.lookat[0];
        double dy = p.position[1] - p.lookat[1];
        double dz = p.position[2] - p.lookat[2];
        cam.distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        cam.azimuth = std::atan2(dx, dy) * 180.0 / M_PI;
        double h_dist = std::sqrt(dx*dx + dy*dy);
        cam.elevation = std::atan2(dz, h_dist) * 180.0 / M_PI;
    } else {
        // 自由相机：通过 lookat + distance/azimuth/elevation
        cam.lookat[0] = p.lookat[0];
        cam.lookat[1] = p.lookat[1];
        cam.lookat[2] = p.lookat[2];
        double dx = p.position[0] - p.lookat[0];
        double dy = p.position[1] - p.lookat[1];
        double dz = p.position[2] - p.lookat[2];
        cam.distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        cam.azimuth = std::atan2(dx, dy) * 180.0 / M_PI;
        double h_dist = std::sqrt(dx*dx + dy*dy);
        cam.elevation = std::atan2(dz, h_dist) * 180.0 / M_PI;
    }
}

// =============================================================================
// 辅助：从 MJCF 相机读取参数
// =============================================================================
static CameraParams read_camera_from_model(const mjModel* m, int cam_id) {
    CameraParams p;

    // 名称
    if (m->names && m->name_camadr[cam_id] >= 0) {
        p.name = m->names + m->name_camadr[cam_id];
    }

    // 分辨率
    p.width = m->cam_resolution[cam_id * 2];
    p.height = m->cam_resolution[cam_id * 2 + 1];
    if (p.width <= 0) p.width = 640;
    if (p.height <= 0) p.height = 480;

    // FOV
    p.fov = static_cast<float>(m->cam_fovy[cam_id]);

    // 正交
    p.orthographic = (m->cam_projection[cam_id] == mjPROJ_ORTHOGRAPHIC);

    // cam_mode 使用 mjtCamLight 枚举 (0=FIXED, 3=TARGETBODY)
    // 所有相机统一用 mjCAMERA_FIXED 让 mjv_updateScene 从模型读取 pos/quat
    int cam_mode = m->cam_mode[cam_id];

    if (cam_mode == mjCAMLIGHT_TARGETBODY || cam_mode == mjCAMLIGHT_TARGETBODYCOM) {
        // 跟踪目标 body 的相机
        p.cam_type = mjCAMERA_TRACKING;
        p.trackbodyid = m->cam_bodyid[cam_id];
    } else {
        // 固定相机或 track/trackcom
        p.cam_type = mjCAMERA_FIXED;
        p.fixedcamid = cam_id;
    }

    p.near_clip = 0.001f;
    p.far_clip = 100.0f;

    return p;
}

// =============================================================================
// GLRenderer
// =============================================================================

GLRenderer::GLRenderer() : impl_(std::make_unique<Impl>()) {}

GLRenderer::~GLRenderer() = default;

void GLRenderer::initialize(mjModel_* m_raw, mjData_* d_raw) {
    auto* m = static_cast<mjModel*>(m_raw);
    auto* d = static_cast<mjData*>(d_raw);
    impl_->model = m;

    // 1. 从模型加载相机参数（先读取分辨率信息）
    load_cameras_from_model(m);

    // 2. 计算所有相机中的最大分辨率，用于 GLFW 窗口
    int max_w = 640, max_h = 480;
    for (auto& p : impl_->cam_params) {
        if (p.width  > max_w) max_w = p.width;
        if (p.height > max_h) max_h = p.height;
    }

    // 3. 创建 GL 上下文（GLFW 隐藏窗口，大小匹配最大相机分辨率）
    if (!gl_ctx_.create(max_w, max_h)) {
        throw std::runtime_error("GLRenderer: failed to create GL context");
    }
    gl_ctx_.make_current();

    // 2. 创建 MuJoCo 渲染上下文
    mjr_defaultContext(&impl_->con);
    mjr_makeContext(m, &impl_->con, mjFONTSCALE_100);

    // 3. 创建抽象场景
    mjv_defaultOption(&impl_->opt);
    mjv_defaultScene(&impl_->scn);
    mjv_makeScene(m, &impl_->scn, impl_->max_geom);

    impl_->initialized = true;

    // 首次场景更新（使用 camera 0 作为初始视角）
    impl_->last_data = d_raw;
    mjv_updateScene(m, d, &impl_->opt, nullptr, &impl_->cams[0], mjCAT_ALL, &impl_->scn);

    // 释放 GL 上下文，让 worker 线程可以通过 make_current() 获取
    gl_ctx_.release_current();
}

void GLRenderer::update_scene(mjData_* d_raw) {
    if (!impl_->initialized) return;

    // 仅缓存数据指针，实际场景更新在 render() 中按相机进行
    impl_->last_data = d_raw;
}

RenderOutput GLRenderer::render(int camera_id, RenderMode mode) {
    if (!impl_->initialized) {
        throw std::runtime_error("GLRenderer: not initialized");
    }
    if (camera_id < 0 || camera_id >= static_cast<int>(impl_->cams.size())) {
        throw std::runtime_error("GLRenderer: invalid camera_id " +
                                 std::to_string(camera_id));
    }

    gl_ctx_.make_current();

    const CameraParams& params = impl_->cam_params[camera_id];
    int w = params.width;
    int h = params.height;

    // 如果需要，调整离屏缓冲区大小
    if (w != impl_->offscreen_width || h != impl_->offscreen_height) {
        mjr_resizeOffscreen(w, h, &impl_->con);
        impl_->offscreen_width = w;
        impl_->offscreen_height = h;
    }

    // 用当前相机视角重新更新场景
    if (impl_->last_data) {
        auto* d = static_cast<mjData*>(impl_->last_data);
        mjv_updateScene(impl_->model, d, &impl_->opt, nullptr,
                        &impl_->cams[camera_id], mjCAT_ALL, &impl_->scn);
    }

    // 渲染到离屏缓冲区
    mjr_setBuffer(mjFB_OFFSCREEN, &impl_->con);
    mjrRect viewport = {0, 0, w, h};
    mjr_render(viewport, &impl_->scn, &impl_->con);

    // 读取像素（OpenGL 原点在左下角，需垂直翻转）
    RenderOutput output;
    output.width = w;
    output.height = h;
    output.camera_id = camera_id;

    if (mode == RenderMode::RGB || mode == RenderMode::All) {
        output.rgb.resize(w * h * 3);
        mjr_readPixels(output.rgb.data(), nullptr, viewport, &impl_->con);
        // 垂直翻转：OpenGL bottom-left → 图像 top-left
        flip_vertical(output.rgb.data(), w, h, 3);
    }
    if (mode == RenderMode::Depth || mode == RenderMode::All) {
        output.depth.resize(w * h);
        mjr_readPixels(nullptr, output.depth.data(), viewport, &impl_->con);
        flip_vertical(output.depth.data(), w, h, 1);
    }

    return output;
}

int GLRenderer::camera_count() const {
    return static_cast<int>(impl_->cam_params.size());
}

void GLRenderer::set_camera_params(int cam_id, const CameraParams& params) {
    if (cam_id < 0 || cam_id >= static_cast<int>(impl_->cam_params.size())) return;

    // 若调用方未显式指定相机类型（默认 FREE + fixedcamid=-1 + trackbodyid=-1），
    // 视为「部分覆盖」（如只改分辨率），保留原相机的类型 / 固定索引 / 跟踪目标。
    CameraParams merged = params;
    if (params.cam_type == mjCAMERA_FREE &&
        params.fixedcamid == -1 &&
        params.trackbodyid == -1) {
        const CameraParams& old = impl_->cam_params[cam_id];
        merged.cam_type = old.cam_type;
        merged.fixedcamid = old.fixedcamid;
        merged.trackbodyid = old.trackbodyid;
    }

    impl_->cam_params[cam_id] = merged;
    // 同步到 mjvCamera
    params_to_mjv_camera(merged, impl_->cams[cam_id]);
}

void GLRenderer::reset_camera_params(int cam_id) {
    if (cam_id < 0 || cam_id >= static_cast<int>(impl_->cam_defaults.size())) return;
    impl_->cam_params[cam_id] = impl_->cam_defaults[cam_id];
    params_to_mjv_camera(impl_->cam_defaults[cam_id], impl_->cams[cam_id]);
}

void GLRenderer::set_resolution(int width, int height) {
    if (!impl_->initialized) return;
    gl_ctx_.make_current();
    mjr_resizeOffscreen(width, height, &impl_->con);
    impl_->offscreen_width = width;
    impl_->offscreen_height = height;
}

// ---------------------------------------------------------------------------
// Private
// ---------------------------------------------------------------------------

void GLRenderer::load_cameras_from_model(mjModel_* m_raw) {
    auto* m = static_cast<mjModel*>(m_raw);
    int ncam = m->ncam;

    impl_->cams.resize(ncam);
    impl_->cam_params.resize(ncam);
    impl_->cam_defaults.resize(ncam);

    for (int i = 0; i < ncam; i++) {
        // 读取模型中的相机参数
        CameraParams defaults = read_camera_from_model(m, i);
        impl_->cam_defaults[i] = defaults;
        impl_->cam_params[i] = defaults;

        // 初始化 mjvCamera
        mjv_defaultCamera(&impl_->cams[i]);
        params_to_mjv_camera(defaults, impl_->cams[i]);
    }
}

void GLRenderer::apply_camera_params(int cam_id, mjData_* d) {
    // 用指定相机更新场景
    if (!impl_->initialized || !impl_->model || !d) return;
    auto* data = static_cast<mjData*>(d);
    mjv_updateScene(impl_->model, data, &impl_->opt, nullptr,
                    &impl_->cams[cam_id], mjCAT_ALL, &impl_->scn);
}

}  // namespace mujoco_camrender
