#include "mujoco_camrender/filament_renderer.hpp"

#include <mujoco/mujoco.h>
#include <mujoco/mjrfilament.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace mujoco_camrender {

// =============================================================================
// PIMPL
// =============================================================================
struct FilamentRenderer::Impl {
    mjrfContext* rf_ctx = nullptr;          // Filament 渲染上下文
    mjrfScene* scene = nullptr;             // Filament 场景
    std::vector<mjrfRenderTarget*> targets;  // 每个相机一个 RenderTarget

    // 相机参数
    std::vector<CameraParams> cam_params;
    std::vector<CameraParams> cam_defaults;

    // 清除色
    float clear_color[3] = {0.0f, 0.0f, 0.0f};

    mjModel* model = nullptr;
    bool initialized = false;

    ~Impl() {
        for (auto* t : targets) {
            if (t) mjrf_destroyRenderTarget(t);
        }
        if (scene) mjrf_destroyScene(scene);
        if (rf_ctx) mjrf_destroyContext(rf_ctx);
    }
};

// =============================================================================
// 辅助：CameraParams → mjrCamera (mjvGLCamera)
// =============================================================================
static void params_to_mjr_camera(const CameraParams& p, mjrCamera& cam) {
    // 位置
    cam.pos[0] = p.position[0];
    cam.pos[1] = p.position[1];
    cam.pos[2] = p.position[2];

    // 前向向量：从 position 指向 lookat
    float fx = p.lookat[0] - p.position[0];
    float fy = p.lookat[1] - p.position[1];
    float fz = p.lookat[2] - p.position[2];
    float len = std::sqrt(fx*fx + fy*fy + fz*fz);
    if (len > 1e-6f) {
        fx /= len; fy /= len; fz /= len;
    }
    cam.forward[0] = fx;
    cam.forward[1] = fy;
    cam.forward[2] = fz;

    // 上向量：世界 +z
    float world_up[3] = {0, 0, 1};
    // 如果前向接近 +z，使用 +y 作为上
    if (std::abs(fz) > 0.99f) {
        world_up[0] = 0; world_up[1] = 1; world_up[2] = 0;
    }

    // right = forward × up
    float right[3];
    right[0] = fy * world_up[2] - fz * world_up[1];
    right[1] = fz * world_up[0] - fx * world_up[2];
    right[2] = fx * world_up[1] - fy * world_up[0];
    float r_len = std::sqrt(right[0]*right[0] + right[1]*right[1] + right[2]*right[2]);
    right[0] /= r_len; right[1] /= r_len; right[2] /= r_len;

    // up = right × forward
    cam.up[0] = right[1] * fz - right[2] * fy;
    cam.up[1] = right[2] * fx - right[0] * fz;
    cam.up[2] = right[0] * fy - right[1] * fx;

    // 投影参数
    cam.orthographic = p.orthographic ? 1 : 0;
    cam.frustum_near = p.near_clip;
    cam.frustum_far = p.far_clip;

    // 从垂直 FOV 和宽高比计算 frustum
    float aspect = static_cast<float>(p.width) / static_cast<float>(p.height);
    float fov_rad = p.fov * static_cast<float>(M_PI) / 180.0f;
    float top = std::tan(fov_rad * 0.5f) * p.near_clip;
    float bottom = -top;

    cam.frustum_top = top;
    cam.frustum_bottom = bottom;
    cam.frustum_center = 0.0f;
    cam.frustum_width = 2.0f * top * aspect;
}

// =============================================================================
// 辅助：从 MJCF 读取（复用在 gl_renderer.cpp 中的逻辑）
// =============================================================================
static CameraParams read_camera_from_model(const mjModel* m, int cam_id) {
    CameraParams p;
    if (m->names && m->name_camadr[cam_id] >= 0) {
        p.name = m->names + m->name_camadr[cam_id];
    }
    p.width = m->cam_resolution[cam_id * 2];
    p.height = m->cam_resolution[cam_id * 2 + 1];
    if (p.width <= 0) p.width = 640;
    if (p.height <= 0) p.height = 480;
    p.fov = static_cast<float>(m->cam_fovy[cam_id]);
    p.orthographic = (m->cam_projection[cam_id] == mjPROJ_ORTHOGRAPHIC);

    int cam_type = m->cam_mode[cam_id];
    if (cam_type == mjCAMERA_FIXED) {
        p.position[0] = static_cast<float>(m->cam_pos[cam_id * 3]);
        p.position[1] = static_cast<float>(m->cam_pos[cam_id * 3 + 1]);
        p.position[2] = static_cast<float>(m->cam_pos[cam_id * 3 + 2]);

        // 从 quat 计算 forward
        double q[4] = {m->cam_quat[cam_id*4], m->cam_quat[cam_id*4+1],
                       m->cam_quat[cam_id*4+2], m->cam_quat[cam_id*4+3]};
        // 默认前向 = -z (MuJoCo 相机朝向)
        double fw[3] = {0, 0, -1};
        // 用 quat 旋转: result = q * v * q_conj
        double a  =  q[1]*fw[0] + q[2]*fw[1] + q[3]*fw[2];
        double b1 =  q[0]*fw[0] + q[2]*fw[2] - q[3]*fw[1];
        double c1 =  q[0]*fw[1] + q[3]*fw[0] - q[1]*fw[2];
        double d1 =  q[0]*fw[2] + q[1]*fw[1] - q[2]*fw[0];
        double fwd[3] = {
            static_cast<float>(b1*q[0] + a*q[1] + c1*q[3] - d1*q[2]),
            static_cast<float>(c1*q[0] + a*q[2] + d1*q[1] - b1*q[3]),
            static_cast<float>(d1*q[0] + a*q[3] + b1*q[2] - c1*q[1])
        };

        p.lookat[0] = p.position[0] + fwd[0] * 2.0f;
        p.lookat[1] = p.position[1] + fwd[1] * 2.0f;
        p.lookat[2] = p.position[2] + fwd[2] * 2.0f;
    } else {
        p.position[0] = static_cast<float>(m->cam_pos[cam_id * 3]);
        p.position[1] = static_cast<float>(m->cam_pos[cam_id * 3 + 1]);
        p.position[2] = static_cast<float>(m->cam_pos[cam_id * 3 + 2]);
        p.lookat[0] = 0; p.lookat[1] = 0; p.lookat[2] = 0.5f;
    }
    p.near_clip = 0.001f;
    p.far_clip = 100.0f;
    return p;
}

// =============================================================================
// FilamentRenderer
// =============================================================================

FilamentRenderer::FilamentRenderer() : impl_(std::make_unique<Impl>()) {}
FilamentRenderer::~FilamentRenderer() = default;

void FilamentRenderer::initialize(mjModel_* m_raw, mjData_* d_raw) {
    auto* m = static_cast<mjModel*>(m_raw);
    impl_->model = m;

    // 1. 创建 Filament 上下文（离屏模式）
    mjrfContextConfig config;
    mjrf_defaultContextConfig(&config);

    impl_->rf_ctx = mjrf_createContext(&config);
    if (!impl_->rf_ctx) {
        throw std::runtime_error("FilamentRenderer: failed to create mjrfContext");
    }

    // 2. 创建场景
    mjrfSceneParams scene_params;
    mjrf_defaultSceneParams(&scene_params);
    impl_->scene = mjrf_createScene(impl_->rf_ctx, &scene_params);
    if (!impl_->scene) {
        throw std::runtime_error("FilamentRenderer: failed to create mjrfScene");
    }

    // 3. 从模型配置场景
    mjrf_configureSceneFromModel(impl_->scene, m);

    // 4. 为每个相机创建 RenderTarget
    load_cameras_from_model(m);

    impl_->initialized = true;
}

void FilamentRenderer::update_scene(mjData_* /*d*/) {
    // Filament 场景通过 render request 中的 mjrCamera 驱动
    // 不需要像 OpenGL 那样预先更新抽象场景
}

RenderOutput FilamentRenderer::render(int camera_id, RenderMode mode) {
    if (!impl_->initialized) {
        throw std::runtime_error("FilamentRenderer: not initialized");
    }
    if (camera_id < 0 || camera_id >= static_cast<int>(impl_->cam_params.size())) {
        throw std::runtime_error("FilamentRenderer: invalid camera_id");
    }

    const CameraParams& params = impl_->cam_params[camera_id];
    int w = params.width;
    int h = params.height;

    // 确保 RenderTarget 分辨率正确
    mjrfRenderTarget* target = impl_->targets[camera_id];
    mjrf_resizeRenderTarget(target, w, h);

    // 构建 Filament 相机
    mjrCamera cam;
    params_to_mjr_camera(params, cam);

    // 构建渲染请求
    mjrfRenderRequest req;
    mjrf_defaultRenderRequest(&req);
    req.scene = impl_->scene;
    req.camera = cam;
    req.viewport = {0, 0, w, h};
    req.target = target;
    req.draw_mode = mjDRAW_MODE_DEFAULT;

    // 构建读取像素请求
    RenderOutput output;
    output.width = w;
    output.height = h;
    output.camera_id = camera_id;

    std::vector<uint8_t> rgb_buffer;
    std::vector<float> depth_buffer;

    mjrfReadPixelsRequest read_req;
    mjrf_defaultReadPixelsRequest(&read_req);
    read_req.target = target;

    if (mode == RenderMode::RGB || mode == RenderMode::All) {
        rgb_buffer.resize(w * h * 3);
        read_req.output = rgb_buffer.data();
        read_req.num_bytes = rgb_buffer.size();
    }
    // 注意: Filament 当前的 ReadPixelsRequest 设计是带回调的异步读取
    // 我们通过 mjrf_waitForFrame 来同步等待

    // 提交渲染
    mjrfFrameHandle frame = mjrf_render(
        impl_->rf_ctx, &req, 1,
        (mode == RenderMode::RGB || mode == RenderMode::All) ? &read_req : nullptr,
        (mode == RenderMode::RGB || mode == RenderMode::All) ? 1 : 0
    );

    // 等待完成
    mjrf_waitForFrame(impl_->rf_ctx, frame);

    // 提取数据
    if (!rgb_buffer.empty()) {
        output.rgb = std::move(rgb_buffer);
    }

    // 深度图：需要另外一次 render（Filament 不支持同时读取 RGB+Depth）
    if (mode == RenderMode::Depth || mode == RenderMode::All) {
        mjrfRenderRequest depth_req;
        mjrf_defaultRenderRequest(&depth_req);
        depth_req.scene = impl_->scene;
        depth_req.camera = cam;
        depth_req.viewport = {0, 0, w, h};
        depth_req.target = target;
        depth_req.draw_mode = mjDRAW_MODE_DEPTH;

        depth_buffer.resize(w * h);

        mjrfReadPixelsRequest depth_read;
        mjrf_defaultReadPixelsRequest(&depth_read);
        depth_read.target = target;
        depth_read.output = depth_buffer.data();
        depth_read.num_bytes = depth_buffer.size() * sizeof(float);

        mjrfFrameHandle depth_frame = mjrf_render(
            impl_->rf_ctx, &depth_req, 1, &depth_read, 1);
        mjrf_waitForFrame(impl_->rf_ctx, depth_frame);

        output.depth = std::move(depth_buffer);
    }

    return output;
}

int FilamentRenderer::camera_count() const {
    return static_cast<int>(impl_->cam_params.size());
}

void FilamentRenderer::set_camera_params(int cam_id, const CameraParams& params) {
    if (cam_id < 0 || cam_id >= static_cast<int>(impl_->cam_params.size())) return;
    impl_->cam_params[cam_id] = params;
}

void FilamentRenderer::reset_camera_params(int cam_id) {
    if (cam_id < 0 || cam_id >= static_cast<int>(impl_->cam_defaults.size())) return;
    impl_->cam_params[cam_id] = impl_->cam_defaults[cam_id];
}

void FilamentRenderer::set_clear_color(float r, float g, float b) {
    impl_->clear_color[0] = r;
    impl_->clear_color[1] = g;
    impl_->clear_color[2] = b;
    if (impl_->rf_ctx) {
        mjrf_setClearColor(impl_->rf_ctx, impl_->clear_color);
    }
}

void FilamentRenderer::load_cameras_from_model(mjModel_* m_raw) {
    auto* m = static_cast<mjModel*>(m_raw);
    int ncam = m->ncam;

    impl_->cam_params.resize(ncam);
    impl_->cam_defaults.resize(ncam);
    impl_->targets.resize(ncam, nullptr);

    for (int i = 0; i < ncam; i++) {
        CameraParams defaults = read_camera_from_model(m, i);
        impl_->cam_defaults[i] = defaults;
        impl_->cam_params[i] = defaults;

        // 创建 RenderTarget
        mjrfRenderTargetConfig target_config;
        mjrf_defaultRenderTargetConfig(&target_config);
        // 设置分辨率
        // (RenderTarget 的宽高从 camera params 读取)

        impl_->targets[i] = mjrf_createRenderTarget(impl_->rf_ctx, &target_config);
        if (impl_->targets[i]) {
            mjrf_resizeRenderTarget(impl_->targets[i], defaults.width, defaults.height);
        }
    }
}

void FilamentRenderer::build_filament_camera(int cam_id, mjData_* d) {
    // 相机参数已经存储在 cam_params 中
    // 渲染时直接使用
    (void)cam_id;
    (void)d;
}

}  // namespace mujoco_camrender
