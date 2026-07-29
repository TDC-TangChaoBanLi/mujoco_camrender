#ifndef MUJOCO_CAMRENDER_GL_RENDERER_HPP_
#define MUJOCO_CAMRENDER_GL_RENDERER_HPP_

#include <memory>
#include <vector>

#include "mujoco_camrender/renderer_interface.hpp"
#include "mujoco_camrender/gl_context.hpp"

// MuJoCo forward declarations (C structs)
struct mjModel_;
struct mjData_;
struct mjrContext_;
struct mjvScene_;
struct mjvCamera_;
struct mjvOption_;

namespace mujoco_camrender {

/// 基于 MuJoCo 原生 OpenGL 渲染器 (mjr_* API) 的离屏渲染实现。
///
/// 使用方式：
///   1. GL 上下文必须在调用线程上激活（构造时自动创建）
///   2. 调用 initialize(model, data) 加载场景
///   3. 每个物理步进后调用 update_scene(data)
///   4. 调用 render(camera_id, mode) 进行渲染
///
/// 每个 GLRenderer 实例拥有独立的 GL 上下文和 MuJoCo 渲染上下文，
/// 因此可以在不同线程中独立工作。
class GLRenderer : public RendererInterface {
public:
    GLRenderer();
    ~GLRenderer() override;

    // 禁止拷贝
    GLRenderer(const GLRenderer&) = delete;
    GLRenderer& operator=(const GLRenderer&) = delete;

    // --- RendererInterface ---
    void initialize(mjModel_* m, mjData_* d) override;
    void update_scene(mjData_* d) override;
    RenderOutput render(int camera_id, RenderMode mode) override;
    int camera_count() const override;
    void set_camera_params(int cam_id, const CameraParams& params) override;
    void reset_camera_params(int cam_id) override;

    // --- GLRenderer 特有 ---
    /// 设置离屏渲染目标分辨率（影响所有相机）
    void set_resolution(int width, int height);

private:
    // 从 MJCF 相机参数构建 CameraParams
    void load_cameras_from_model(mjModel_* m);
    // 将 CameraParams 应用到 mjvCamera
    void apply_camera_params(int cam_id, mjData_* d);

    GLContext gl_ctx_;

    // MuJoCo 渲染上下文（不透明指针，避免在头文件中引入 mujoco.h）
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_camrender

#endif  // MUJOCO_CAMRENDER_GL_RENDERER_HPP_
