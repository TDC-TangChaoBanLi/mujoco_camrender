#ifndef MUJOCO_CAMRENDER_FILAMENT_RENDERER_HPP_
#define MUJOCO_CAMRENDER_FILAMENT_RENDERER_HPP_

#include <memory>
#include <vector>

#include "mujoco_camrender/renderer_interface.hpp"

// MuJoCo 前向声明
struct mjModel_;
struct mjData_;

namespace mujoco_camrender {

/// 基于 MuJoCo Filament 渲染器 (mjrf_* API) 的离屏渲染实现。
///
/// Filament 使用 PBR（物理渲染）管线，支持阴影、反射、后处理等高级特性。
///
/// 使用方式与 GLRenderer 相同。
class FilamentRenderer : public RendererInterface {
public:
    FilamentRenderer();
    ~FilamentRenderer() override;

    // 禁止拷贝
    FilamentRenderer(const FilamentRenderer&) = delete;
    FilamentRenderer& operator=(const FilamentRenderer&) = delete;

    // --- RendererInterface ---
    void initialize(mjModel_* m, mjData_* d) override;
    void update_scene(mjData_* d) override;
    RenderOutput render(int camera_id, RenderMode mode) override;
    int camera_count() const override;
    void set_camera_params(int cam_id, const CameraParams& params) override;
    void reset_camera_params(int cam_id) override;

    // --- FilamentRenderer 特有 ---
    /// 设置背景清除色（RGB，每分量 0-1）
    void set_clear_color(float r, float g, float b);

private:
    void load_cameras_from_model(mjModel_* m);
    void build_filament_camera(int cam_id, mjData_* d);

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_camrender

#endif  // MUJOCO_CAMRENDER_FILAMENT_RENDERER_HPP_
