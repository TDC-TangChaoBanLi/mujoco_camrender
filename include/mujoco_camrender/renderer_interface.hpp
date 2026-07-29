#ifndef MUJOCO_CAMRENDER_RENDERER_INTERFACE_HPP_
#define MUJOCO_CAMRENDER_RENDERER_INTERFACE_HPP_

#include <memory>

#include "mujoco_camrender/types.hpp"

// 前向声明 MuJoCo 类型
struct mjModel_;
struct mjData_;

namespace mujoco_camrender {

/// 抽象渲染器接口 —— 同步、单相机渲染
class RendererInterface {
public:
    virtual ~RendererInterface() = default;

    // -------------------------------------------------------------------------
    // 生命周期
    // -------------------------------------------------------------------------

    /// 初始化渲染器，加载模型资源
    /// @param m MuJoCo 模型指针（mjModel*）
    /// @param d MuJoCo 数据指针（mjData*）
    virtual void initialize(mjModel_* m, mjData_* d) = 0;

    // -------------------------------------------------------------------------
    // 场景更新
    // -------------------------------------------------------------------------

    /// 更新抽象场景（物理步进后必须调用，然后才能 render）
    virtual void update_scene(mjData_* d) = 0;

    // -------------------------------------------------------------------------
    // 渲染
    // -------------------------------------------------------------------------

    /// 同步渲染指定相机
    /// @param camera_id 相机在模型中的索引（0-based）
    /// @param mode      渲染模式
    /// @return RenderOutput 包含 RGB / 深度数据
    virtual RenderOutput render(int camera_id, RenderMode mode) = 0;

    // -------------------------------------------------------------------------
    // 相机管理
    // -------------------------------------------------------------------------

    /// 获取模型中的相机数量
    virtual int camera_count() const = 0;

    /// 覆盖指定相机的渲染参数（不影响模型本身）
    /// @param cam_id 相机索引
    /// @param params 新的相机参数
    virtual void set_camera_params(int cam_id, const CameraParams& params) = 0;

    /// 恢复指定相机为模型默认参数
    virtual void reset_camera_params(int cam_id) = 0;
};

}  // namespace mujoco_camrender

#endif  // MUJOCO_CAMRENDER_RENDERER_INTERFACE_HPP_
