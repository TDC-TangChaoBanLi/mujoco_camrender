#ifndef MUJOCO_CAMRENDER_TYPES_HPP_
#define MUJOCO_CAMRENDER_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace mujoco_camrender {

// =============================================================================
// Enums
// =============================================================================

/// 渲染模式
enum class RenderMode {
    RGB = 0,   // 只输出 RGB 图像
    Depth,     // 只输出深度图
    All        // 同时输出 RGB 和深度
};

/// 渲染后端
enum class RenderBackend {
    OpenGL = 0,
    Filament
};

// =============================================================================
// CameraParams
// =============================================================================

/// 相机渲染参数，可从 XML 相机读取或手动覆盖
struct CameraParams {
    std::string name;           // 相机名称
    int width = 640;            // 图像宽度
    int height = 480;           // 图像高度
    float fov = 45.0f;          // 垂直 FOV（度）
    float position[3] = {};     // 相机位置 (x, y, z)
    float lookat[3] = {};       // 注视点 (x, y, z)，仅自由相机使用
    bool orthographic = false;  // 是否正交投影
    float near_clip = 0.001f;   // 近裁剪面
    float far_clip = 100.0f;    // 远裁剪面
    int cam_type = 0;           // mjCAMERA_FREE / _FIXED / _TRACKING
    int fixedcamid = -1;        // 固定相机在模型中的索引
    int trackbodyid = -1;       // 跟踪目标 body ID
};

// =============================================================================
// RenderOutput
// =============================================================================

/// 单次渲染输出
struct RenderOutput {
    int width = 0;                    // 图像宽度
    int height = 0;                   // 图像高度
    std::vector<uint8_t> rgb;         // RGB 8-bit 数据，排列: [H*W*3]
    std::vector<float> depth;         // 深度 float32 数据，排列: [H*W]
    int camera_id = -1;               // 来源相机 ID
};

}  // namespace mujoco_camrender

#endif  // MUJOCO_CAMRENDER_TYPES_HPP_
