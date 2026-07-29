#ifndef MUJOCO_CAMRENDER_RENDER_POOL_HPP_
#define MUJOCO_CAMRENDER_RENDER_POOL_HPP_

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "mujoco_camrender/renderer_interface.hpp"
#include "mujoco_camrender/types.hpp"

namespace mujoco_camrender {

/// 多线程渲染池：将 N 个相机分配到 M 个渲染线程并行渲染。
///
/// 每个线程拥有独立的 RendererInterface 实例（对于 OpenGL 后端，
/// 这意味着独立的 GL 上下文），确保线程安全。
class RenderPool {
public:
    /// @param num_threads 工作线程数
    /// @param backend     渲染后端
    RenderPool(int num_threads, RenderBackend backend);
    ~RenderPool();

    // 禁止拷贝
    RenderPool(const RenderPool&) = delete;
    RenderPool& operator=(const RenderPool&) = delete;

    /// 初始化所有工作线程
    void initialize(mjModel_* m, mjData_* d);

    /// 同步并行渲染：camera_ids 被分配到各线程并行执行
    /// @return 结果顺序与 camera_ids 一致
    std::vector<RenderOutput> render_all(
        const std::vector<int>& camera_ids,
        RenderMode mode);

    /// 更新所有线程的场景（仅 OpenGL 后端需要）
    void update_scene(mjData_* d);

    /// 覆盖相机参数
    void set_camera_params(int cam_id, const CameraParams& params);

    /// 获取线程数
    int thread_count() const;

private:
    // 每个工作线程
    struct Worker {
        std::unique_ptr<RendererInterface> renderer;
        std::thread thread;

        // 同步
        std::mutex mtx;
        std::condition_variable cv;
        bool ready = false;        // 渲染器已初始化
        bool has_work = false;     // 有待处理任务
        bool shutdown = false;

        // 任务数据
        std::vector<int> camera_ids;
        RenderMode mode = RenderMode::RGB;
        std::vector<RenderOutput> results;
        mjData_* data = nullptr;

        // 主函数
        void run();
    };

    std::vector<std::unique_ptr<Worker>> workers_;
    RenderBackend backend_;
    mjModel_* model_ = nullptr;
};

}  // namespace mujoco_camrender

#endif  // MUJOCO_CAMRENDER_RENDER_POOL_HPP_
