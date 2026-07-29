#ifndef MUJOCO_CAMRENDER_ASYNC_ENGINE_HPP_
#define MUJOCO_CAMRENDER_ASYNC_ENGINE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "mujoco_camrender/types.hpp"

struct mjModel_;
struct mjData_;

namespace mujoco_camrender {

/// 异步渲染引擎：在后台线程中持续渲染，主线程可以非阻塞地获取最新帧。
///
/// 典型用法：
///   engine.start(model, data, 30.0f, RenderBackend::OpenGL);
///   while (...) {
///       engine.step_simulation();        // 推进物理
///       auto frame = engine.get_latest_frame(0);  // 非阻塞获取
///       process(frame);
///   }
///   engine.stop();
class AsyncEngine {
public:
    AsyncEngine();
    ~AsyncEngine();

    // 禁止拷贝
    AsyncEngine(const AsyncEngine&) = delete;
    AsyncEngine& operator=(const AsyncEngine&) = delete;

    /// 启动后台渲染循环
    /// @param m            MuJoCo 模型
    /// @param d            MuJoCo 数据（会被内部复制）
    /// @param framerate    目标帧率，<= 0 表示最大帧率
    /// @param backend      渲染后端
    /// @param num_threads  渲染线程数
    void start(mjModel_* m, mjData_* d,
               float framerate,
               RenderBackend backend,
               int num_threads);

    /// 停止后台循环
    void stop();

    /// 推进物理仿真一步（在主线程中调用）
    void step_simulation();

    /// 非阻塞获取指定相机的最新帧
    /// @return 可能为 nullptr（尚未渲染出第一帧）
    std::shared_ptr<RenderOutput> get_latest_frame(int camera_id);

    /// 覆盖相机参数（线程安全）
    void set_camera_params(int cam_id, const CameraParams& params);

    /// 恢复相机为模型默认参数
    void reset_camera_params(int cam_id);

    /// 是否正在运行
    bool running() const;

    /// 渲染线程已完成的帧数（渲染循环迭代次数）
    long long frame_count() const;

private:
    void render_loop();

    mjModel_* model_ = nullptr;
    mjData_* sim_data_ = nullptr;       // 仿真数据（主线程写入）
    mjData_* render_data_ = nullptr;     // 渲染数据副本（后台线程读取）

    float framerate_ = 0;
    RenderBackend backend_ = RenderBackend::OpenGL;

    // 双缓冲帧存储：每个相机一个 shared_ptr，用 mutex 保护
    std::vector<std::shared_ptr<RenderOutput>> frames_;
    mutable std::mutex frames_mtx_;
    int num_cameras_ = 0;

    // 同步
    std::mutex data_mtx_;
    bool data_ready_ = false;

    // 渲染线程
    std::thread render_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<long long> frame_count_{0};

    // RenderPool（仅在渲染线程中使用）
    std::unique_ptr<class RenderPool> pool_;
};

}  // namespace mujoco_camrender

#endif  // MUJOCO_CAMRENDER_ASYNC_ENGINE_HPP_
