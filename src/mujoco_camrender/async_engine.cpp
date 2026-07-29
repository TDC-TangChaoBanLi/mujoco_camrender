#include "mujoco_camrender/async_engine.hpp"
#include "mujoco_camrender/render_pool.hpp"

#include <mujoco/mujoco.h>

#include <chrono>
#include <cstdio>

namespace mujoco_camrender {

AsyncEngine::AsyncEngine() = default;

AsyncEngine::~AsyncEngine() {
    stop();
    if (render_data_) {
        mj_deleteData(static_cast<mjData*>(render_data_));
        render_data_ = nullptr;
    }
}

void AsyncEngine::start(mjModel_* m, mjData_* d,
                         float framerate,
                         RenderBackend backend,
                         int num_threads)
{
    if (running_) stop();

    model_ = m;
    framerate_ = framerate;
    backend_ = backend;

    // 复制一份 mjData 供渲染线程使用
    auto* src_data = static_cast<mjData*>(d);
    auto* src_model = static_cast<mjModel*>(m);

    if (render_data_) {
        mj_deleteData(static_cast<mjData*>(render_data_));
    }
    render_data_ = mj_copyData(nullptr, src_model, src_data);

    sim_data_ = d;

    // 获取相机数量
    num_cameras_ = src_model->ncam;
    frames_.resize(num_cameras_);

    // 创建渲染池
    pool_ = std::make_unique<RenderPool>(num_threads, backend);
    pool_->initialize(m, render_data_);

    // 启动渲染线程
    stop_requested_ = false;
    running_ = true;
    frame_count_ = 0;
    render_thread_ = std::thread(&AsyncEngine::render_loop, this);

    // 等待首帧就绪（最多等 3 秒）
    for (int wait = 0; wait < 3000 && frame_count_.load() == 0; wait++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::fprintf(stderr, "[AsyncEngine] Started: %d cameras, %.1f fps, %d threads\n",
                 num_cameras_, framerate, num_threads);
}

void AsyncEngine::stop() {
    if (!running_) return;

    stop_requested_ = true;
    {
        std::lock_guard<std::mutex> lock(data_mtx_);
        data_ready_ = true;  // 唤醒等待的渲染线程
    }

    if (render_thread_.joinable()) {
        render_thread_.join();
    }
    running_ = false;

    // 销毁渲染池（在渲染线程停止后安全）
    pool_.reset();

    std::fprintf(stderr, "[AsyncEngine] Stopped\n");
}

void AsyncEngine::step_simulation() {
    if (!running_) return;

    auto* m = static_cast<mjModel*>(model_);
    auto* d = static_cast<mjData*>(sim_data_);
    mj_step(m, d);

    // 复制状态到渲染数据
    {
        std::lock_guard<std::mutex> lock(data_mtx_);
        mj_copyData(static_cast<mjData*>(render_data_), m, d);
        data_ready_ = true;
    }
}

std::shared_ptr<RenderOutput> AsyncEngine::get_latest_frame(int camera_id) {
    if (camera_id < 0 || camera_id >= num_cameras_) return nullptr;
    std::lock_guard<std::mutex> lock(frames_mtx_);
    return frames_[camera_id];
}

void AsyncEngine::set_camera_params(int cam_id, const CameraParams& params) {
    if (pool_) pool_->set_camera_params(cam_id, params);
}

void AsyncEngine::reset_camera_params(int cam_id) {
    // 暂未实现（需要暴露到 RenderPool）
    (void)cam_id;
}

bool AsyncEngine::running() const {
    return running_;
}

long long AsyncEngine::frame_count() const {
    return frame_count_.load(std::memory_order_relaxed);
}

// =============================================================================
// 渲染线程主循环
// =============================================================================
void AsyncEngine::render_loop() {
    using clock = std::chrono::steady_clock;
    auto next_frame_time = clock::now();

    while (!stop_requested_) {

        // 更新场景
        auto* render_d = static_cast<mjData*>(render_data_);
        if (pool_ && render_d) {
            pool_->update_scene(render_d);

            // 渲染所有相机
            std::vector<int> all_cams(num_cameras_);
            for (int i = 0; i < num_cameras_; i++) all_cams[i] = i;

            auto results = pool_->render_all(all_cams, RenderMode::All);

            // 原子交换帧缓冲区
            {
                std::lock_guard<std::mutex> lock(frames_mtx_);
                for (int i = 0; i < num_cameras_ && i < static_cast<int>(results.size()); i++) {
                    frames_[i] = std::make_shared<RenderOutput>(std::move(results[i]));
                }
            }
        }

        frame_count_.fetch_add(1, std::memory_order_relaxed);

        if (framerate_ > 0) {
            auto frame_duration = std::chrono::microseconds(
                static_cast<long long>(1.0e6 / framerate_));
            next_frame_time += frame_duration;

            auto now = clock::now();
            if (next_frame_time > now) {
                std::this_thread::sleep_until(next_frame_time);
            } else {
                // 落伍了，重置计时
                next_frame_time = now;
            }
        }
    }

    std::fprintf(stderr, "[AsyncEngine] Render loop exited\n");
}

}  // namespace mujoco_camrender
