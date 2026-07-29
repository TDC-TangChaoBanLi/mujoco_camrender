#include "mujoco_camrender/render_pool.hpp"
#include "mujoco_camrender/gl_renderer.hpp"

#include <cstdio>
#include <stdexcept>

namespace mujoco_camrender {

// =============================================================================
// Worker 线程主循环
// =============================================================================
void RenderPool::Worker::run() {
    // 等待初始化完成
    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return ready || shutdown; });
    }
    if (shutdown) return;

    // 工作循环
    while (true) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return has_work || shutdown; });

        if (shutdown) break;
        if (!has_work) continue;

        // 复制任务数据（在锁内）
        std::vector<int> cams = camera_ids;
        RenderMode m = mode;
        lock.unlock();

        // 执行渲染
        results.clear();
        for (int cam_id : cams) {
            results.push_back(renderer->render(cam_id, m));
        }

        // 标记完成
        lock.lock();
        has_work = false;
        lock.unlock();
        cv.notify_one();
    }
}

// =============================================================================
// RenderPool
// =============================================================================

RenderPool::RenderPool(int num_threads, RenderBackend backend)
    : backend_(backend)
{
    workers_.reserve(num_threads);

    for (int i = 0; i < num_threads; i++) {
        auto w = std::make_unique<Worker>();

        switch (backend) {
        case RenderBackend::OpenGL:
            w->renderer = std::make_unique<GLRenderer>();
            break;
        case RenderBackend::Filament:
            // Filament support requires mujoco built with filament
            throw std::runtime_error("Filament backend not available in this build");
        }

        w->thread = std::thread(&Worker::run, w.get());
        workers_.push_back(std::move(w));
    }
}

RenderPool::~RenderPool() {
    for (auto& wp : workers_) {
        {
            std::lock_guard<std::mutex> lock(wp->mtx);
            wp->shutdown = true;
            wp->ready = true;
        }
        wp->cv.notify_one();
        if (wp->thread.joinable()) {
            wp->thread.join();
        }
    }
}

void RenderPool::initialize(mjModel_* m, mjData_* d) {
    model_ = m;
    for (auto& wp : workers_) {
        wp->renderer->initialize(m, d);
        {
            std::lock_guard<std::mutex> lock(wp->mtx);
            wp->ready = true;
        }
        wp->cv.notify_one();
    }
}

std::vector<RenderOutput> RenderPool::render_all(
        const std::vector<int>& camera_ids,
        RenderMode mode)
{
    if (camera_ids.empty()) return {};

    int n = static_cast<int>(workers_.size());
    if (n == 0) return {};

    int total = static_cast<int>(camera_ids.size());
    int per_thread = (total + n - 1) / n;

    for (int i = 0; i < n; i++) {
        auto& wp = workers_[i];
        int start = i * per_thread;
        int end = std::min(start + per_thread, total);

        std::lock_guard<std::mutex> lock(wp->mtx);
        wp->camera_ids.clear();
        wp->mode = mode;
        if (start < total) {
            wp->camera_ids.assign(camera_ids.begin() + start,
                                  camera_ids.begin() + end);
        }
        wp->has_work = !wp->camera_ids.empty();
    }

    for (auto& wp : workers_) wp->cv.notify_one();

    for (auto& wp : workers_) {
        std::unique_lock<std::mutex> lock(wp->mtx);
        wp->cv.wait(lock, [&wp] { return !wp->has_work || wp->shutdown; });
    }

    std::vector<RenderOutput> all_results;
    all_results.reserve(total);
    for (auto& wp : workers_) {
        std::lock_guard<std::mutex> lock(wp->mtx);
        for (auto& r : wp->results) {
            all_results.push_back(std::move(r));
        }
    }
    return all_results;
}

void RenderPool::update_scene(mjData_* d) {
    for (auto& wp : workers_) {
        wp->renderer->update_scene(d);
    }
}

void RenderPool::set_camera_params(int cam_id, const CameraParams& params) {
    for (auto& wp : workers_) {
        wp->renderer->set_camera_params(cam_id, params);
    }
}

int RenderPool::thread_count() const {
    return static_cast<int>(workers_.size());
}

}  // namespace mujoco_camrender
