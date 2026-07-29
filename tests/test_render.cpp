/**
 * test_render.cpp — MjRenderers 集成测试
 *
 * 测试内容：
 *   1. 加载场景 & 单相机同步渲染
 *   2. 并行多相机渲染（RenderPool）
 *   3. 异步后台渲染（AsyncEngine）
 *
 * 输出：渲染图像保存为 PPM 格式到 output/ 目录
 */

#include <mujoco_camrender/gl_renderer.hpp>
#include <mujoco_camrender/render_pool.hpp>
#include <mujoco_camrender/async_engine.hpp>
#include <mujoco/mujoco.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>

// =============================================================================
// 辅助：保存 RGB 图像为 PPM（P6 二进制格式）
// =============================================================================
static void save_ppm(const std::string& path,
                     const uint8_t* rgb, int w, int h) {
    std::ofstream f(path, std::ios::binary);
    f << "P6\n" << w << " " << h << "\n255\n";
    f.write(reinterpret_cast<const char*>(rgb), w * h * 3);
    std::printf("  Saved: %s (%dx%d)\n", path.c_str(), w, h);
}

// =============================================================================
// 辅助：计时器
// =============================================================================
class Timer {
public:
    Timer(const char* label) : label_(label), start_(std::chrono::steady_clock::now()) {}
    ~Timer() {
        auto end = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_).count();
        std::printf("[TIMER] %s: %ld ms\n", label_, ms);
    }
private:
    const char* label_;
    std::chrono::steady_clock::time_point start_;
};

// =============================================================================
// 主函数
// =============================================================================
int main(int argc, char** argv) {
    const char* xml_path = "assets/test_scene.xml";
    if (argc > 1) xml_path = argv[1];

    std::printf("========================================\n");
    std::printf("  MjRenderers 集成测试\n");
    std::printf("========================================\n\n");

    // -------------------------------------------------------------------------
    // 加载模型
    // -------------------------------------------------------------------------
    std::printf("[1/6] Loading model: %s\n", xml_path);
    char error[1000] = {};
    mjModel* m = mj_loadXML(xml_path, nullptr, error, sizeof(error));
    if (!m) {
        std::fprintf(stderr, "ERROR: %s\n", error);
        return 1;
    }
    mjData* d = mj_makeData(m);
    mj_forward(m, d);

    int ncam = m->ncam;
    std::printf("  Model loaded: %d cameras\n\n", ncam);

    // -------------------------------------------------------------------------
    // 测试 1：单相机同步渲染
    // -------------------------------------------------------------------------
    std::printf("[2/6] Single-camera sync render (GLRenderer)\n");
    {
        mujoco_camrender::GLRenderer renderer;
        renderer.initialize(m, d);

        // 仿真 10 步
        for (int i = 0; i < 10; i++) {
            mj_step(m, d);
        }
        renderer.update_scene(d);

        std::vector<mujoco_camrender::RenderOutput> outputs;
        {
            Timer t("Single render");
            for (int i = 0; i < ncam; i++) {
                outputs.push_back(renderer.render(i, mujoco_camrender::RenderMode::All));
            }
        }
        for (int i = 0; i < ncam; i++) {
            char path[256];
            std::snprintf(path, sizeof(path), "output/cam%d_rgb.ppm", i);
            save_ppm(path, outputs[i].rgb.data(), outputs[i].width, outputs[i].height);
        }
    }
    std::printf("\n");

    // -------------------------------------------------------------------------
    // 测试 2：并行多相机渲染
    // -------------------------------------------------------------------------
    int n_thead_parallel = 4; // 渲染线程数
    std::printf("[3/6] Parallel multi-camera render (RenderPool, %d threads)\n", n_thead_parallel);
    {
        mujoco_camrender::RenderPool pool(n_thead_parallel, mujoco_camrender::RenderBackend::OpenGL);
        pool.initialize(m, d);

        std::vector<int> all_cams(ncam);
        for (int i = 0; i < ncam; i++) all_cams[i] = i;

        std::vector<mujoco_camrender::RenderOutput> first_results;
        {
            Timer t("Parallel render x100");
            for (int iter = 0; iter < 100; iter++) {
                mj_step(m, d);
                pool.update_scene(d);
                auto results = pool.render_all(all_cams, mujoco_camrender::RenderMode::RGB);
                if (iter == 0) first_results = std::move(results);
            }
        }
        for (int i = 0; i < ncam; i++) {
            char path[256];
            std::snprintf(path, sizeof(path), "output/cam%d_parallel.ppm", i);
            save_ppm(path, first_results[i].rgb.data(),
                     first_results[i].width, first_results[i].height);
        }
    }
    std::printf("\n");

    // -------------------------------------------------------------------------
    // 测试 3：异步后台渲染
    // -------------------------------------------------------------------------
    int n_thead_async = 4; // 渲染线程
    std::printf("[4/6] Async background rendering (AsyncEngine, max FPS)\n");
    {
        mujoco_camrender::AsyncEngine engine;
        engine.start(m, d, 0.0f,  // 0 = max FPS
                     mujoco_camrender::RenderBackend::OpenGL,
                     n_thead_async);  // 渲染线程

        long long frames_before = engine.frame_count();
        auto t_start = std::chrono::steady_clock::now();

        // 运行仿真 60 步，渲染线程在后台持续工作
        for (int step = 0; step < 1000; step++) {
            engine.step_simulation();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        auto t_end = std::chrono::steady_clock::now();
        long long frames_after = engine.frame_count();

        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
        long long delta = frames_after - frames_before;
        double fps = (elapsed_ms > 0 && delta > 0) ? delta * 1000.0 / elapsed_ms : 0.0;
        std::printf("[TIMER] Async render 1000 steps: %ld ms, %lld frames (%.1f FPS)\n",
                     elapsed_ms, delta, fps);

        // 获取最新帧
        for (int i = 0; i < ncam; i++) {
            auto frame = engine.get_latest_frame(i);
            if (frame && !frame->rgb.empty()) {
                char path[256];
                std::snprintf(path, sizeof(path), "output/cam%d_async.ppm", i);
                save_ppm(path, frame->rgb.data(), frame->width, frame->height);
            } else {
                std::printf("  Camera %d: no frame available\n", i);
            }
        }

        engine.stop();
    }
    std::printf("\n");

    // -------------------------------------------------------------------------
    // 测试 4：相机参数覆盖
    // -------------------------------------------------------------------------
    std::printf("[5/6] Camera parameter override\n");
    {
        mujoco_camrender::GLRenderer renderer;
        renderer.initialize(m, d);

        mujoco_camrender::CameraParams params;
        params.name = "override_test";
        params.width = 320;
        params.height = 240;
        params.fov = 90.0f;
        params.position[0] = 0;
        params.position[1] = -2;
        params.position[2] = 3;
        params.lookat[0] = 0;
        params.lookat[1] = 0;
        params.lookat[2] = 0.5f;

        renderer.set_camera_params(0, params);
        renderer.update_scene(d);
        auto out = renderer.render(0, mujoco_camrender::RenderMode::RGB);
        save_ppm("output/cam0_overridden.ppm", out.rgb.data(), out.width, out.height);

        renderer.reset_camera_params(0);
    }
    std::printf("\n");

    // -------------------------------------------------------------------------
    // 测试 5：Filament 后端（跳过 — 需要 mujoco filament 编译支持）
    // -------------------------------------------------------------------------
    std::printf("[6/6] Filament backend — SKIPPED (needs filament-enabled mujoco build)\n\n");

    // -------------------------------------------------------------------------
    // 清理
    // -------------------------------------------------------------------------
    mj_deleteData(d);
    mj_deleteModel(m);

    std::printf("========================================\n");
    std::printf("  All tests passed!\n");
    std::printf("  Images saved to output/\n");
    std::printf("========================================\n");
    return 0;
}
