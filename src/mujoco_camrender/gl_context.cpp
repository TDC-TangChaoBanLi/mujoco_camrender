#include "mujoco_camrender/gl_context.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include <cstdio>
#include <mutex>

namespace mujoco_camrender {

// ---------------------------------------------------------------------------
// GLFW 全局初始化（引用计数，保证多实例安全）
// ---------------------------------------------------------------------------
static int  g_glfw_refcount = 0;
static std::mutex g_glfw_mutex;

static bool glfw_global_init() {
    std::lock_guard<std::mutex> lock(g_glfw_mutex);
    if (g_glfw_refcount == 0) {
        if (!glfwInit()) {
            std::fprintf(stderr, "[GL] glfwInit failed\n");
            return false;
        }
    }
    g_glfw_refcount++;
    return true;
}

static void glfw_global_shutdown() {
    std::lock_guard<std::mutex> lock(g_glfw_mutex);
    g_glfw_refcount--;
    if (g_glfw_refcount == 0) {
        glfwTerminate();
    }
}

// ---------------------------------------------------------------------------
// GLContext
// ---------------------------------------------------------------------------

GLContext::GLContext() : window_(nullptr), valid_(false) {}

GLContext::~GLContext() { destroy(); }

GLContext::GLContext(GLContext&& o) noexcept
    : window_(o.window_), valid_(o.valid_)
{
    o.window_ = nullptr;
    o.valid_  = false;
}

GLContext& GLContext::operator=(GLContext&& o) noexcept {
    if (this != &o) {
        destroy();
        window_ = o.window_;
        valid_  = o.valid_;
        o.window_ = nullptr;
        o.valid_  = false;
    }
    return *this;
}

bool GLContext::create(int w, int h) {
    if (valid_) destroy();

    // 全局初始化 GLFW（显式指定 X11 平台，避免 Wayland 下的 GTK 警告）
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
    if (!glfw_global_init()) return false;

    // 隐藏窗口 + 桌面 OpenGL
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
    glfwWindowHint(GLFW_DEPTH_BITS, 24);
    glfwWindowHint(GLFW_STENCIL_BITS, 8);
    glfwWindowHint(GLFW_SAMPLES, 0);          // 关闭 MSAA
    // 不指定 CONTEXT_VERSION / PROFILE，使用兼容模式 → 桌面 GL

    GLFWwindow* win = glfwCreateWindow(w, h, "mjrenderers_offscreen",
                                       nullptr, nullptr);
    if (!win) {
        std::fprintf(stderr, "[GL] glfwCreateWindow failed\n");
        glfw_global_shutdown();
        return false;
    }

    glfwMakeContextCurrent(win);

    // 打印 GL 信息
    std::fprintf(stderr, "[GL] %s / %s / %s\n",
                 glGetString(GL_VENDOR),
                 glGetString(GL_RENDERER),
                 glGetString(GL_VERSION));

    // 验证是桌面 GL（非 ES）
    const char* ver = reinterpret_cast<const char*>(glGetString(GL_VERSION));
    if (ver && (ver[0] == 'O' && ver[1] == 'p' && ver[2] == 'e' && ver[3] == 'n'
                && ver[4] == 'G' && ver[5] == 'L' && ver[6] == ' ' && ver[7] == 'E')) {
        std::fprintf(stderr, "[GL] WARNING: got OpenGL ES, MuJoCo requires desktop GL\n");
    }

    // 释放上下文，由调用线程通过 make_current() 按需获取
    glfwMakeContextCurrent(nullptr);

    window_ = win;
    valid_  = true;
    return true;
}

void GLContext::destroy() {
    if (!valid_) return;

    auto* win = static_cast<GLFWwindow*>(window_);
    // glfwDestroyWindow 内部会把 context 设为 non-current，无需手动释放
    glfwDestroyWindow(win);
    glfw_global_shutdown();

    window_ = nullptr;
    valid_  = false;
}

void GLContext::make_current() {
    if (!valid_) return;
    glfwMakeContextCurrent(static_cast<GLFWwindow*>(window_));
}

void GLContext::release_current() {
    if (!valid_) return;
    glfwMakeContextCurrent(nullptr);
}

bool GLContext::valid() const { return valid_; }

}  // namespace mujoco_camrender
