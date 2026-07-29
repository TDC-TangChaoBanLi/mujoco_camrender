#ifndef MUJOCO_CAMRENDER_GL_CONTEXT_HPP_
#define MUJOCO_CAMRENDER_GL_CONTEXT_HPP_

namespace mujoco_camrender {

/// 跨平台离屏 OpenGL 上下文。
///
/// 通过 GLFW 隐藏窗口创建桌面 OpenGL 上下文，
/// 兼容 MuJoCo mjr_* API（需要 ARB_framebuffer_object）。
///
/// 每个 GLContext 绑定到一个线程。不可跨线程共享。
class GLContext {
public:
    GLContext();
    ~GLContext();

    // 禁止拷贝
    GLContext(const GLContext&) = delete;
    GLContext& operator=(const GLContext&) = delete;

    // 允许移动
    GLContext(GLContext&& other) noexcept;
    GLContext& operator=(GLContext&& other) noexcept;

    /// 创建 GLFW 隐藏窗口和 OpenGL 上下文
    /// @param width  窗口帧缓冲宽度（建议 >= 最大相机分辨率）
    /// @param height 窗口帧缓冲高度
    /// @return 成功返回 true
    bool create(int width = 640, int height = 480);

    /// 销毁上下文及窗口
    void destroy();

    /// 将当前上下文绑定到调用线程
    void make_current();

    /// 从当前线程释放上下文（允许其他线程获取）
    void release_current();

    /// 检查上下文是否有效
    bool valid() const;

private:
    void* window_;   // GLFWwindow*
    bool valid_;
};

}  // namespace mujoco_camrender

#endif  // MUJOCO_CAMRENDER_GL_CONTEXT_HPP_
