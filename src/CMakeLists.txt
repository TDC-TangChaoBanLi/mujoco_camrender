# =============================================================================
# libmujoco_camrender — shared library
# =============================================================================

set(MJRENDERERS_SOURCES
    mujoco_camrender/gl_context.cpp
    mujoco_camrender/gl_renderer.cpp
    mujoco_camrender/render_pool.cpp
    mujoco_camrender/async_engine.cpp
)

set(MJRENDERERS_HEADERS
    ${PROJECT_SOURCE_DIR}/include/mujoco_camrender/types.hpp
    ${PROJECT_SOURCE_DIR}/include/mujoco_camrender/renderer_interface.hpp
    ${PROJECT_SOURCE_DIR}/include/mujoco_camrender/gl_context.hpp
    ${PROJECT_SOURCE_DIR}/include/mujoco_camrender/gl_renderer.hpp
    ${PROJECT_SOURCE_DIR}/include/mujoco_camrender/filament_renderer.hpp
    ${PROJECT_SOURCE_DIR}/include/mujoco_camrender/render_pool.hpp
    ${PROJECT_SOURCE_DIR}/include/mujoco_camrender/async_engine.hpp
)

add_library(mujoco_camrender SHARED ${MJRENDERERS_SOURCES} ${MJRENDERERS_HEADERS})

target_include_directories(mujoco_camrender
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${MUJOCO_INCLUDE_DIR}
)

target_link_libraries(mujoco_camrender
    PRIVATE
        ${MUJOCO_LIB}
        glfw
        ${OPENGL_LIBRARY}
)

# 在 Linux 上链接 pthread（MuJoCo 需要）
find_package(Threads REQUIRED)
target_link_libraries(mujoco_camrender PRIVATE Threads::Threads)

target_compile_options(mujoco_camrender PRIVATE
    -Wall -Wextra -Wno-unused-parameter
)
