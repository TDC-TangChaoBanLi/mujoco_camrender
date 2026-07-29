# =============================================================================
# test_render — 集成测试
# =============================================================================

find_package(Threads REQUIRED)

add_executable(test_render test_render.cpp)

target_include_directories(test_render
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src
        ${MUJOCO_INCLUDE_DIR}
)

target_link_libraries(test_render
    PRIVATE
        mujoco_camrender
        ${MUJOCO_LIB}
        ${OPENGL_LIBRARY}
        Threads::Threads
)

# 从 build/ 目录运行，需要 assets/ 在 ../assets/
set_target_properties(test_render PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)
