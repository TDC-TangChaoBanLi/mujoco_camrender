# =============================================================================
# Python 绑定（pybind11）
# =============================================================================

# 查找 Python 和 pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

pybind11_add_module(_mujoco_camrender
    mujoco_camrender_pybind.cpp
)

target_include_directories(_mujoco_camrender
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src
        ${MUJOCO_INCLUDE_DIR}
)

target_link_libraries(_mujoco_camrender
    PRIVATE
        mujoco_camrender
        ${MUJOCO_LIB}
        ${OPENGL_LIBRARY}
)

# 安装到 Python 包目录
install(TARGETS _mujoco_camrender
    LIBRARY DESTINATION ${CMAKE_SOURCE_DIR}/src/mujoco_camrender
)
