cmake_minimum_required(VERSION 3.16)
project(mujoco_camrender VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# =============================================================================
# MuJoCo (REQUIRED — 用户必须指定安装路径)
# =============================================================================
set(MUJOCO_DIR "" CACHE PATH "Path to MuJoCo installation (required)")
if(NOT MUJOCO_DIR)
    message(FATAL_ERROR "MUJOCO_DIR is required. Pass -DMUJOCO_DIR=/path/to/mujoco")
endif()

set(MUJOCO_INCLUDE_DIR "${MUJOCO_DIR}/include")
set(MUJOCO_LIB_DIR "${MUJOCO_DIR}/lib")

find_library(MUJOCO_LIB mujoco
    PATHS "${MUJOCO_LIB_DIR}"
    REQUIRED)

message(STATUS "MuJoCo include: ${MUJOCO_INCLUDE_DIR}")
message(STATUS "MuJoCo lib:     ${MUJOCO_LIB}")

# =============================================================================
# GLFW + OpenGL (for headless OpenGL rendering via hidden window)
# =============================================================================
find_package(glfw3 REQUIRED)

find_library(OPENGL_LIBRARY NAMES OpenGL
    REQUIRED)

message(STATUS "GLFW libs:    ${glfw3_LIBRARIES}")
message(STATUS "OpenGL lib:   ${OPENGL_LIBRARY}")

# =============================================================================
# Subdirectories
# =============================================================================
add_subdirectory(src)
add_subdirectory(tests)

# Python 绑定（可选，需要 pybind11 和 Python 开发包）
set(PYBIND11_FINDPYTHON ON)  # 使用 FindPython 而非已弃用的 FindPythonLibs
find_package(pybind11 QUIET)
if(pybind11_FOUND)
    message(STATUS "pybind11 found, building Python bindings")
    add_subdirectory(src/pybind)
else()
    message(STATUS "pybind11 not found, skipping Python bindings")
endif()
