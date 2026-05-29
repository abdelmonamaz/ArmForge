cmake_minimum_required(VERSION 3.21)

project(ArmForge VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)   # required by qt_add_qml_module (metatype generation)

# ── CUDA (Phase 5 — optional GPU-accelerated motion planning) ─
# Transparent CPU fallback: if no CUDA Toolkit is found, ARMFORGE_HAS_CUDA
# stays unset, CudaRrtPlanner is excluded from the build, and the UI hides
# the GPU benchmark — RrtConnectPlanner keeps running on CPU as before.
#
# NOTE: we deliberately do NOT enable_language(CUDA). CMake's native CUDA
# language rules mis-quote the nvcc command line under the "NMake Makefiles
# JOM" generator (this kit's generator) whenever the host compiler path
# contains spaces/parentheses (e.g. "Program Files (x86)"), producing
# `nvcc fatal: A single input file is required for a non-link phase...`.
# Instead, core/CMakeLists.txt invokes nvcc directly via add_custom_command
# with VERBATIM, which CMake quotes correctly on every generator.
option(ARMFORGE_ENABLE_CUDA "Build the CUDA-accelerated motion planner if the CUDA Toolkit is available" ON)
set(ARMFORGE_HAS_CUDA FALSE)
if(ARMFORGE_ENABLE_CUDA)
    find_package(CUDAToolkit QUIET)
    if(CUDAToolkit_FOUND)
        set(ARMFORGE_HAS_CUDA TRUE)
        message(STATUS "CUDA Toolkit ${CUDAToolkit_VERSION} found (${CUDAToolkit_NVCC_EXECUTABLE}) -- building CudaRrtPlanner")
    else()
        message(STATUS "ARMFORGE_ENABLE_CUDA is ON but the CUDA Toolkit was not found -- building CPU-only")
    endif()
endif()

# ── Eigen 3 ───────────────────────────────────────────────────
# Priority 1: vcpkg / system install (CMAKE_TOOLCHAIN_FILE configured in Qt Creator)
# Priority 2: FetchContent fallback when vcpkg toolchain is not active
find_package(Eigen3 3.4 QUIET CONFIG)
if(NOT Eigen3_FOUND)
    message(STATUS "Eigen3 not found locally -- fetching via FetchContent")
    include(FetchContent)
    FetchContent_Declare(
        eigen3
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG        3.4.0
        GIT_SHALLOW    TRUE
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    set(EIGEN_BUILD_DOC       OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_BLAS      OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_LAPACK    OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTING         OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(eigen3)
endif()

# ── Qt 6 ──────────────────────────────────────────────────────
find_package(Qt6 REQUIRED COMPONENTS Core Gui Quick Quick3D Quick3DHelpers QuickControls2)

qt_policy(SET QTP0001 NEW)
qt_policy(SET QTP0004 NEW)

# ── AF_Core : domaine pur (Eigen, ZÉRO Qt) ────────────────────
add_subdirectory(core)

# ── Exécutable principal ──────────────────────────────────────
qt_add_executable(ArmForge WIN32
    main.cpp
)

# ── Module QML ────────────────────────────────────────────────
qt_add_qml_module(ArmForge
    URI ArmForge
    VERSION 1.0
    SOURCES
        app/controllers/robot_controller.h
        app/controllers/robot_controller.cpp
        app/controllers/planning_controller.h
        app/controllers/planning_controller.cpp
        app/control/control_loop.h
        app/control/control_loop.cpp
    QML_FILES
        ui/Main.qml
        ui/scene/SceneView.qml
        ui/scene/RobotView.qml
        ui/scene/GizmoView.qml
        ui/scene/PlanningView.qml
        ui/panels/JogPanel.qml
        ui/panels/ChartPanel.qml
        ui/panels/PlanningPanel.qml
)

target_include_directories(ArmForge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/app/controllers
    ${CMAKE_CURRENT_SOURCE_DIR}/app/control
)

target_link_libraries(ArmForge PRIVATE
    AF_Core
    Qt6::Core
    Qt6::Quick
    Qt6::Quick3D
    Qt6::Quick3DHelpers
    Qt6::QuickControls2
)

set_target_properties(ArmForge PROPERTIES
    MACOSX_BUNDLE_BUNDLE_VERSION   ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING
        ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

if(MSVC)
    target_compile_options(ArmForge PRIVATE /utf-8)
endif()
