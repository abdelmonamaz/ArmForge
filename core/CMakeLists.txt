add_library(AF_Core STATIC
    math/se3.h
    math/rotation.h
    model/dh_parameters.h
    model/joint.h
    model/kinematic_chain.h
    model/kinematic_chain.cpp
    kinematics/forward_kinematics.h
    kinematics/forward_kinematics.cpp
    kinematics/ik_solver.h
    kinematics/ik_solver.cpp
    planning/obstacle.h
    planning/collision.h
    planning/i_motion_planner.h
    planning/rrt_connect_planner.h
    planning/rrt_connect_planner.cpp
)

target_include_directories(AF_Core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(AF_Core PUBLIC Eigen3::Eigen)

if(MSVC)
    target_compile_options(AF_Core PRIVATE /utf-8)
endif()

# ── CudaRrtPlanner (Phase 5 — only built when the CUDA Toolkit was found) ──
# collision_kernel.cu is compiled by an explicit add_custom_command that calls
# nvcc directly (VERBATIM-quoted), instead of CMake's native CUDA-language
# rules — see the note in the top-level CMakeLists.txt: those native rules
# mis-quote the command line under the "NMake Makefiles JOM" generator and
# nvcc fails with "A single input file is required for a non-link phase...".
# The resulting .obj is attached to AF_Core as a plain external object file.
if(ARMFORGE_HAS_CUDA)
    set(AF_CUDA_KERNEL_SRC "${CMAKE_CURRENT_SOURCE_DIR}/planning/cuda/collision_kernel.cu")
    set(AF_CUDA_KERNEL_OBJ "${CMAKE_CURRENT_BINARY_DIR}/collision_kernel.cu.obj")

    add_custom_command(
        OUTPUT  "${AF_CUDA_KERNEL_OBJ}"
        COMMAND "${CUDAToolkit_NVCC_EXECUTABLE}"
                -ccbin "${CMAKE_CXX_COMPILER}"
                -c "${AF_CUDA_KERNEL_SRC}"
                -o "${AF_CUDA_KERNEL_OBJ}"
                -arch=sm_75
                --std=c++17
                -I"${CMAKE_CURRENT_SOURCE_DIR}"
                -Xcompiler=/utf-8
                "-Xcompiler=$<IF:$<CONFIG:Debug>,/MDd,/MD>"
        DEPENDS "${AF_CUDA_KERNEL_SRC}" "${CMAKE_CURRENT_SOURCE_DIR}/planning/cuda/collision_kernel.cuh"
        COMMENT "nvcc: compiling collision_kernel.cu directly (bypasses the NMake+CUDA quoting bug)"
        VERBATIM
    )

    target_sources(AF_Core PRIVATE
        planning/cuda/collision_kernel.cuh
        planning/cuda/cuda_rrt_planner.h
        planning/cuda/cuda_rrt_planner.cpp
        "${AF_CUDA_KERNEL_OBJ}"
    )
    set_source_files_properties("${AF_CUDA_KERNEL_OBJ}" PROPERTIES
        EXTERNAL_OBJECT TRUE
        GENERATED TRUE
    )
    target_compile_definitions(AF_Core PUBLIC ARMFORGE_HAS_CUDA)
    target_link_libraries(AF_Core PUBLIC CUDA::cudart)
endif()
