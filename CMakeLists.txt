cmake_minimum_required(VERSION 3.16)
project(openarm_sim_cpp)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(OPENARM_BUILD_LCM_TOOLS "Build LCM pose pub/sub tools" ON)

find_package(ompl QUIET)
if(ompl_FOUND)
    message(STATUS "Found OMPL")
else()
    message(STATUS "OMPL not found. Building with interpolation planner fallback.")
endif()

# --- 1. Find Pinocchio ---
find_program(SHELL_PYTHON3_EXECUTABLE NAMES python3 REQUIRED)
find_package(Eigen3 QUIET)
find_package(pinocchio QUIET)

if(pinocchio_FOUND)
    message(STATUS "Found Pinocchio: ${pinocchio_DIR}")
    add_definitions(-DPINOCCHIO_WITH_URDFDOM) # 确保支持 URDF
else()
    message(FATAL_ERROR "Pinocchio not found! Please install ros-<distro>-pinocchio")
endif()

# --- 2. Configure MuJoCo (Match active Python package version) ---
execute_process(
    COMMAND "${SHELL_PYTHON3_EXECUTABLE}" -c
            "import os, mujoco; print(os.path.dirname(mujoco.__file__))"
    OUTPUT_VARIABLE MUJOCO_PY_LIB_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE MUJOCO_PY_QUERY_RESULT
)

if(NOT MUJOCO_PY_QUERY_RESULT EQUAL 0 OR MUJOCO_PY_LIB_DIR STREQUAL "")
    message(FATAL_ERROR "Could not resolve MuJoCo package path from ${SHELL_PYTHON3_EXECUTABLE}")
endif()

set(MUJOCO_PY_INCLUDE_DIR "${MUJOCO_PY_LIB_DIR}/include")
unset(MUJOCO_RUNTIME_LIB CACHE)
find_library(MUJOCO_RUNTIME_LIB
    NAMES mujoco libmujoco.so.3.5.0 libmujoco.so.3.4.0 libmujoco.so.3
    PATHS "${MUJOCO_PY_LIB_DIR}"
    NO_DEFAULT_PATH
)

if(EXISTS "${MUJOCO_PY_INCLUDE_DIR}/mujoco/mujoco.h" AND MUJOCO_RUNTIME_LIB)
    message(STATUS "Found MuJoCo headers: ${MUJOCO_PY_INCLUDE_DIR}")
    message(STATUS "Found MuJoCo runtime: ${MUJOCO_RUNTIME_LIB}")
    add_library(mujoco::mujoco SHARED IMPORTED GLOBAL)
    set_target_properties(mujoco::mujoco PROPERTIES
        IMPORTED_LOCATION "${MUJOCO_RUNTIME_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_PY_INCLUDE_DIR}"
    )
else()
    message(FATAL_ERROR "Could not find MuJoCo headers/runtime library.")
endif()

# --- 3. Collect Source Files ---
# 包含控制器实现文件
set(SOURCE_FILES 
    src/main_interface.cpp
    src/cartesian_impedance_controller.cpp
)

include_directories(include)

# --- 4. Build Shared Library ---
add_library(openarm_core SHARED ${SOURCE_FILES})

# --- 5. Link Libraries ---
target_link_libraries(openarm_core PUBLIC mujoco::mujoco pinocchio::pinocchio)

# --- 6. Output Settings ---
set_target_properties(openarm_core PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}"
    PREFIX ""
    BUILD_RPATH "${MUJOCO_PY_LIB_DIR}"
)

# --- 7. Optional: LCM Lightweight Communication ---
if(OPENARM_BUILD_LCM_TOOLS)
    find_package(PkgConfig QUIET)
    if(NOT PkgConfig_FOUND)
        message(WARNING "PkgConfig not found. Skip LCM tools.")
    else()
        pkg_check_modules(LCM QUIET lcm)
        find_program(LCM_GEN_EXECUTABLE lcm-gen)

        if(NOT LCM_FOUND)
            message(WARNING "LCM library not found (pkg-config lcm). Skip LCM tools.")
        elseif(NOT LCM_GEN_EXECUTABLE)
            message(WARNING "lcm-gen not found. Skip LCM tools.")
        else()
            message(STATUS "Found LCM: ${LCM_VERSION}")
            message(STATUS "Found lcm-gen: ${LCM_GEN_EXECUTABLE}")

            set(LCM_TYPES_DIR "${CMAKE_SOURCE_DIR}/lcm_types")
            set(LCM_GEN_DIR "${CMAKE_BINARY_DIR}/generated/lcm")
            set(LCM_TYPE_FILE "${LCM_TYPES_DIR}/openarm_pose.lcm")
            set(LCM_GEN_HEADER_POSE "${LCM_GEN_DIR}/openarm_lcm/ObjectPose.hpp")
            set(LCM_GEN_HEADER_POSE_ARRAY "${LCM_GEN_DIR}/openarm_lcm/ObjectPoseArray.hpp")

            add_custom_command(
                OUTPUT "${LCM_GEN_HEADER_POSE}" "${LCM_GEN_HEADER_POSE_ARRAY}"
                COMMAND "${CMAKE_COMMAND}" -E make_directory "${LCM_GEN_DIR}"
                COMMAND "${LCM_GEN_EXECUTABLE}" --cpp-hpath "${LCM_GEN_DIR}" -x "${LCM_TYPE_FILE}"
                DEPENDS "${LCM_TYPE_FILE}"
                COMMENT "Generating LCM C++ headers from ${LCM_TYPE_FILE}"
                VERBATIM
            )

            add_custom_target(generate_lcm_types
                DEPENDS "${LCM_GEN_HEADER_POSE}" "${LCM_GEN_HEADER_POSE_ARRAY}"
            )

            add_library(openarm_closed_loop SHARED
                src/closed_loop_interface.cpp
                src/publisher/lcm_pose_publisher.cpp
                src/subscriber/lcm_pose_subscriber.cpp
                src/state_machine/grasp_state_machine.cpp
                src/robot/robot_kinematics.cpp
                src/task/interpolation_motion_planner.cpp
                src/task/manipulate_task.cpp
                src/task/ompl_motion_planner.cpp
                src/task/trajectory_executor.cpp
                src/runtime/closed_loop_runtime.cpp
            )

            add_dependencies(openarm_closed_loop generate_lcm_types)

            target_include_directories(openarm_closed_loop PRIVATE
                include
                "${LCM_GEN_DIR}"
                ${LCM_INCLUDE_DIRS}
            )

            target_link_libraries(openarm_closed_loop PUBLIC
                mujoco::mujoco
                pinocchio::pinocchio
                ${LCM_LIBRARIES}
            )
            if(ompl_FOUND)
                target_include_directories(openarm_closed_loop PRIVATE ${OMPL_INCLUDE_DIRS})
                target_link_libraries(openarm_closed_loop PUBLIC ${OMPL_LIBRARIES})
                target_compile_definitions(openarm_closed_loop PRIVATE OPENARM_HAVE_OMPL=1)
            endif()
            target_compile_options(openarm_closed_loop PRIVATE ${LCM_CFLAGS_OTHER})

            set_target_properties(openarm_closed_loop PROPERTIES
                LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}"
                PREFIX ""
                BUILD_RPATH "${MUJOCO_PY_LIB_DIR}"
            )
        endif()
    endif()
endif()
