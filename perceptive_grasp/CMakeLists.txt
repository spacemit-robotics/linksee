cmake_minimum_required(VERSION 3.16)
project(perceptive_grasp LANGUAGES C CXX)

include(GNUInstallDirs)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# 抑制 K3 镜像上 libvision.so 已知的 OpenCV DT_NEEDED 链接警告。
# SDK 视觉库与系统 OpenCV 运行时保持 ABI 兼容。
if(CMAKE_SYSTEM_PROCESSOR MATCHES "riscv" OR CMAKE_CROSSCOMPILING)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-w")
endif()

# ============ 构建选项 ============
# 检测器：VisionService SDK 或 OpenCV DNN。
# 执行器：真实 SO101 硬件或 mock 日志执行器。
option(X86_STANDALONE "Shortcut: OPENCV_DNN detector + MOCK executor" OFF)
option(USE_OPENCV_DNN "Use OpenCV DNN for detection (no VisionService needed)" OFF)
option(USE_MOCK_EXECUTOR "Use mock executor (no real arm)" OFF)
option(ENABLE_ROS2_VOICE "Enable optional ROS2 std_msgs/String voice bridge" OFF)
option(ENABLE_LAS2_CAMERA
       "Enable optional spacemit_las2 stereo camera backend" ON)
option(ENABLE_WEBRTC_AEC
       "Enable WebRTC acoustic echo cancellation for the voice bridge" ON)
option(ENABLE_REMOTE_MUJOCO
       "Enable remote MuJoCo client backends for split PC/K3 simulation" OFF)
option(ENABLE_MUJOCO_EXECUTOR
       "Enable optional MuJoCo UR5e executor backend for PC simulation" OFF)

# X86_STANDALONE 是主机侧规划与仿真验证构建的快捷选项。
if(X86_STANDALONE)
    set(USE_OPENCV_DNN ON)
    set(USE_MOCK_EXECUTOR ON)
endif()

# ============ 通用依赖 ============

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui imgcodecs dnn calib3d)
find_package(yaml-cpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(PkgConfig REQUIRED)
pkg_check_modules(URDFDOM REQUIRED IMPORTED_TARGET urdfdom)

set(HAVE_WEBRTC_AEC OFF)
if(ENABLE_WEBRTC_AEC)
    find_program(MESON_EXECUTABLE meson)
    find_program(NINJA_EXECUTABLE ninja)
    if(MESON_EXECUTABLE AND NINJA_EXECUTABLE)
        include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/webrtc_audio_processing.cmake")
        set(HAVE_WEBRTC_AEC ON)
        message(STATUS "[Voice] WebRTC AEC enabled")
    else()
        message(FATAL_ERROR
            "[Voice] WebRTC AEC requires meson and ninja-build; install "
            "them or configure with -DENABLE_WEBRTC_AEC=OFF")
    endif()
endif()

set(HAVE_MUJOCO_EXECUTOR OFF)
set(HAVE_MUJOCO_CAMERA OFF)
if(ENABLE_MUJOCO_EXECUTOR)
    if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
        message(FATAL_ERROR
            "[Executor] MuJoCo executor is PC/x86_64 only; disable "
            "ENABLE_MUJOCO_EXECUTOR for K3 builds")
    endif()
    execute_process(
        COMMAND ${Python3_EXECUTABLE}
                "${CMAKE_CURRENT_SOURCE_DIR}/scripts/prepare_mujoco_assets.py"
        RESULT_VARIABLE MUJOCO_ASSET_RESULT
    )
    if(NOT MUJOCO_ASSET_RESULT EQUAL 0)
        message(FATAL_ERROR
            "[Simulation] failed to prepare pinned MuJoCo Menagerie assets")
    endif()
    include("${CMAKE_CURRENT_SOURCE_DIR}/../../../../components/simulation/mujoco/cmake/FindMuJoCo.cmake")
    set(HAVE_MUJOCO_EXECUTOR ON)
    message(STATUS "[Executor] mujoco ur5e simulation backend enabled")
    find_package(glfw3 QUIET)
    if(glfw3_FOUND)
        set(HAVE_MUJOCO_CAMERA ON)
        message(STATUS "[Camera] mujoco rendered camera backend enabled")
    else()
        message(STATUS "[Camera] MuJoCo rendered camera backend disabled: glfw3 not found")
    endif()
endif()

set(REALSENSE2_HINTS)
if(EXISTS "/opt/ros/humble/lib/riscv64-linux-gnu/cmake/realsense2")
    list(APPEND REALSENSE2_HINTS
        "/opt/ros/humble"
        "/opt/ros/humble/lib/riscv64-linux-gnu/cmake/realsense2")
endif()
find_package(realsense2 QUIET CONFIG PATHS ${REALSENSE2_HINTS})
set(HAVE_REALSENSE_CAMERA OFF)
if(realsense2_FOUND)
    set(HAVE_REALSENSE_CAMERA ON)
    message(STATUS "[Camera] realsense backend found")
else()
    message(STATUS "[Camera] realsense backend not found")
endif()

set(HAVE_ROS2_VOICE OFF)
if(ENABLE_ROS2_VOICE)
    find_package(rclcpp QUIET)
    find_package(std_msgs QUIET)
    if(rclcpp_FOUND AND std_msgs_FOUND)
        set(HAVE_ROS2_VOICE ON)
        message(STATUS "[Voice] ROS2 subscriber enabled")
    else()
        message(WARNING
            "[Voice] ROS2 voice requested but rclcpp/std_msgs not found; "
            "building without voice subscriber")
    endif()
endif()

# ============ 源文件 ============

set(SOURCES
    src/main.cpp
    src/camera_calibration.cpp
    src/grasp_depth.cpp
    src/grasp_geometry.cpp
    src/gripper_holding.cpp
    src/grasp_planner.cpp
    src/grasp_pipeline.cpp
    src/grasp_pipeline_execution.cpp
    src/grasp_pipeline_runtime.cpp
    src/simulation_color_detector.cpp
    src/stereo_camera.cpp
    src/stereo_geometry.cpp
    src/target_association.cpp
    src/mobile_base_controller.cpp
    src/orientation_estimator.cpp
    src/pipeline_debug_writer.cpp
    src/pipeline_state.cpp
    src/pipeline_timing.cpp
    src/voice_command_parser.cpp
)

set(HAVE_REMOTE_MUJOCO_CAMERA OFF)
if(ENABLE_REMOTE_MUJOCO)
    set(HAVE_REMOTE_MUJOCO_CAMERA ON)
    list(APPEND SOURCES
        src/remote_mujoco_camera.cpp
        src/remote_mujoco_grasp_executor.cpp
        src/remote_mujoco_protocol.cpp
    )
    add_definitions(-DHAVE_REMOTE_MUJOCO=1)
    message(STATUS "[Simulation] remote_mujoco client backend enabled")
endif()

if(HAVE_REALSENSE_CAMERA)
    list(APPEND SOURCES src/depth_camera.cpp)
    add_definitions(-DHAVE_REALSENSE_CAMERA=1)
endif()

if(HAVE_ROS2_VOICE)
    list(APPEND SOURCES src/voice_command_listener.cpp)
endif()

set(EXTRA_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

set(EXTRA_LIBRARIES
    ${OpenCV_LIBS}
    yaml-cpp
    pthread
)
if(HAVE_REALSENSE_CAMERA)
    list(APPEND EXTRA_LIBRARIES realsense2::realsense2)
endif()
set(EXTRA_LINK_OPTIONS)
set(EXTRA_BUILD_RPATH)
set(EXTRA_INSTALL_RPATH)

# ============ 可选底盘控制 ============

if(NOT DEFINED _STAGING_DIR)
    set(_STAGING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../output/staging")
endif()

# ============ 可选 spacemit_las2 双目相机 ============

set(HAVE_LAS2_CAMERA OFF)
if(ENABLE_LAS2_CAMERA)
    set(_LAS2_HINTS)
    if(DEFINED LAS2_RUNTIME_DIR)
        list(APPEND _LAS2_HINTS "${LAS2_RUNTIME_DIR}")
    endif()
    if(DEFINED ENV{LAS2_RUNTIME_DIR})
        list(APPEND _LAS2_HINTS "$ENV{LAS2_RUNTIME_DIR}")
    endif()
    if(DEFINED ENV{HOME})
        list(APPEND _LAS2_HINTS "$ENV{HOME}/las2_runtime")
    endif()
    list(REMOVE_DUPLICATES _LAS2_HINTS)

    find_path(LAS2_INCLUDE_DIR las2_usb_stereo.h
        PATHS
            ${_LAS2_HINTS}
            "${_STAGING_DIR}"
        PATH_SUFFIXES include
        NO_DEFAULT_PATH
    )
    find_library(LAS2_LIBRARY las2_usb_stereo
        PATHS
            ${_LAS2_HINTS}
            "${_STAGING_DIR}"
        PATH_SUFFIXES lib
        NO_DEFAULT_PATH
    )

    if(LAS2_INCLUDE_DIR AND LAS2_LIBRARY)
        set(HAVE_LAS2_CAMERA ON)
        message(STATUS "[Camera] spacemit_las2 backend found")
        list(APPEND SOURCES src/las2_stereo_camera.cpp)
        list(APPEND EXTRA_INCLUDE_DIRS ${LAS2_INCLUDE_DIR})
        get_filename_component(_LAS2_LIB_DIR "${LAS2_LIBRARY}" DIRECTORY)
        get_filename_component(_LAS2_ROOT_DIR "${_LAS2_LIB_DIR}" DIRECTORY)

        # The delivered library carries RUNPATH entries from its build host.
        # Patch private build copies so transitive OpenCV/ORT dependencies
        # resolve on the deployed K3 without modifying the runtime package.
        set(_LAS2_LINK_LIBRARY "${LAS2_LIBRARY}")
        set(_LAS2_INSTALL_LIBRARIES "${LAS2_LIBRARY}")
        find_file(_LAS2_OPENCV_CALIB3D_LIBRARY
            NAMES libopencv_calib3d.so.413
            PATHS "${_LAS2_ROOT_DIR}/lib/opencv"
            NO_DEFAULT_PATH
        )
        find_program(PATCHELF_EXECUTABLE patchelf)
        if(PATCHELF_EXECUTABLE AND _LAS2_OPENCV_CALIB3D_LIBRARY)
            set(_LAS2_PATCH_DIR "${CMAKE_BINARY_DIR}/las2_runtime")
            set(_LAS2_PATCHED_LIBRARY
                "${_LAS2_PATCH_DIR}/liblas2_usb_stereo.so")
            set(_LAS2_PATCHED_OPENCV_CALIB3D
                "${_LAS2_PATCH_DIR}/libopencv_calib3d.so.413")
            string(CONCAT _LAS2_PRIVATE_RPATH
                "\$ORIGIN:${_LAS2_LIB_DIR}:"
                "${_LAS2_ROOT_DIR}/lib/spacemit_ort:"
                "${_LAS2_ROOT_DIR}/lib/opencv")
            add_custom_command(
                OUTPUT
                    "${_LAS2_PATCHED_LIBRARY}"
                    "${_LAS2_PATCHED_OPENCV_CALIB3D}"
                COMMAND "${CMAKE_COMMAND}" -E make_directory
                    "${_LAS2_PATCH_DIR}"
                COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                    "${LAS2_LIBRARY}" "${_LAS2_PATCHED_LIBRARY}"
                COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                    "${_LAS2_OPENCV_CALIB3D_LIBRARY}"
                    "${_LAS2_PATCHED_OPENCV_CALIB3D}"
                COMMAND "${PATCHELF_EXECUTABLE}" --set-rpath
                    "${_LAS2_PRIVATE_RPATH}" "${_LAS2_PATCHED_LIBRARY}"
                COMMAND "${PATCHELF_EXECUTABLE}" --set-rpath
                    "${_LAS2_ROOT_DIR}/lib/opencv"
                    "${_LAS2_PATCHED_OPENCV_CALIB3D}"
                DEPENDS
                    "${LAS2_LIBRARY}"
                    "${_LAS2_OPENCV_CALIB3D_LIBRARY}"
                COMMENT "Preparing private spacemit_las2 runtime"
                VERBATIM
            )
            add_custom_target(spacemit_las2_runtime
                DEPENDS
                    "${_LAS2_PATCHED_LIBRARY}"
                    "${_LAS2_PATCHED_OPENCV_CALIB3D}")
            set(_LAS2_RUNTIME_TARGET spacemit_las2_runtime)
            set(_LAS2_LINK_LIBRARY "${_LAS2_PATCHED_LIBRARY}")
            set(_LAS2_INSTALL_LIBRARIES
                "${_LAS2_PATCHED_LIBRARY}"
                "${_LAS2_PATCHED_OPENCV_CALIB3D}")
            list(APPEND EXTRA_BUILD_RPATH "${_LAS2_PATCH_DIR}")
            message(STATUS
                "[Camera] LAS2 private runtime will be prepared during build")
        else()
            message(WARNING
                "[Camera] patchelf or LAS2 OpenCV calib3d library not found; "
                "LD_LIBRARY_PATH is required for LAS2 private dependencies")
        endif()
        list(APPEND EXTRA_LIBRARIES "${_LAS2_LINK_LIBRARY}")

        foreach(_LAS2_DEP_DIR
                "${_LAS2_LIB_DIR}"
                "${_LAS2_ROOT_DIR}/lib/spacemit_ort"
                "${_LAS2_ROOT_DIR}/lib/opencv")
            list(APPEND EXTRA_LINK_OPTIONS "-Wl,-rpath-link,${_LAS2_DEP_DIR}")
            list(APPEND EXTRA_BUILD_RPATH "${_LAS2_DEP_DIR}")
        endforeach()
        list(APPEND EXTRA_INSTALL_RPATH
            "\$ORIGIN/../${CMAKE_INSTALL_LIBDIR}/perceptive_grasp")
        add_definitions(-DHAVE_LAS2_CAMERA=1)
    else()
        message(STATUS
            "[Camera] spacemit_las2 backend not found; "
            "camera.type=spacemit_las2 will fail with a clear runtime message")
    endif()
endif()

if(NOT HAVE_REALSENSE_CAMERA AND NOT HAVE_LAS2_CAMERA AND
   NOT HAVE_MUJOCO_CAMERA AND NOT HAVE_REMOTE_MUJOCO_CAMERA AND
   NOT X86_STANDALONE)
    message(FATAL_ERROR
        "No stereo camera backend found. Install realsense dependencies or "
        "place spacemit_las2 in ~/las2_runtime.")
endif()

find_path(CHASSIS_INCLUDE_DIR chassis.h
    PATHS "${_STAGING_DIR}/include"
    NO_DEFAULT_PATH
)
find_library(CHASSIS_LIBRARY chassis
    PATHS "${_STAGING_DIR}/lib"
    NO_DEFAULT_PATH
)

if(CHASSIS_INCLUDE_DIR AND CHASSIS_LIBRARY)
    message(STATUS "[MobileBase] chassis SDK enabled")
    list(APPEND EXTRA_INCLUDE_DIRS ${CHASSIS_INCLUDE_DIR})
    list(APPEND EXTRA_LIBRARIES ${CHASSIS_LIBRARY})
    add_definitions(-DHAVE_CHASSIS=1)
else()
    message(WARNING
        "[MobileBase] chassis SDK not found; mobile_base.enabled=true "
        "will fail at runtime")
endif()

# ============ 检测器选择 ============

if(USE_OPENCV_DNN)
    message(STATUS "[Detector] OpenCV DNN (no VisionService)")
    list(APPEND SOURCES src/mock/mock_detector.cpp)
    list(APPEND EXTRA_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/mock)
    add_definitions(-DMOCK_DETECTOR=1)
else()
    message(STATUS "[Detector] VisionService SDK")

    set(_STAGING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../output/staging")
    set(_VISION_THIRD_PARTY "${CMAKE_CURRENT_SOURCE_DIR}/third_party/vision")
    set(_VISION_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../components/model_zoo/vision")

    find_path(VISION_INCLUDE_DIR vision_service.h
        PATHS
            "${_VISION_THIRD_PARTY}/include"
            "${_STAGING_DIR}/include"
            "${_VISION_SRC_DIR}/include"
        NO_DEFAULT_PATH
    )
    if(NOT VISION_INCLUDE_DIR)
        find_path(VISION_INCLUDE_DIR vision_service.h)
    endif()

    find_library(VISION_LIBRARY vision
        PATHS
            "${_VISION_THIRD_PARTY}/lib"
            "${_STAGING_DIR}/lib"
        NO_DEFAULT_PATH
    )
    if(NOT VISION_LIBRARY)
        find_library(VISION_LIBRARY vision)
    endif()

    if(NOT VISION_INCLUDE_DIR OR NOT VISION_LIBRARY)
        message(FATAL_ERROR
            "VisionService not found.\n"
            "  VISION_INCLUDE_DIR=${VISION_INCLUDE_DIR}\n"
            "  VISION_LIBRARY=${VISION_LIBRARY}\n"
            "Hint: Build components/model_zoo/vision first, or use -DUSE_OPENCV_DNN=ON")
    endif()

    list(APPEND SOURCES src/target_detector.cpp)
    list(APPEND EXTRA_INCLUDE_DIRS ${VISION_INCLUDE_DIR})
    list(APPEND EXTRA_LIBRARIES ${VISION_LIBRARY})
endif()

# MuJoCo is self-contained and must not inherit real manipulator SDK links.
if(HAVE_MUJOCO_EXECUTOR)
    list(APPEND SOURCES
        src/mujoco_grasp_executor.cpp
        src/mujoco_simulation.cpp
    )
    list(APPEND EXTRA_INCLUDE_DIRS ${MUJOCO_INCLUDE_DIR})
    list(APPEND EXTRA_LIBRARIES ${MUJOCO_LIB})
    add_definitions(-DHAVE_MUJOCO_EXECUTOR=1)
    if(HAVE_MUJOCO_CAMERA)
        list(APPEND SOURCES src/mujoco_stereo_camera.cpp)
        list(APPEND EXTRA_LIBRARIES glfw)
        add_definitions(-DHAVE_MUJOCO_CAMERA=1)
    endif()
endif()

# ============ 执行器选择 ============

if(USE_MOCK_EXECUTOR)
    message(STATUS "[Executor] MOCK (print only)")
    list(APPEND SOURCES src/mock/mock_executor.cpp)
    if(NOT USE_OPENCV_DNN)
        list(APPEND EXTRA_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/mock)
    endif()
    add_definitions(-DMOCK_EXECUTOR=1)
else()
    message(STATUS "[Executor] REAL (manipulator + grasp)")

    if(NOT DEFINED _STAGING_DIR)
        set(_STAGING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../output/staging")
    endif()

    find_path(MANIP_INCLUDE_DIR manipulator.h
        PATHS "${_STAGING_DIR}/include"
        NO_DEFAULT_PATH
    )
    find_library(MANIP_LIBRARY manipulator
        PATHS "${_STAGING_DIR}/lib"
        NO_DEFAULT_PATH
    )
    find_library(GRASP_LIBRARY grasp
        PATHS "${_STAGING_DIR}/lib"
        NO_DEFAULT_PATH
    )

    if(NOT MANIP_INCLUDE_DIR OR NOT MANIP_LIBRARY OR NOT GRASP_LIBRARY)
        message(FATAL_ERROR
            "Manipulator/Grasp SDK not found in staging.\n"
            "Build components/control first, or use -DUSE_MOCK_EXECUTOR=ON")
    endif()

    list(APPEND SOURCES
        src/grasp_executor.cpp
        src/grasp_executor_gripper.cpp
        src/grasp_executor_runtime.cpp
        src/arm_path_safety.cpp
        src/motion_completion.cpp
    )
    list(APPEND EXTRA_INCLUDE_DIRS ${MANIP_INCLUDE_DIR})
    list(APPEND EXTRA_LIBRARIES
        ${MANIP_LIBRARY}
        ${GRASP_LIBRARY}
        PkgConfig::URDFDOM
    )
endif()

# ============ 构建摘要 ============

message(STATUS "")
message(STATUS "=== perceptive_grasp Build Summary ===")
if(USE_OPENCV_DNN)
    message(STATUS "  Detector:  OpenCV DNN")
else()
    message(STATUS "  Detector:  VisionService")
endif()
if(USE_MOCK_EXECUTOR AND HAVE_MUJOCO_EXECUTOR)
    message(STATUS "  Executor:  mock pipeline, mujoco ur5e service")
elseif(USE_MOCK_EXECUTOR)
    message(STATUS "  Executor:  mock (print only)")
elseif(HAVE_MUJOCO_EXECUTOR)
    message(STATUS "  Executor:  real hardware pipeline, mujoco ur5e service")
else()
    message(STATUS "  Executor:  real hardware")
endif()
set(_CAMERA_BACKENDS)
if(HAVE_REALSENSE_CAMERA)
    list(APPEND _CAMERA_BACKENDS "realsense")
endif()
if(HAVE_LAS2_CAMERA)
    list(APPEND _CAMERA_BACKENDS "spacemit_las2")
endif()
if(HAVE_MUJOCO_CAMERA)
    list(APPEND _CAMERA_BACKENDS "mujoco")
endif()
if(HAVE_REMOTE_MUJOCO_CAMERA)
    list(APPEND _CAMERA_BACKENDS "remote_mujoco")
endif()
list(JOIN _CAMERA_BACKENDS ", " _CAMERA_BACKENDS_SUMMARY)
message(STATUS "  Camera:    ${_CAMERA_BACKENDS_SUMMARY}")
message(STATUS "")

# ============ 主程序与日志入口 ============

add_executable(perceptive_grasp_core ${SOURCES})
if(DEFINED _LAS2_RUNTIME_TARGET)
    add_dependencies(perceptive_grasp_core ${_LAS2_RUNTIME_TARGET})
endif()

target_include_directories(perceptive_grasp_core PRIVATE ${EXTRA_INCLUDE_DIRS})
target_link_libraries(perceptive_grasp_core PRIVATE ${EXTRA_LIBRARIES})
target_link_options(perceptive_grasp_core PRIVATE ${EXTRA_LINK_OPTIONS})
if(EXTRA_BUILD_RPATH)
    set_target_properties(perceptive_grasp_core PROPERTIES
        BUILD_RPATH "${EXTRA_BUILD_RPATH}"
    )
endif()
if(EXTRA_INSTALL_RPATH)
    set_target_properties(perceptive_grasp_core PROPERTIES
        INSTALL_RPATH "${EXTRA_INSTALL_RPATH}"
    )
endif()
if(HAVE_ROS2_VOICE)
    target_compile_definitions(perceptive_grasp_core PRIVATE ENABLE_ROS2_VOICE=1)
    target_include_directories(perceptive_grasp_core PRIVATE
        ${rclcpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
    )
    if(TARGET rclcpp::rclcpp)
        target_link_libraries(perceptive_grasp_core PRIVATE rclcpp::rclcpp)
    else()
        target_link_libraries(perceptive_grasp_core PRIVATE ${rclcpp_LIBRARIES})
    endif()
    target_link_libraries(perceptive_grasp_core PRIVATE
        ${std_msgs_TARGETS}
        ${std_msgs_LIBRARIES}
    )
endif()

# The public executable filters third-party stdout and stderr before
# any SDK static initializer runs. --debug bypasses filtering.
add_custom_target(perceptive_grasp ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_perceptive_grasp.py"
        "${CMAKE_BINARY_DIR}/perceptive_grasp"
    COMMAND chmod +x "${CMAKE_BINARY_DIR}/perceptive_grasp"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/local_voice_bridge.py"
        "${CMAKE_BINARY_DIR}/local_voice_bridge.py"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/asr_node.py"
        "${CMAKE_BINARY_DIR}/asr_node.py"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/asr_backend.py"
        "${CMAKE_BINARY_DIR}/asr_backend.py"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/tts_node.py"
        "${CMAKE_BINARY_DIR}/tts_node.py"
    COMMAND chmod +x "${CMAKE_BINARY_DIR}/local_voice_bridge.py"
    DEPENDS
        perceptive_grasp_core
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_perceptive_grasp.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/local_voice_bridge.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/asr_backend.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/asr_node.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/tts_node.py"
    COMMENT "Preparing perceptive_grasp structured-log launcher"
)

# ============ 单元测试 ============

enable_testing()

add_executable(mobile_base_alignment_test
    tests/mobile_base_alignment_test.cpp
    src/mobile_base_controller.cpp
)
target_include_directories(mobile_base_alignment_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(mobile_base_alignment_test PRIVATE pthread)
if(CHASSIS_INCLUDE_DIR AND CHASSIS_LIBRARY)
    target_include_directories(mobile_base_alignment_test PRIVATE
        ${CHASSIS_INCLUDE_DIR}
    )
    target_link_libraries(mobile_base_alignment_test PRIVATE
        ${CHASSIS_LIBRARY}
    )
endif()
add_test(NAME mobile_base_alignment_test
    COMMAND mobile_base_alignment_test)

add_executable(stereo_geometry_test
    tests/stereo_geometry_test.cpp
    src/stereo_geometry.cpp
)
target_include_directories(stereo_geometry_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(stereo_geometry_test PRIVATE ${OpenCV_LIBS})
add_test(NAME stereo_geometry_test COMMAND stereo_geometry_test)

add_executable(camera_calibration_test
    tests/camera_calibration_test.cpp
    src/camera_calibration.cpp
)
target_include_directories(camera_calibration_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(camera_calibration_test PRIVATE yaml-cpp)
add_test(NAME camera_calibration_test COMMAND camera_calibration_test)

add_executable(grasp_depth_test
    tests/grasp_depth_test.cpp
    src/grasp_depth.cpp
)
target_include_directories(grasp_depth_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(grasp_depth_test PRIVATE ${OpenCV_LIBS})
add_test(NAME grasp_depth_test COMMAND grasp_depth_test)

add_executable(gripper_holding_test
    tests/gripper_holding_test.cpp
    src/gripper_holding.cpp
)
target_include_directories(gripper_holding_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
add_test(NAME gripper_holding_test COMMAND gripper_holding_test)

add_executable(target_association_test
    tests/target_association_test.cpp
    src/target_association.cpp
)
target_include_directories(target_association_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(target_association_test PRIVATE MOCK_DETECTOR=1)
target_link_libraries(target_association_test PRIVATE ${OpenCV_LIBS})
add_test(NAME target_association_test COMMAND target_association_test)

add_executable(simulation_color_detector_test
    tests/simulation_color_detector_test.cpp
    src/simulation_color_detector.cpp
)
target_include_directories(simulation_color_detector_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(simulation_color_detector_test PRIVATE
    MOCK_DETECTOR=1)
target_link_libraries(simulation_color_detector_test PRIVATE ${OpenCV_LIBS})
add_test(NAME simulation_color_detector_test
    COMMAND simulation_color_detector_test)

if(USE_OPENCV_DNN)
    add_executable(mock_detector_test
        tests/mock_detector_test.cpp
        src/mock/mock_detector.cpp
        src/simulation_color_detector.cpp
    )
    target_include_directories(mock_detector_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
    )
    target_compile_definitions(mock_detector_test PRIVATE MOCK_DETECTOR=1)
    target_link_libraries(mock_detector_test PRIVATE ${OpenCV_LIBS} yaml-cpp)
    add_test(NAME mock_detector_test COMMAND mock_detector_test)
endif()

add_test(NAME config_validation_test
    COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_config_validation.py
        --binary $<TARGET_FILE:perceptive_grasp_core>
        --config ${CMAKE_CURRENT_SOURCE_DIR}/config/grasp_pipeline.yaml
        --config ${CMAKE_CURRENT_SOURCE_DIR}/config/grasp_pipeline_mujoco_ur5e.yaml
        --config ${CMAKE_CURRENT_SOURCE_DIR}/config/grasp_pipeline_remote_mujoco_ur5e.yaml)

add_executable(grasp_geometry_test
    tests/grasp_geometry_test.cpp
    src/grasp_geometry.cpp
    src/grasp_planner.cpp
)
target_include_directories(grasp_geometry_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(grasp_geometry_test PRIVATE MOCK_DETECTOR=1)
target_link_libraries(grasp_geometry_test PRIVATE ${OpenCV_LIBS})
add_test(NAME grasp_geometry_test COMMAND grasp_geometry_test)

add_executable(grasp_strategy_regression_test
    tests/grasp_strategy_regression_test.cpp
    src/grasp_geometry.cpp
    src/grasp_planner.cpp
)
target_include_directories(grasp_strategy_regression_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(grasp_strategy_regression_test PRIVATE
    MOCK_DETECTOR=1)
target_link_libraries(grasp_strategy_regression_test PRIVATE ${OpenCV_LIBS})
add_test(NAME grasp_strategy_regression_test
    COMMAND grasp_strategy_regression_test)

add_executable(orientation_estimator_test
    tests/orientation_estimator_test.cpp
    src/orientation_estimator.cpp
)
target_include_directories(orientation_estimator_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(orientation_estimator_test PRIVATE MOCK_DETECTOR=1)
target_link_libraries(orientation_estimator_test PRIVATE ${OpenCV_LIBS})
add_test(NAME orientation_estimator_test COMMAND orientation_estimator_test)

add_executable(arm_path_safety_test
    tests/arm_path_safety_test.cpp
    src/arm_path_safety.cpp
)
target_include_directories(arm_path_safety_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_definitions(arm_path_safety_test PRIVATE
    PERCEPTIVE_GRASP_TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/urdf/so101.urdf"
)
target_link_libraries(arm_path_safety_test PRIVATE PkgConfig::URDFDOM)
add_test(NAME arm_path_safety_test COMMAND arm_path_safety_test)

add_executable(motion_completion_test
    tests/motion_completion_test.cpp
    src/motion_completion.cpp
)
target_include_directories(motion_completion_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
add_test(NAME motion_completion_test COMMAND motion_completion_test)

add_executable(pipeline_debug_writer_test
    tests/pipeline_debug_writer_test.cpp
    src/pipeline_debug_writer.cpp
)
target_include_directories(pipeline_debug_writer_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(pipeline_debug_writer_test PRIVATE ${OpenCV_LIBS})
add_test(
    NAME pipeline_debug_writer_test
    COMMAND pipeline_debug_writer_test
)

add_executable(pipeline_timing_test
    tests/pipeline_timing_test.cpp
    src/pipeline_state.cpp
    src/pipeline_timing.cpp
)
target_include_directories(pipeline_timing_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
add_test(NAME pipeline_timing_test COMMAND pipeline_timing_test)

add_executable(voice_command_parser_test
    tests/voice_command_parser_test.cpp
    src/voice_command_parser.cpp
)
target_include_directories(voice_command_parser_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
add_test(NAME voice_command_parser_test COMMAND voice_command_parser_test)

if(ENABLE_REMOTE_MUJOCO OR HAVE_MUJOCO_CAMERA)
    add_executable(remote_mujoco_protocol_test
        tests/remote_mujoco_protocol_test.cpp
        src/remote_mujoco_protocol.cpp
    )
    target_include_directories(remote_mujoco_protocol_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(remote_mujoco_protocol_test PRIVATE pthread)
    add_test(NAME remote_mujoco_protocol_test
        COMMAND remote_mujoco_protocol_test)
endif()

if(HAVE_MUJOCO_EXECUTOR)
    add_executable(mujoco_grasp_executor_test
        tests/mujoco_grasp_executor_test.cpp
        src/mujoco_grasp_executor.cpp
        src/mujoco_simulation.cpp
    )
    target_include_directories(mujoco_grasp_executor_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${MUJOCO_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(mujoco_grasp_executor_test PRIVATE
        ${MUJOCO_LIB}
        ${OpenCV_LIBS}
        pthread
    )
    target_compile_definitions(mujoco_grasp_executor_test PRIVATE
        HAVE_MUJOCO_EXECUTOR=1
        MOCK_EXECUTOR=1
        PERCEPTIVE_GRASP_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME mujoco_grasp_executor_test
        COMMAND mujoco_grasp_executor_test)
endif()

if(HAVE_MUJOCO_CAMERA)
    add_executable(mujoco_grasp_sim_server
        tools/mujoco_grasp_sim_server.cpp
        src/mujoco_grasp_executor.cpp
        src/mujoco_simulation.cpp
        src/mujoco_stereo_camera.cpp
        src/remote_mujoco_protocol.cpp
    )
    target_include_directories(mujoco_grasp_sim_server PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${MUJOCO_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(mujoco_grasp_sim_server PRIVATE
        ${MUJOCO_LIB}
        ${OpenCV_LIBS}
        yaml-cpp
        glfw
        pthread
    )
    target_compile_definitions(mujoco_grasp_sim_server PRIVATE
        HAVE_MUJOCO_EXECUTOR=1
        HAVE_MUJOCO_CAMERA=1
        MOCK_EXECUTOR=1)
    install(TARGETS mujoco_grasp_sim_server DESTINATION bin)
    install(DIRECTORY simulation/mujoco/
        DESTINATION share/perceptive_grasp/simulation/mujoco)
endif()

if(HAVE_WEBRTC_AEC)
    add_library(perceptive_voice_aec SHARED src/voice_aec_processor.cpp)
    add_dependencies(perceptive_voice_aec webrtc_audio_processing_project)
    target_include_directories(perceptive_voice_aec PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${WEBRTC_INCLUDE_DIRS}
    )
    target_link_libraries(perceptive_voice_aec PRIVATE ${WEBRTC_LIBRARY})
    set_target_properties(perceptive_voice_aec PROPERTIES
        BUILD_RPATH "${WEBRTC_BINARY_DIR}/webrtc/modules/audio_processing"
        INSTALL_RPATH "$ORIGIN"
    )

    add_executable(voice_aec_processor_test
        tests/voice_aec_processor_test.cpp)
    target_include_directories(voice_aec_processor_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(voice_aec_processor_test PRIVATE perceptive_voice_aec)
    add_test(NAME voice_aec_processor_test COMMAND voice_aec_processor_test)
endif()


# ============ 安装 ============

install(TARGETS perceptive_grasp_core DESTINATION bin)
if(HAVE_WEBRTC_AEC)
    install(TARGETS perceptive_voice_aec DESTINATION lib)
    install(FILES "${WEBRTC_RUNTIME_LIBRARY}" DESTINATION lib)
    install(FILES "${WEBRTC_SOURCE_DIR}/COPYING"
        DESTINATION share/licenses/perceptive_grasp
        RENAME webrtc-audio-processing.COPYING)
    install(FILES "${WEBRTC_SOURCE_DIR}/webrtc/LICENSE"
        DESTINATION share/licenses/perceptive_grasp
        RENAME webrtc.LICENSE)
    install(FILES "${ABSEIL_INCLUDE_DIR}/LICENSE"
        DESTINATION share/licenses/perceptive_grasp
        RENAME abseil.LICENSE
        OPTIONAL)
endif()
install(PROGRAMS scripts/run_perceptive_grasp.py
    DESTINATION bin
    RENAME perceptive_grasp)
install(DIRECTORY config/ DESTINATION share/perceptive_grasp/config)
install(DIRECTORY docs/ DESTINATION share/perceptive_grasp/docs)
install(DIRECTORY urdf/ DESTINATION share/perceptive_grasp/urdf)
install(FILES resources/eye-to-hand.jpg
    DESTINATION share/perceptive_grasp/resources)
if(HAVE_LAS2_CAMERA)
    install(FILES ${_LAS2_INSTALL_LIBRARIES}
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/perceptive_grasp")
endif()
install(PROGRAMS
    scripts/asr_backend.py
    scripts/asr_node.py
    scripts/check_runtime_env.py
    scripts/local_voice_bridge.py
    scripts/tts_node.py
    DESTINATION share/perceptive_grasp/scripts)

# ============ 立体相机调试工具 ============

if(HAVE_REALSENSE_CAMERA OR HAVE_LAS2_CAMERA OR HAVE_MUJOCO_CAMERA OR
   HAVE_REMOTE_MUJOCO_CAMERA)
    set(DEBUG_VIEW_SOURCES
        tools/debug_view.cpp
        src/stereo_camera.cpp
        src/stereo_geometry.cpp
    )
    if(HAVE_REMOTE_MUJOCO_CAMERA)
        list(APPEND DEBUG_VIEW_SOURCES
            src/remote_mujoco_camera.cpp
            src/remote_mujoco_protocol.cpp
        )
    endif()
    if(HAVE_REALSENSE_CAMERA)
        list(APPEND DEBUG_VIEW_SOURCES src/depth_camera.cpp)
    endif()
    if(HAVE_LAS2_CAMERA)
        list(APPEND DEBUG_VIEW_SOURCES src/las2_stereo_camera.cpp)
    endif()
    if(HAVE_MUJOCO_CAMERA)
        list(APPEND DEBUG_VIEW_SOURCES
            src/mujoco_stereo_camera.cpp
            src/mujoco_simulation.cpp
        )
    endif()

    add_executable(debug_view ${DEBUG_VIEW_SOURCES})
    target_include_directories(debug_view PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(debug_view PRIVATE
        ${OpenCV_LIBS}
        yaml-cpp
        pthread
    )
    if(HAVE_REALSENSE_CAMERA)
        target_link_libraries(debug_view PRIVATE realsense2::realsense2)
    endif()
    if(HAVE_LAS2_CAMERA)
        if(DEFINED _LAS2_RUNTIME_TARGET)
            add_dependencies(debug_view ${_LAS2_RUNTIME_TARGET})
        endif()
        target_include_directories(debug_view PRIVATE ${LAS2_INCLUDE_DIR})
        target_link_libraries(debug_view PRIVATE ${_LAS2_LINK_LIBRARY})
        target_link_options(debug_view PRIVATE ${EXTRA_LINK_OPTIONS})
        if(EXTRA_BUILD_RPATH)
            set_target_properties(debug_view PROPERTIES
                BUILD_RPATH "${EXTRA_BUILD_RPATH}"
            )
        endif()
        if(EXTRA_INSTALL_RPATH)
            set_target_properties(debug_view PROPERTIES
                INSTALL_RPATH "${EXTRA_INSTALL_RPATH}"
            )
        endif()
    endif()
    if(HAVE_MUJOCO_CAMERA)
        target_include_directories(debug_view PRIVATE ${MUJOCO_INCLUDE_DIR})
        target_link_libraries(debug_view PRIVATE ${MUJOCO_LIB} glfw)
    endif()

    if(USE_OPENCV_DNN)
        target_compile_definitions(debug_view PRIVATE NO_VISION_SERVICE=1)
    else()
        target_include_directories(debug_view PRIVATE ${VISION_INCLUDE_DIR})
        target_link_libraries(debug_view PRIVATE ${VISION_LIBRARY})
    endif()

    install(TARGETS debug_view DESTINATION bin)
else()
    message(STATUS
        "[Tools] debug_view skipped: no camera backend available")
endif()

if(HAVE_REALSENSE_CAMERA OR HAVE_LAS2_CAMERA OR HAVE_MUJOCO_CAMERA OR
   HAVE_REMOTE_MUJOCO_CAMERA)
    set(DEBUG_LOCALIZE_SOURCES
        tools/debug_localize.cpp
        src/camera_calibration.cpp
        src/grasp_geometry.cpp
        src/grasp_planner.cpp
        src/stereo_camera.cpp
        src/stereo_geometry.cpp
    )
    if(HAVE_REMOTE_MUJOCO_CAMERA)
        list(APPEND DEBUG_LOCALIZE_SOURCES
            src/remote_mujoco_camera.cpp
            src/remote_mujoco_protocol.cpp
        )
    endif()
    if(HAVE_REALSENSE_CAMERA)
        list(APPEND DEBUG_LOCALIZE_SOURCES src/depth_camera.cpp)
    endif()
    if(HAVE_LAS2_CAMERA)
        list(APPEND DEBUG_LOCALIZE_SOURCES src/las2_stereo_camera.cpp)
    endif()
    if(HAVE_MUJOCO_CAMERA)
        list(APPEND DEBUG_LOCALIZE_SOURCES
            src/mujoco_stereo_camera.cpp
            src/mujoco_simulation.cpp
        )
    endif()

    add_executable(debug_localize ${DEBUG_LOCALIZE_SOURCES})
    target_include_directories(debug_localize PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(debug_localize PRIVATE
        ${OpenCV_LIBS}
        yaml-cpp
        pthread
    )
    if(HAVE_REALSENSE_CAMERA)
        target_link_libraries(debug_localize PRIVATE
            realsense2::realsense2)
    endif()
    if(HAVE_LAS2_CAMERA)
        if(DEFINED _LAS2_RUNTIME_TARGET)
            add_dependencies(debug_localize ${_LAS2_RUNTIME_TARGET})
        endif()
        target_include_directories(debug_localize PRIVATE ${LAS2_INCLUDE_DIR})
        target_link_libraries(debug_localize PRIVATE ${_LAS2_LINK_LIBRARY})
        target_link_options(debug_localize PRIVATE ${EXTRA_LINK_OPTIONS})
        if(EXTRA_BUILD_RPATH)
            set_target_properties(debug_localize PROPERTIES
                BUILD_RPATH "${EXTRA_BUILD_RPATH}"
            )
        endif()
        if(EXTRA_INSTALL_RPATH)
            set_target_properties(debug_localize PROPERTIES
                INSTALL_RPATH "${EXTRA_INSTALL_RPATH}"
            )
        endif()
    endif()
    if(HAVE_MUJOCO_CAMERA)
        target_include_directories(debug_localize PRIVATE ${MUJOCO_INCLUDE_DIR})
        target_link_libraries(debug_localize PRIVATE ${MUJOCO_LIB} glfw)
    endif()

    if(USE_OPENCV_DNN)
        message(WARNING
            "debug_localize skipped: requires VisionService "
            "(USE_OPENCV_DNN=OFF)")
        set_target_properties(debug_localize PROPERTIES EXCLUDE_FROM_ALL TRUE)
    else()
        target_include_directories(debug_localize PRIVATE
            ${VISION_INCLUDE_DIR})
        target_link_libraries(debug_localize PRIVATE ${VISION_LIBRARY})
        install(TARGETS debug_localize DESTINATION bin)
    endif()
else()
    message(STATUS
        "[Tools] debug_localize skipped: no camera backend available")
endif()

if(HAVE_REALSENSE_CAMERA)
    add_executable(debug_grasp
        tools/debug_grasp.cpp
        src/camera_calibration.cpp
        src/grasp_planner.cpp
        src/orientation_estimator.cpp
    )
    target_include_directories(debug_grasp PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(debug_grasp PRIVATE
        realsense2::realsense2
        ${OpenCV_LIBS}
        yaml-cpp
        pthread
    )

    if(USE_OPENCV_DNN OR USE_MOCK_EXECUTOR)
        message(WARNING
            "debug_grasp skipped: requires VisionService and the real "
            "manipulator SDK")
        set_target_properties(debug_grasp PROPERTIES EXCLUDE_FROM_ALL TRUE)
    else()
        target_include_directories(debug_grasp PRIVATE
            ${VISION_INCLUDE_DIR}
            ${MANIP_INCLUDE_DIR}
        )
        target_link_libraries(debug_grasp PRIVATE
            ${VISION_LIBRARY}
            ${MANIP_LIBRARY}
        )
        install(TARGETS debug_grasp DESTINATION bin)
    endif()
else()
    message(STATUS
        "[Tools] debug_grasp skipped: realsense backend not available")
endif()

if(NOT USE_MOCK_EXECUTOR)
    add_executable(debug_ik tools/debug_ik.cpp)
    target_include_directories(debug_ik PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${MANIP_INCLUDE_DIR}
    )
    target_link_libraries(debug_ik PRIVATE
        yaml-cpp
        ${MANIP_LIBRARY}
    )
    install(TARGETS debug_ik DESTINATION bin)

    add_executable(debug_execute_safe
        tools/debug_execute_safe.cpp
        src/grasp_executor.cpp
        src/grasp_executor_gripper.cpp
        src/grasp_executor_runtime.cpp
        src/arm_path_safety.cpp
        src/gripper_holding.cpp
        src/motion_completion.cpp
    )
    target_include_directories(debug_execute_safe PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${MANIP_INCLUDE_DIR}
    )
    target_link_libraries(debug_execute_safe PRIVATE
        yaml-cpp
        ${OpenCV_LIBS}
        ${MANIP_LIBRARY}
        ${GRASP_LIBRARY}
        PkgConfig::URDFDOM
        pthread
    )
    install(TARGETS debug_execute_safe DESTINATION bin)
endif()

# ============ 关节读取工具 ============

if(NOT USE_MOCK_EXECUTOR)
    # read_joints 需要真实的 manipulator SDK。
    add_executable(read_joints tools/read_joints.cpp)
    target_include_directories(read_joints PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${MANIP_INCLUDE_DIR}
    )
    target_link_libraries(read_joints PRIVATE
        ${MANIP_LIBRARY}
        pthread
    )
    install(TARGETS read_joints DESTINATION bin)
endif()
