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

# X86_STANDALONE 是主机侧规划和调试构建的快捷选项。
if(X86_STANDALONE)
    set(USE_OPENCV_DNN ON)
    set(USE_MOCK_EXECUTOR ON)
endif()

# ============ 通用依赖 ============

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui imgcodecs dnn calib3d)
find_package(yaml-cpp REQUIRED)

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
    src/grasp_planner.cpp
    src/grasp_pipeline.cpp
    src/stereo_camera.cpp
    src/stereo_geometry.cpp
    src/mobile_base_controller.cpp
    src/orientation_estimator.cpp
    src/voice_command_parser.cpp
)

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
        # Patch a private build copy so its OpenCV/ORT dependencies resolve on
        # the deployed K3 without modifying the original runtime package.
        set(_LAS2_LINK_LIBRARY "${LAS2_LIBRARY}")
        find_program(PATCHELF_EXECUTABLE patchelf)
        if(PATCHELF_EXECUTABLE)
            set(_LAS2_PATCH_DIR "${CMAKE_BINARY_DIR}/las2_runtime")
            set(_LAS2_PATCHED_LIBRARY
                "${_LAS2_PATCH_DIR}/liblas2_usb_stereo.so")
            string(CONCAT _LAS2_PRIVATE_RPATH
                "${_LAS2_LIB_DIR}:${_LAS2_ROOT_DIR}/lib/spacemit_ort:"
                "${_LAS2_ROOT_DIR}/lib/opencv")
            add_custom_command(
                OUTPUT "${_LAS2_PATCHED_LIBRARY}"
                COMMAND "${CMAKE_COMMAND}" -E make_directory
                    "${_LAS2_PATCH_DIR}"
                COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                    "${LAS2_LIBRARY}" "${_LAS2_PATCHED_LIBRARY}"
                COMMAND "${PATCHELF_EXECUTABLE}" --set-rpath
                    "${_LAS2_PRIVATE_RPATH}" "${_LAS2_PATCHED_LIBRARY}"
                DEPENDS "${LAS2_LIBRARY}"
                COMMENT "Preparing private spacemit_las2 runtime"
                VERBATIM
            )
            add_custom_target(spacemit_las2_runtime
                DEPENDS "${_LAS2_PATCHED_LIBRARY}")
            set(_LAS2_RUNTIME_TARGET spacemit_las2_runtime)
            set(_LAS2_LINK_LIBRARY "${_LAS2_PATCHED_LIBRARY}")
            list(APPEND EXTRA_BUILD_RPATH "${_LAS2_PATCH_DIR}")
            message(STATUS
                "[Camera] LAS2 private runtime will be prepared during build")
        else()
            message(WARNING
                "[Camera] patchelf not found; LD_LIBRARY_PATH is required "
                "for LAS2 private dependencies")
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

if(NOT HAVE_REALSENSE_CAMERA AND NOT HAVE_LAS2_CAMERA)
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

    list(APPEND SOURCES src/grasp_executor.cpp)
    list(APPEND EXTRA_INCLUDE_DIRS ${MANIP_INCLUDE_DIR})
    list(APPEND EXTRA_LIBRARIES ${MANIP_LIBRARY} ${GRASP_LIBRARY})
endif()

# ============ 构建摘要 ============

message(STATUS "")
message(STATUS "=== perceptive_grasp Build Summary ===")
if(USE_OPENCV_DNN)
    message(STATUS "  Detector:  OpenCV DNN")
else()
    message(STATUS "  Detector:  VisionService")
endif()
if(USE_MOCK_EXECUTOR)
    message(STATUS "  Executor:  Mock (print only)")
else()
    message(STATUS "  Executor:  Real hardware")
endif()
set(_CAMERA_BACKENDS)
if(HAVE_REALSENSE_CAMERA)
    list(APPEND _CAMERA_BACKENDS "realsense")
endif()
if(HAVE_LAS2_CAMERA)
    list(APPEND _CAMERA_BACKENDS "spacemit_las2")
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
    DEPENDS
        perceptive_grasp_core
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_perceptive_grasp.py"
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


# ============ 安装 ============

install(TARGETS perceptive_grasp_core DESTINATION bin)
install(PROGRAMS scripts/run_perceptive_grasp.py
    DESTINATION bin
    RENAME perceptive_grasp)
install(DIRECTORY config/ DESTINATION share/perceptive_grasp/config)
install(DIRECTORY docs/ DESTINATION share/perceptive_grasp/docs)
install(DIRECTORY urdf/ DESTINATION share/perceptive_grasp/urdf)
install(FILES resources/eye-to-hand.jpg
    DESTINATION share/perceptive_grasp/resources)
if(HAVE_LAS2_CAMERA)
    install(FILES "${_LAS2_LINK_LIBRARY}"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/perceptive_grasp")
endif()
install(PROGRAMS
    scripts/asr_node.py
    scripts/check_runtime_env.py
    scripts/local_voice_bridge.py
    scripts/tts_node.py
    DESTINATION share/perceptive_grasp/scripts)

# ============ 立体相机调试工具 ============

if(HAVE_REALSENSE_CAMERA OR HAVE_LAS2_CAMERA)
    set(DEBUG_VIEW_SOURCES
        tools/debug_view.cpp
        src/stereo_camera.cpp
        src/stereo_geometry.cpp
    )
    if(HAVE_REALSENSE_CAMERA)
        list(APPEND DEBUG_VIEW_SOURCES src/depth_camera.cpp)
    endif()
    if(HAVE_LAS2_CAMERA)
        list(APPEND DEBUG_VIEW_SOURCES src/las2_stereo_camera.cpp)
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

if(HAVE_REALSENSE_CAMERA OR HAVE_LAS2_CAMERA)
    set(DEBUG_LOCALIZE_SOURCES
        tools/debug_localize.cpp
        src/grasp_planner.cpp
        src/stereo_camera.cpp
        src/stereo_geometry.cpp
    )
    if(HAVE_REALSENSE_CAMERA)
        list(APPEND DEBUG_LOCALIZE_SOURCES src/depth_camera.cpp)
    endif()
    if(HAVE_LAS2_CAMERA)
        list(APPEND DEBUG_LOCALIZE_SOURCES src/las2_stereo_camera.cpp)
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

    if(USE_OPENCV_DNN)
        message(WARNING
            "debug_localize skipped: requires VisionService "
            "(USE_OPENCV_DNN=OFF)")
        set_target_properties(debug_localize PROPERTIES EXCLUDE_FROM_ALL TRUE)
    else()
        target_include_directories(debug_localize PRIVATE
            ${VISION_INCLUDE_DIR})
        target_link_libraries(debug_localize PRIVATE ${VISION_LIBRARY})
    endif()

    install(TARGETS debug_localize DESTINATION bin)
else()
    message(STATUS
        "[Tools] debug_localize skipped: no camera backend available")
endif()

if(HAVE_REALSENSE_CAMERA)
    add_executable(debug_grasp
        tools/debug_grasp.cpp
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

    if(USE_OPENCV_DNN)
        message(WARNING
            "debug_grasp skipped: requires VisionService "
            "(USE_OPENCV_DNN=OFF)")
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
    endif()

    install(TARGETS debug_grasp DESTINATION bin)
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

    add_executable(debug_execute_safe tools/debug_execute_safe.cpp src/grasp_executor.cpp)
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
