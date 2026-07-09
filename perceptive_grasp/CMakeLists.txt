cmake_minimum_required(VERSION 3.16)
project(perceptive_grasp LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# X86_STANDALONE 是主机侧规划和调试构建的快捷选项。
if(X86_STANDALONE)
    set(USE_OPENCV_DNN ON)
    set(USE_MOCK_EXECUTOR ON)
endif()

# ============ 通用依赖 ============

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui imgcodecs dnn)
find_package(yaml-cpp REQUIRED)

set(REALSENSE2_HINTS)
if(EXISTS "/opt/ros/humble/lib/riscv64-linux-gnu/cmake/realsense2")
    list(APPEND REALSENSE2_HINTS
        "/opt/ros/humble"
        "/opt/ros/humble/lib/riscv64-linux-gnu/cmake/realsense2")
endif()
find_package(realsense2 REQUIRED CONFIG PATHS ${REALSENSE2_HINTS})

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
    src/depth_camera.cpp
    src/grasp_planner.cpp
    src/grasp_pipeline.cpp
    src/mobile_base_controller.cpp
    src/orientation_estimator.cpp
    src/voice_command_parser.cpp
)

if(HAVE_ROS2_VOICE)
    list(APPEND SOURCES src/voice_command_listener.cpp)
endif()

set(EXTRA_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

set(EXTRA_LIBRARIES
    realsense2::realsense2
    ${OpenCV_LIBS}
    yaml-cpp
    pthread
)

# ============ 可选底盘控制 ============

if(NOT DEFINED _STAGING_DIR)
    set(_STAGING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../output/staging")
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
message(STATUS "  Camera:    RealSense D435i (always real)")
message(STATUS "")

# ============ 主目标 ============

add_executable(perceptive_grasp ${SOURCES})

target_include_directories(perceptive_grasp PRIVATE ${EXTRA_INCLUDE_DIRS})
target_link_libraries(perceptive_grasp PRIVATE ${EXTRA_LIBRARIES})
if(HAVE_ROS2_VOICE)
    target_compile_definitions(perceptive_grasp PRIVATE ENABLE_ROS2_VOICE=1)
    target_include_directories(perceptive_grasp PRIVATE
        ${rclcpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
    )
    if(TARGET rclcpp::rclcpp)
        target_link_libraries(perceptive_grasp PRIVATE rclcpp::rclcpp)
    else()
        target_link_libraries(perceptive_grasp PRIVATE ${rclcpp_LIBRARIES})
    endif()
    target_link_libraries(perceptive_grasp PRIVATE
        ${std_msgs_TARGETS}
        ${std_msgs_LIBRARIES}
    )
endif()

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


# ============ 安装 ============

install(TARGETS perceptive_grasp DESTINATION bin)
install(DIRECTORY config/ DESTINATION share/perceptive_grasp/config)
install(DIRECTORY docs/ DESTINATION share/perceptive_grasp/docs)
install(PROGRAMS
    scripts/asr_node.py
    scripts/check_runtime_env.py
    scripts/local_voice_bridge.py
    scripts/tts_node.py
    DESTINATION share/perceptive_grasp/scripts)

# ============ 调试工具 ============

add_executable(debug_view tools/debug_view.cpp)
target_include_directories(debug_view PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(debug_view PRIVATE
    realsense2::realsense2
    ${OpenCV_LIBS}
    pthread
)

if(USE_OPENCV_DNN)
    target_compile_definitions(debug_view PRIVATE NO_VISION_SERVICE=1)
else()
    target_include_directories(debug_view PRIVATE ${VISION_INCLUDE_DIR})
    target_link_libraries(debug_view PRIVATE ${VISION_LIBRARY})
endif()

install(TARGETS debug_view DESTINATION bin)

add_executable(debug_localize
    tools/debug_localize.cpp
    src/grasp_planner.cpp
)
target_include_directories(debug_localize PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(debug_localize PRIVATE
    realsense2::realsense2
    ${OpenCV_LIBS}
    yaml-cpp
    pthread
)

if(USE_OPENCV_DNN)
    message(WARNING "debug_localize skipped: requires VisionService (USE_OPENCV_DNN=OFF)")
    set_target_properties(debug_localize PROPERTIES EXCLUDE_FROM_ALL TRUE)
else()
    target_include_directories(debug_localize PRIVATE ${VISION_INCLUDE_DIR})
    target_link_libraries(debug_localize PRIVATE ${VISION_LIBRARY})
endif()

install(TARGETS debug_localize DESTINATION bin)

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
    message(WARNING "debug_grasp skipped: requires VisionService (USE_OPENCV_DNN=OFF)")
    set_target_properties(debug_grasp PROPERTIES EXCLUDE_FROM_ALL TRUE)
else()
    target_include_directories(debug_grasp PRIVATE ${VISION_INCLUDE_DIR} ${MANIP_INCLUDE_DIR})
    target_link_libraries(debug_grasp PRIVATE ${VISION_LIBRARY} ${MANIP_LIBRARY})
endif()

install(TARGETS debug_grasp DESTINATION bin)

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
