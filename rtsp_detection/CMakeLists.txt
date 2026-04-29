cmake_minimum_required(VERSION 3.16)
project(rtsp_detection_standalone LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(FetchContent)

# External VisionService dependency.
# Prefer local vendored SDK under applications/rtsp_detection/third_party/vision.
set(_VISION_LOCAL_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/vision/include")
set(_VISION_LOCAL_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/third_party/vision/lib/libvision.so")
set(VISION_INCLUDE_DIR "${_VISION_LOCAL_INCLUDE}" CACHE PATH "Directory containing vision_service.h")
set(VISION_LIBRARY "${_VISION_LOCAL_LIBRARY}" CACHE FILEPATH "Path to libvision.so")

if(NOT EXISTS "${VISION_INCLUDE_DIR}/vision_service.h")
    message(FATAL_ERROR "vision_service.h not found in VISION_INCLUDE_DIR=${VISION_INCLUDE_DIR}")
endif()
if(NOT EXISTS "${VISION_LIBRARY}")
    message(FATAL_ERROR
        "VISION_LIBRARY does not exist: ${VISION_LIBRARY}\n"
        "Copy libvision.so to ${_VISION_LOCAL_LIBRARY}, or pass -DVISION_LIBRARY=/path/to/libvision.so")
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../mpp ${CMAKE_BINARY_DIR}/mpp)

# cpp-httplib dependency.
# - Default: download a pinned single-header release at configure time.
# - Optional: override with -DHTTPLIB_LOCAL_PATH=/path/to/httplib.h for offline builds.
set(HTTPLIB_LOCAL_PATH "" CACHE FILEPATH "Optional local path to httplib.h")
set(_httplib_include_dir "${CMAKE_BINARY_DIR}/_deps/cpp-httplib-src")
set(_httplib_header "${_httplib_include_dir}/httplib.h")
if(HTTPLIB_LOCAL_PATH)
    if(NOT EXISTS "${HTTPLIB_LOCAL_PATH}")
        message(FATAL_ERROR "HTTPLIB_LOCAL_PATH does not exist: ${HTTPLIB_LOCAL_PATH}")
    endif()
    get_filename_component(_httplib_include_dir "${HTTPLIB_LOCAL_PATH}" DIRECTORY)
elseif(NOT EXISTS "${_httplib_header}")
    FetchContent_Declare(cpp_httplib_header
        URL https://raw.githubusercontent.com/yhirose/cpp-httplib/v0.15.3/httplib.h
        DOWNLOAD_NO_EXTRACT TRUE
        DOWNLOAD_DIR "${_httplib_include_dir}"
        DOWNLOAD_NAME "httplib.h")
    FetchContent_MakeAvailable(cpp_httplib_header)
endif()

set(OpenCV_DIR "/opt/opencv-spacemit/lib/cmake/opencv4")
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_executable(example_rtsp_detection
    cpp/main.cpp
    cpp/pipeline.cpp
    cpp/http_server.cpp
)

target_include_directories(example_rtsp_detection PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp
    ${VISION_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../mpp/include
    ${_httplib_include_dir}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(example_rtsp_detection PRIVATE
    ${VISION_LIBRARY}
    ${OpenCV_LIBS}
    mpp_uvc
    mpp_vdec
    mpp_venc
    mpp_mux
    mpp_sys
    pthread
)

get_filename_component(_vision_lib_dir "${VISION_LIBRARY}" DIRECTORY)
set_target_properties(example_rtsp_detection PROPERTIES
    BUILD_RPATH "${_vision_lib_dir}"
    INSTALL_RPATH "${_vision_lib_dir}"
)
