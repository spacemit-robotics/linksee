# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
# Linksee dependency profile for the system OpenCV ABI on Bianbu K3.
# Build the existing vision sources from components/model_zoo/vision with:
# mm -- -C "$SROBOTIS_ROOT/application/ros2/linksee/perceptive_grasp/cmake/vision-k3-system-cpp.cmake"

set(_vision_system_opencv "/usr/lib/riscv64-linux-gnu/cmake/opencv4")
set(_vision_system_blas "/usr/lib/riscv64-linux-gnu/libopenblas.so.0")
set(_vision_blas_headers "/opt/openblas-spacemit/include")

foreach(_dependency IN ITEMS
        "${_vision_system_opencv}/OpenCVConfig.cmake"
        "/usr/include/eigen3/Eigen/Dense"
        "${_vision_system_blas}"
        "${_vision_blas_headers}/cblas.h"
        "${_vision_blas_headers}/openblas_config.h")
    if(NOT EXISTS "${_dependency}")
        message(FATAL_ERROR
            "k3-system-cpp dependency not found: ${_dependency}. "
            "This profile requires Bianbu K3 system libopencv-dev, "
            "libeigen3-dev and libopenblas0-pthread, plus matching "
            "openblas-spacemit development headers.")
    endif()
endforeach()

# Bianbu's system runtime and SpaceMIT development package provide the same
# LP64 OpenBLAS API. Link the system runtime used by OpenCV; never the /opt library.
file(READ "${_vision_blas_headers}/openblas_config.h" _vision_blas_config)
string(REGEX MATCH "OPENBLAS_VERSION[ \t]+\"[ \t]*OpenBLAS[ \t]+([0-9]+\\.[0-9]+\\.[0-9]+)"
    _vision_header_version_match "${_vision_blas_config}")
set(_vision_header_version "${CMAKE_MATCH_1}")
get_filename_component(_vision_blas_real "${_vision_system_blas}" REALPATH)
string(REGEX MATCH "-r([0-9]+\\.[0-9]+\\.[0-9]+)\\.so$"
    _vision_library_version_match "${_vision_blas_real}")
set(_vision_library_version "${CMAKE_MATCH_1}")
if(NOT _vision_header_version_match OR NOT _vision_library_version_match
        OR NOT _vision_header_version STREQUAL _vision_library_version
        OR _vision_blas_config MATCHES "#[ \t]*define[ \t]+OPENBLAS_USE64BITINT([ \t\r\n]|$)")
    message(FATAL_ERROR
        "k3-system-cpp requires matching LP64 OpenBLAS headers/runtime: "
        "headers=${_vision_header_version}, runtime=${_vision_library_version}. "
        "Use matching Bianbu system OpenBLAS and openblas-spacemit packages.")
endif()

# FORCE replaces an earlier vendor or Python configuration in the same build
# directory; the profile is opt-in and does not alter ordinary CMake defaults.
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
set(OpenCV_DIR "${_vision_system_opencv}" CACHE PATH "System OpenCV package" FORCE)
set(OpenCV_INSTALL_DIR "/usr" CACHE PATH "System OpenCV prefix" FORCE)
set(EIGEN_SPACEMIT_DIR "/usr" CACHE PATH "System Eigen prefix" FORCE)
set(OPENBLAS_DIR "/usr" CACHE PATH "System OpenBLAS prefix" FORCE)
set(OPENBLAS_LIB "${_vision_system_blas}" CACHE FILEPATH "System LP64 OpenBLAS runtime" FORCE)
set(OPENBLAS_INCLUDE_DIR "${_vision_blas_headers}" CACHE PATH "Matching LP64 OpenBLAS headers" FORCE)
set(SPACEMIT_DIR "/usr" CACHE PATH "SpaceMIT inference runtime prefix" FORCE)
set(BUILD_TESTS ON CACHE BOOL "Build tests" FORCE)
foreach(_option IN ITEMS BUILD_EXAMPLES BUILD_PYTHON_BINDINGS BUILD_PYTHON_WHEEL
        VISION_WITH_MPP VISION_WITH_OPENCL VISION_WITH_V2D)
    set(${_option} OFF CACHE BOOL "Disabled by the k3-system-cpp profile" FORCE)
endforeach()

message(STATUS "Vision profile: k3-system-cpp (system OpenCV, LP64 OpenBLAS ${_vision_library_version})")
