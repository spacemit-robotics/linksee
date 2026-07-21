# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0

if(DEFINED _PERCEPTIVE_GRASP_WEBRTC_AP_LOADED)
    return()
endif()
set(_PERCEPTIVE_GRASP_WEBRTC_AP_LOADED ON)

include("${CMAKE_CURRENT_LIST_DIR}/fetch_third_party.cmake")

fetch_third_party(
    NAME webrtc-audio-processing
    GIT_REPOSITORY https://gitee.com/spacemit-robotics/webrtc-audio-processing.git
    GIT_COMMIT d0569cfa50c1858ee279d77b3fc8870be6902441
    OUT_SOURCE_DIR WEBRTC_SOURCE_DIR
)

if(DEFINED ENV{SROBOTIS_THIRDPARTY_CACHE})
    set(WEBRTC_CACHE_DIR "$ENV{SROBOTIS_THIRDPARTY_CACHE}")
elseif(DEFINED ENV{HOME})
    set(WEBRTC_CACHE_DIR "$ENV{HOME}/.cache/thirdparty")
else()
    set(WEBRTC_CACHE_DIR "${CMAKE_BINARY_DIR}/.cache/thirdparty")
endif()
set(WEBRTC_BINARY_DIR "${WEBRTC_CACHE_DIR}/webrtc-audio-processing/build")

include(ExternalProject)
ExternalProject_Add(webrtc_audio_processing_project
    SOURCE_DIR "${WEBRTC_SOURCE_DIR}"
    BINARY_DIR "${WEBRTC_BINARY_DIR}"
    CONFIGURE_COMMAND "${MESON_EXECUTABLE}" setup <BINARY_DIR> <SOURCE_DIR>
        --buildtype=release
    BUILD_COMMAND "${NINJA_EXECUTABLE}" -C <BINARY_DIR>
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS
        "${WEBRTC_BINARY_DIR}/webrtc/modules/audio_processing/libwebrtc-audio-processing-2.so"
        "${WEBRTC_BINARY_DIR}/webrtc/modules/audio_processing/libwebrtc-audio-processing-2.so.1"
)

set(WEBRTC_LIBRARY
    "${WEBRTC_BINARY_DIR}/webrtc/modules/audio_processing/libwebrtc-audio-processing-2.so")
set(WEBRTC_RUNTIME_LIBRARY
    "${WEBRTC_BINARY_DIR}/webrtc/modules/audio_processing/libwebrtc-audio-processing-2.so.1")
set(ABSEIL_INCLUDE_DIR
    "${WEBRTC_SOURCE_DIR}/subprojects/abseil-cpp-20240722.0")
set(WEBRTC_INCLUDE_DIRS
    "${WEBRTC_SOURCE_DIR}"
    "${WEBRTC_SOURCE_DIR}/webrtc"
    "${WEBRTC_BINARY_DIR}"
    "${WEBRTC_BINARY_DIR}/webrtc"
    "${ABSEIL_INCLUDE_DIR}"
)
