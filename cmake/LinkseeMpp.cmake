# LinkseeMpp.cmake - prebuilt MPP from top-level output/staging (mm components/multimedia/mpp).
#
# Required before include (from rtsp_detection / rtsp_tracking CMakeLists.txt):
#   get_filename_component(LINKSEE_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../" ABSOLUTE)
#   include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/LinkseeMpp.cmake")
#
# Do not include() this file by absolute path without setting LINKSEE_REPO_ROOT; use the
# relative include from the application directory as above.
#
# Optional: cmake -DMPP_STAGING_DIR=/path/to/staging ..
#
# Outputs:
#   MPP_LIBRARY, MPP_INCLUDE_DIRS, target mpp
#   MPP_VCODEC_PLUGIN - NOT linked; POST_BUILD copy only (runtime: MPP_V4L2_LINLON_PLUGIN)
#   MPP_VCODEC_PLUGIN_AVAILABLE - TRUE if plugin file exists at configure time

if(DEFINED LINKSEE_MPP_SETUP)
    return()
endif()
set(LINKSEE_MPP_SETUP TRUE)

if(NOT LINKSEE_REPO_ROOT OR LINKSEE_REPO_ROOT STREQUAL "")
    message(FATAL_ERROR
        "LINKSEE_REPO_ROOT is not set. Before including LinkseeMpp.cmake:\n"
        "  get_filename_component(LINKSEE_REPO_ROOT \"\${CMAKE_CURRENT_SOURCE_DIR}/../../../../\" ABSOLUTE)")
endif()
get_filename_component(LINKSEE_REPO_ROOT "${LINKSEE_REPO_ROOT}" ABSOLUTE)
if(NOT EXISTS "${LINKSEE_REPO_ROOT}/build/envsetup.sh")
    message(FATAL_ERROR
        "Invalid LINKSEE_REPO_ROOT: ${LINKSEE_REPO_ROOT}\n"
        "Expected build/envsetup.sh under the spacemit_robot repository root.")
endif()

if(NOT DEFINED MPP_STAGING_DIR OR MPP_STAGING_DIR STREQUAL "")
    set(MPP_STAGING_DIR "${LINKSEE_REPO_ROOT}/output/staging")
endif()
set(MPP_STAGING_DIR "${MPP_STAGING_DIR}" CACHE PATH
    "MPP install prefix (libmpp.so and headers from mm components/multimedia/mpp)")

set(MPP_LIBRARY "${MPP_STAGING_DIR}/lib/libmpp.so")
set(MPP_VCODEC_PLUGIN "${MPP_STAGING_DIR}/lib/libv4l2_linlonv5v7_codec2.so")

if(EXISTS "${MPP_VCODEC_PLUGIN}")
    set(MPP_VCODEC_PLUGIN_AVAILABLE TRUE)
else()
    set(MPP_VCODEC_PLUGIN_AVAILABLE FALSE)
    message(WARNING
        "MPP codec plugin not found (POST_BUILD copy will be skipped):\n"
        "  ${MPP_VCODEC_PLUGIN}\n"
        "Build MPP: mm components/multimedia/mpp\n"
        "At runtime set MPP_V4L2_LINLON_PLUGIN to the plugin path, or install to /usr/lib.")
endif()

if(NOT EXISTS "${MPP_LIBRARY}")
    message(FATAL_ERROR
        "libmpp.so not found: ${MPP_LIBRARY}\n"
        "Build MPP first: source build/envsetup.sh && mm components/multimedia/mpp")
endif()

if(NOT TARGET mpp)
    add_library(mpp SHARED IMPORTED GLOBAL)
    set_target_properties(mpp PROPERTIES IMPORTED_LOCATION "${MPP_LIBRARY}")
endif()

set(MPP_INCLUDE_DIRS
    ${MPP_STAGING_DIR}/include
    ${MPP_STAGING_DIR}/include/sys
    ${MPP_STAGING_DIR}/include/uvc
    ${MPP_STAGING_DIR}/include/vdec
    ${MPP_STAGING_DIR}/include/venc
    ${MPP_STAGING_DIR}/include/mux
    ${MPP_STAGING_DIR}/include/vi
    ${MPP_STAGING_DIR}/include/vo
    ${MPP_STAGING_DIR}/include/demux
    ${MPP_STAGING_DIR}/include/v2d
)
