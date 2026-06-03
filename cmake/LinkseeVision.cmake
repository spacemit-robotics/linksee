# LinkseeVision.cmake - VisionService (prefer output/staging, fallback app third_party/vision).
#
# Required before include:
#   get_filename_component(LINKSEE_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../" ABSOLUTE)
#   set(LINKSEE_APP_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
#   include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/LinkseeVision.cmake")
#
# Do not include() by absolute path without LINKSEE_REPO_ROOT / LINKSEE_APP_SOURCE_DIR.
#
# Optional: cmake -DVISION_INCLUDE_DIR=... -DVISION_LIBRARY=...

if(DEFINED LINKSEE_VISION_SETUP)
    return()
endif()
set(LINKSEE_VISION_SETUP TRUE)

if(NOT LINKSEE_REPO_ROOT OR LINKSEE_REPO_ROOT STREQUAL "")
    message(FATAL_ERROR
        "LINKSEE_REPO_ROOT is not set. Before including LinkseeVision.cmake:\n"
        "  get_filename_component(LINKSEE_REPO_ROOT \"\${CMAKE_CURRENT_SOURCE_DIR}/../../../../\" ABSOLUTE)")
endif()
get_filename_component(LINKSEE_REPO_ROOT "${LINKSEE_REPO_ROOT}" ABSOLUTE)
if(NOT EXISTS "${LINKSEE_REPO_ROOT}/build/envsetup.sh")
    message(FATAL_ERROR
        "Invalid LINKSEE_REPO_ROOT: ${LINKSEE_REPO_ROOT}\n"
        "Expected build/envsetup.sh under the spacemit_robot repository root.")
endif()

if(NOT LINKSEE_APP_SOURCE_DIR)
    message(FATAL_ERROR
        "LINKSEE_APP_SOURCE_DIR is not set. Before including LinkseeVision.cmake, set:\n"
        "  set(LINKSEE_APP_SOURCE_DIR \"\${CMAKE_CURRENT_SOURCE_DIR}\")")
endif()

set(_VISION_STAGING_INCLUDE "${LINKSEE_REPO_ROOT}/output/staging/include")
set(_VISION_STAGING_LIBRARY "${LINKSEE_REPO_ROOT}/output/staging/lib/libvision.so")
set(_VISION_VENDOR_INCLUDE "${LINKSEE_APP_SOURCE_DIR}/third_party/vision/include")
set(_VISION_VENDOR_LIBRARY "${LINKSEE_APP_SOURCE_DIR}/third_party/vision/lib/libvision.so")

if(VISION_LIBRARY AND VISION_INCLUDE_DIR)
    if(NOT EXISTS "${VISION_LIBRARY}")
        message(FATAL_ERROR "VISION_LIBRARY does not exist: ${VISION_LIBRARY}")
    endif()
    if(NOT EXISTS "${VISION_INCLUDE_DIR}/vision_service.h")
        message(FATAL_ERROR "vision_service.h not found in VISION_INCLUDE_DIR=${VISION_INCLUDE_DIR}")
    endif()
    set(VISION_INCLUDE_DIR "${VISION_INCLUDE_DIR}" CACHE PATH "Directory containing vision_service.h" FORCE)
    set(VISION_LIBRARY "${VISION_LIBRARY}" CACHE FILEPATH "Path to libvision.so" FORCE)
    message(STATUS "Vision: using user-specified ${VISION_LIBRARY}")
elseif(EXISTS "${_VISION_STAGING_LIBRARY}" AND EXISTS "${_VISION_STAGING_INCLUDE}/vision_service.h")
    set(VISION_INCLUDE_DIR "${_VISION_STAGING_INCLUDE}" CACHE PATH "Directory containing vision_service.h" FORCE)
    set(VISION_LIBRARY "${_VISION_STAGING_LIBRARY}" CACHE FILEPATH "Path to libvision.so" FORCE)
    message(STATUS "Vision: using ${_VISION_STAGING_LIBRARY}")
elseif(EXISTS "${_VISION_VENDOR_LIBRARY}" AND EXISTS "${_VISION_VENDOR_INCLUDE}/vision_service.h")
    set(VISION_INCLUDE_DIR "${_VISION_VENDOR_INCLUDE}" CACHE PATH "Directory containing vision_service.h" FORCE)
    set(VISION_LIBRARY "${_VISION_VENDOR_LIBRARY}" CACHE FILEPATH "Path to libvision.so" FORCE)
    message(STATUS "Vision: using vendored ${_VISION_VENDOR_LIBRARY} (under ${LINKSEE_APP_SOURCE_DIR})")
else()
    message(FATAL_ERROR
        "libvision.so not found.\n"
        "  staging: ${_VISION_STAGING_LIBRARY}\n"
        "  vendor:  ${_VISION_VENDOR_LIBRARY}\n"
        "Build vision first: source build/envsetup.sh && mm components/model_zoo/vision\n"
        "Or copy libvision.so to ${_VISION_VENDOR_LIBRARY}")
endif()
