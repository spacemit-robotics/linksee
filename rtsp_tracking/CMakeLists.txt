cmake_minimum_required(VERSION 3.16)
project(rtsp_tracking_standalone LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# External VisionService dependency.
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

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../../components/multimedia/mpp ${CMAKE_BINARY_DIR}/mpp)

function(_rtsp_httplib_header_ok hdr_path out_var)
    if(NOT EXISTS "${hdr_path}")
        set(${out_var} 0 PARENT_SCOPE)
        return()
    endif()
    file(SIZE "${hdr_path}" _hv_sz)
    if(_hv_sz LESS 8000)
        set(${out_var} 0 PARENT_SCOPE)
        return()
    endif()
    file(READ "${hdr_path}" _hv_snip LIMIT 16384)
    string(FIND "${_hv_snip}" "namespace httplib" _hv_ns)
    if(_hv_ns EQUAL -1)
        set(${out_var} 0 PARENT_SCOPE)
        return()
    endif()
    set(${out_var} 1 PARENT_SCOPE)
endfunction()

function(_rtsp_fetch_httplib_to_dest url dest timeout_sec)
    file(DOWNLOAD "${url}" "${dest}"
        TIMEOUT "${timeout_sec}"
        TLS_VERIFY ON
        STATUS _dl_st)
    list(GET _dl_st 0 _dl_code)
    if(NOT _dl_code EQUAL 0)
        list(GET _dl_st 1 _dl_msg)
        message(STATUS "CMake file(DOWNLOAD) failed: ${_dl_msg}; trying curl, then wget...")
    endif()
    _rtsp_httplib_header_ok("${dest}" _fetch_ok)
    if(_fetch_ok)
        return()
    endif()
    find_program(_RTSP_CURL NAMES curl)
    if(_RTSP_CURL)
        execute_process(
            COMMAND "${_RTSP_CURL}" -fsSL
                --connect-timeout "${timeout_sec}"
                --max-time "${timeout_sec}"
                -o "${dest}" "${url}"
            RESULT_VARIABLE _curl_rc
            ERROR_VARIABLE _curl_err
            OUTPUT_QUIET)
        if(_curl_rc EQUAL 0)
            _rtsp_httplib_header_ok("${dest}" _ok_curl)
            if(_ok_curl)
                message(STATUS "Fetched httplib.h with curl (CMake download incomplete or timed out).")
                return()
            endif()
        else()
            message(STATUS "curl failed (${_curl_rc}): ${_curl_err}")
        endif()
    endif()
    find_program(_RTSP_WGET NAMES wget)
    if(_RTSP_WGET)
        execute_process(
            COMMAND "${_RTSP_WGET}" -q -T "${timeout_sec}" -O "${dest}" "${url}"
            RESULT_VARIABLE _wget_rc
            ERROR_VARIABLE _wget_err
            OUTPUT_QUIET)
        if(_wget_rc EQUAL 0)
            _rtsp_httplib_header_ok("${dest}" _ok_wget)
            if(_ok_wget)
                message(STATUS "Fetched httplib.h with wget (CMake download incomplete or timed out).")
                return()
            endif()
        else()
            message(STATUS "wget failed (${_wget_rc}): ${_wget_err}")
        endif()
    endif()
endfunction()

function(_rtsp_assert_valid_httplib hdr_path)
    if(NOT EXISTS "${hdr_path}")
        message(FATAL_ERROR "httplib.h not found: ${hdr_path}")
    endif()
    file(SIZE "${hdr_path}" _hv_sz)
    if(_hv_sz LESS 8000)
        message(FATAL_ERROR
            "httplib.h is too small (${_hv_sz} bytes) at:\n  ${hdr_path}\n"
            "Likely an empty file or an HTML error page from a failed download — delete it.\n"
            "Install cpp-httplib v0.15.3 single header at:\n"
            "  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cpp_httplib/httplib.h\n"
            "or pass -DHTTPLIB_LOCAL_PATH=/path/to/httplib.h")
    endif()
    file(READ "${hdr_path}" _hv_snip LIMIT 16384)
    string(FIND "${_hv_snip}" "namespace httplib" _hv_ns)
    if(_hv_ns EQUAL -1)
        message(FATAL_ERROR
            "httplib.h does not look like yhirose/cpp-httplib (no namespace httplib) at:\n  ${hdr_path}\n"
            "Replace with v0.15.3 httplib.h or set -DHTTPLIB_LOCAL_PATH=...")
    endif()
endfunction()

set(HTTPLIB_LOCAL_PATH "" CACHE FILEPATH "Optional local path to httplib.h")
set(HTTPLIB_DOWNLOAD_TIMEOUT_SECONDS 120 CACHE STRING
    "Per-attempt timeout (CMake/curl/wget) for httplib.h; increase on slow links or use HTTPLIB_LOCAL_PATH")
set(_httplib_upstream_url "https://raw.githubusercontent.com/yhirose/cpp-httplib/v0.15.3/httplib.h")
set(_httplib_vendor_dir "${CMAKE_CURRENT_SOURCE_DIR}/third_party/cpp_httplib")
set(_httplib_vendor "${_httplib_vendor_dir}/httplib.h")
set(_httplib_dest_dir "${CMAKE_BINARY_DIR}/_deps/cpp_httplib_header-src")
set(_httplib_dest "${_httplib_dest_dir}/httplib.h")

if(HTTPLIB_LOCAL_PATH)
    if(NOT EXISTS "${HTTPLIB_LOCAL_PATH}")
        message(FATAL_ERROR "HTTPLIB_LOCAL_PATH does not exist: ${HTTPLIB_LOCAL_PATH}")
    endif()
    set(_httplib_hdr "${HTTPLIB_LOCAL_PATH}")
    get_filename_component(_httplib_include_dir "${HTTPLIB_LOCAL_PATH}" DIRECTORY)
else()
    file(MAKE_DIRECTORY "${_httplib_dest_dir}")
    _rtsp_httplib_header_ok("${_httplib_dest}" _httplib_dest_ok)
    if(NOT _httplib_dest_ok)
        _rtsp_fetch_httplib_to_dest("${_httplib_upstream_url}" "${_httplib_dest}"
            "${HTTPLIB_DOWNLOAD_TIMEOUT_SECONDS}")
    endif()
    _rtsp_httplib_header_ok("${_httplib_dest}" _httplib_dest_ok)
    if(_httplib_dest_ok)
        set(_httplib_hdr "${_httplib_dest}")
        set(_httplib_include_dir "${_httplib_dest_dir}")
    else()
        _rtsp_httplib_header_ok("${_httplib_vendor}" _httplib_vendor_ok)
        if(_httplib_vendor_ok)
            message(STATUS "Using third_party/cpp_httplib/httplib.h (download produced no valid header).")
            set(_httplib_hdr "${_httplib_vendor}")
            set(_httplib_include_dir "${_httplib_vendor_dir}")
        else()
            message(FATAL_ERROR
                "Could not obtain a valid cpp-httplib v0.15.3 httplib.h.\n"
                "CMake file(DOWNLOAD), curl, and wget were tried (CMake often times out while wget works).\n"
                "Tried HTTP download to:\n  ${_httplib_dest}\n"
                "URL:\n  ${_httplib_upstream_url}\n"
                "Then checked:\n  ${_httplib_vendor}\n"
                "Fix: place the file under third_party, or:\n"
                "  -DHTTPLIB_LOCAL_PATH=/absolute/path/to/httplib.h\n"
                "Or raise timeout: -DHTTPLIB_DOWNLOAD_TIMEOUT_SECONDS=300")
        endif()
    endif()
endif()

_rtsp_assert_valid_httplib("${_httplib_hdr}")

set(OpenCV_DIR "/opt/opencv-spacemit/lib/cmake/opencv4")
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_executable(example_rtsp_tracking
    cpp/main.cpp
    cpp/pipeline.cpp
    cpp/http_server.cpp
)

target_include_directories(example_rtsp_tracking PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp
    ${VISION_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../components/multimedia/mpp/include
    ${_httplib_include_dir}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(example_rtsp_tracking PRIVATE
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
set_target_properties(example_rtsp_tracking PROPERTIES
    BUILD_RPATH "${_vision_lib_dir}"
    INSTALL_RPATH "${_vision_lib_dir}"
)

# Copy the codec plugin next to the executable so the MPP plugin loader
# finds the freshly built version instead of the stale /usr/lib copy.
add_custom_command(TARGET example_rtsp_tracking POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:example_rtsp_tracking>/lib"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:v4l2_linlonv5v7_codec2>"
        "$<TARGET_FILE_DIR:example_rtsp_tracking>/lib/"
    COMMENT "Copying v4l2_linlonv5v7_codec2.so next to executable"
)
