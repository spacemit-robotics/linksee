cmake_minimum_required(VERSION 3.16)
project(rtsp_detection_standalone LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../../components/multimedia/mpp ${CMAKE_BINARY_DIR}/mpp)

# cpp-httplib: resolve header (invalid/empty downloads still "exist" and break compilation).
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

# Detect timeout messages from CMake/curl/wget so we skip further attempts and ask for manual download.
function(_rtsp_is_timeout_msg msg out_var)
    string(TOLOWER "${msg}" _m)
    if(_m MATCHES "timeout" OR _m MATCHES "timed out" OR _m MATCHES "connection timed out"
            OR _m MATCHES "operation timed out")
        set(${out_var} 1 PARENT_SCOPE)
    else()
        set(${out_var} 0 PARENT_SCOPE)
    endif()
endfunction()

# CMake's file(DOWNLOAD) uses its own HTTP stack (often hits timeouts before wget/curl).
function(_rtsp_fetch_httplib_to_dest url dest timeout_sec)
    file(DOWNLOAD "${url}" "${dest}"
        TIMEOUT "${timeout_sec}"
        TLS_VERIFY ON
        STATUS _dl_st)
    list(GET _dl_st 0 _dl_code)
    if(NOT _dl_code EQUAL 0)
        list(GET _dl_st 1 _dl_msg)
        _rtsp_is_timeout_msg("${_dl_msg}" _dl_timeout)
        if(_dl_timeout)
            message(WARNING
                "httplib.h download timed out (CMake file(DOWNLOAD)). Skipping curl/wget.\n"
                "Download manually and place at:\n"
                "  ${dest}\n"
                "or:\n"
                "  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cpp_httplib/httplib.h\n"
                "URL:\n  ${url}\n"
                "Or set -DHTTPLIB_LOCAL_PATH=/path/to/httplib.h / -DHTTPLIB_SKIP_DOWNLOAD=ON")
            return()
        endif()
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
            _rtsp_is_timeout_msg("${_curl_err}" _curl_timeout)
            if(_curl_timeout OR _curl_rc EQUAL 28)
                message(WARNING
                    "httplib.h download timed out (curl). Skipping wget.\n"
                    "Download manually and place at:\n"
                    "  ${dest}\n"
                    "or:\n"
                    "  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cpp_httplib/httplib.h\n"
                    "URL:\n  ${url}\n"
                    "Or: -DHTTPLIB_LOCAL_PATH=/path/to/httplib.h / -DHTTPLIB_SKIP_DOWNLOAD=ON")
                return()
            endif()
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
            _rtsp_is_timeout_msg("${_wget_err}" _wget_timeout)
            if(_wget_timeout)
                message(WARNING
                    "httplib.h download timed out (wget).\n"
                    "Download manually and place at:\n"
                    "  ${dest}\n"
                    "or:\n"
                    "  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cpp_httplib/httplib.h\n"
                    "URL:\n  ${url}\n"
                    "Or: -DHTTPLIB_LOCAL_PATH=/path/to/httplib.h / -DHTTPLIB_SKIP_DOWNLOAD=ON")
                return()
            endif()
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

# Priority: HTTPLIB_LOCAL_PATH -> HTTP download -> third_party/cpp_httplib/httplib.h fallback.
set(HTTPLIB_LOCAL_PATH "" CACHE FILEPATH "Optional local path to httplib.h")
option(HTTPLIB_SKIP_DOWNLOAD "Do not fetch httplib.h over the network; use vendor path or HTTPLIB_LOCAL_PATH")
set(HTTPLIB_DOWNLOAD_TIMEOUT_SECONDS 120 CACHE STRING
    "Timeout for httplib.h download (CMake first; on timeout further tools are skipped). Use HTTPLIB_LOCAL_PATH if unable to download.")
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
        if(HTTPLIB_SKIP_DOWNLOAD)
            message(STATUS
                "HTTPLIB_SKIP_DOWNLOAD=ON: not downloading httplib.h. "
                "Place v0.15.3 httplib.h under third_party/cpp_httplib/ or set HTTPLIB_LOCAL_PATH.")
        else()
            _rtsp_fetch_httplib_to_dest("${_httplib_upstream_url}" "${_httplib_dest}"
                "${HTTPLIB_DOWNLOAD_TIMEOUT_SECONDS}")
        endif()
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
                "Download it yourself (e.g. browser or machine with GitHub access) from:\n"
                "  ${_httplib_upstream_url}\n"
                "Save to either:\n"
                "  ${_httplib_vendor}\n"
                "  ${_httplib_dest}\n"
                "Then re-run CMake, or pass:\n"
                "  -DHTTPLIB_LOCAL_PATH=/absolute/path/to/httplib.h\n"
                "Options: -DHTTPLIB_SKIP_DOWNLOAD=ON (no network), "
                "-DHTTPLIB_DOWNLOAD_TIMEOUT_SECONDS=300 (slower links)")
        endif()
    endif()
endif()

_rtsp_assert_valid_httplib("${_httplib_hdr}")

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
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../components/multimedia/mpp/include
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
