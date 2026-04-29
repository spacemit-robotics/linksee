cmake_minimum_required(VERSION 3.16)
project(linksee_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
set(STAGING_LIB_DIR ${REPO_ROOT}/output/staging/lib)

find_package(Components QUIET)

if(TARGET components::mlink_device)
    set(MLINK_DEVICE_TARGET components::mlink_device)
else()
    set(MLINK_INCLUDE_DIR
        ${REPO_ROOT}/components/agent_tools/mlink/device/include
    )
    set(MLINK_LIBRARY
        ${REPO_ROOT}/output/build/cmake/pkgs/components_agent_tools_mlink_device/libmlink_device.so
    )

    if(NOT EXISTS ${MLINK_INCLUDE_DIR}/mlink.h)
        message(FATAL_ERROR "mlink.h not found at ${MLINK_INCLUDE_DIR}")
    endif()

    if(NOT EXISTS ${MLINK_LIBRARY})
        message(FATAL_ERROR "libmlink_device not found at ${MLINK_LIBRARY}")
    endif()
endif()

add_library(linksee_runner STATIC
    src/cpp/runner.cpp
)

target_include_directories(linksee_runner PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(linksee_native
    src/cpp/main.cpp
)

target_link_libraries(linksee_native PRIVATE linksee_runner)

add_executable(linksee_device
    src/cpp/linksee_device.cpp
)

set_target_properties(linksee_device PROPERTIES
    BUILD_RPATH ${STAGING_LIB_DIR}
    INSTALL_RPATH ${STAGING_LIB_DIR}
)

if(TARGET components::mlink_device)
    target_link_libraries(linksee_device PRIVATE
        linksee_runner
        ${MLINK_DEVICE_TARGET}
    )
else()
    target_include_directories(linksee_device PRIVATE
        ${MLINK_INCLUDE_DIR}
    )

    target_link_libraries(linksee_device PRIVATE
        linksee_runner
        ${MLINK_LIBRARY}
    )
endif()

install(TARGETS linksee_native linksee_device linksee_runner
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS
    scripts/setup_env.sh
    scripts/common.py
    scripts/evaluate.py
    scripts/run_host.py
    scripts/start_host.sh
    scripts/start_inference.sh
    scripts/stop_host.sh
    scripts/stop_inference.sh
    DESTINATION share/linksee/scripts
)

install(DIRECTORY models/ DESTINATION share/linksee/models)
install(DIRECTORY DESTINATION share/linksee/datasets)
install(FILES
    LICENSE
    NOTICE
    README.md
    package.xml
    pyproject.toml
    DESTINATION share/linksee/linksee_app
)
