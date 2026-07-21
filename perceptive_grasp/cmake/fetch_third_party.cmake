# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0

if(DEFINED _PERCEPTIVE_GRASP_FETCH_THIRD_PARTY_LOADED)
    return()
endif()
set(_PERCEPTIVE_GRASP_FETCH_THIRD_PARTY_LOADED ON)

if(DEFINED ENV{SROBOTIS_THIRDPARTY_CACHE})
    set(_THIRD_PARTY_CACHE_ROOT "$ENV{SROBOTIS_THIRDPARTY_CACHE}")
elseif(DEFINED ENV{HOME})
    set(_THIRD_PARTY_CACHE_ROOT "$ENV{HOME}/.cache/thirdparty")
else()
    set(_THIRD_PARTY_CACHE_ROOT "${CMAKE_BINARY_DIR}/.cache/thirdparty")
endif()

function(fetch_third_party)
    cmake_parse_arguments(
        ARG
        ""
        "NAME;GIT_REPOSITORY;GIT_REF;GIT_COMMIT;OUT_SOURCE_DIR"
        ""
        ${ARGN}
    )
    if(NOT ARG_NAME OR NOT ARG_GIT_REPOSITORY OR
       (NOT ARG_GIT_REF AND NOT ARG_GIT_COMMIT))
        message(FATAL_ERROR
            "fetch_third_party requires NAME, GIT_REPOSITORY and either "
            "GIT_REF or GIT_COMMIT")
    endif()

    set(source_dir "${_THIRD_PARTY_CACHE_ROOT}/${ARG_NAME}")
    if(NOT EXISTS "${source_dir}/.git")
        file(MAKE_DIRECTORY "${_THIRD_PARTY_CACHE_ROOT}")
        if(ARG_GIT_COMMIT)
            execute_process(
                COMMAND git clone --filter=blob:none --no-checkout
                    "${ARG_GIT_REPOSITORY}" "${source_dir}"
                RESULT_VARIABLE clone_result
            )
        else()
            execute_process(
                COMMAND git clone --depth 1 --branch "${ARG_GIT_REF}"
                    "${ARG_GIT_REPOSITORY}" "${source_dir}"
                RESULT_VARIABLE clone_result
            )
        endif()
        if(NOT clone_result EQUAL 0)
            message(FATAL_ERROR "Failed to fetch ${ARG_NAME}")
        endif()
    endif()

    if(ARG_GIT_COMMIT)
        execute_process(
            COMMAND git cat-file -e "${ARG_GIT_COMMIT}^{commit}"
            WORKING_DIRECTORY "${source_dir}"
            RESULT_VARIABLE commit_exists
            OUTPUT_QUIET
            ERROR_QUIET
        )
        if(NOT commit_exists EQUAL 0)
            execute_process(
                COMMAND git fetch --depth 1 origin "${ARG_GIT_COMMIT}"
                WORKING_DIRECTORY "${source_dir}"
                RESULT_VARIABLE fetch_result
            )
            if(NOT fetch_result EQUAL 0)
                message(FATAL_ERROR
                    "Failed to fetch ${ARG_NAME} commit ${ARG_GIT_COMMIT}")
            endif()
        endif()
        execute_process(
            COMMAND git checkout --detach "${ARG_GIT_COMMIT}"
            WORKING_DIRECTORY "${source_dir}"
            RESULT_VARIABLE checkout_result
            OUTPUT_QUIET
        )
        if(NOT checkout_result EQUAL 0)
            message(FATAL_ERROR
                "Failed to checkout ${ARG_NAME} commit ${ARG_GIT_COMMIT}")
        endif()
    endif()

    if(ARG_OUT_SOURCE_DIR)
        set(${ARG_OUT_SOURCE_DIR} "${source_dir}" PARENT_SCOPE)
    endif()
endfunction()
