cmake_minimum_required(VERSION 3.20)

project(casting_auto_parting LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(casting_auto_parting
    src/AutoPartingPipeline.cpp
    src/Geometry.cpp
)
if(MSVC)
    target_compile_options(casting_auto_parting PRIVATE /utf-8)
endif()

target_include_directories(casting_auto_parting
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/casting
)

add_executable(casting_auto_parting_demo src/main.cpp)
target_link_libraries(casting_auto_parting_demo PRIVATE casting_auto_parting)
if(MSVC)
    target_compile_options(casting_auto_parting_demo PRIVATE /utf-8)
endif()

option(NXOPEN_ENABLED "Build NXOpen entry point" OFF)
set(NXOPEN_ROOT_DIR "" CACHE PATH "NX installation root directory (contains UGOPEN)")
set(NXOPEN_INCLUDE_DIR "" CACHE PATH "NXOpen include directory")
set(NXOPEN_LIB_DIR "" CACHE PATH "NXOpen library directory")

if(NXOPEN_ENABLED)
    if(NXOPEN_INCLUDE_DIR STREQUAL "")
        if(NXOPEN_ROOT_DIR STREQUAL "" AND DEFINED ENV{UGII_BASE_DIR})
            set(NXOPEN_ROOT_DIR "$ENV{UGII_BASE_DIR}")
        endif()
        if(NXOPEN_ROOT_DIR AND EXISTS "${NXOPEN_ROOT_DIR}/UGOPEN")
            set(NXOPEN_INCLUDE_DIR "${NXOPEN_ROOT_DIR}/UGOPEN")
        endif()
    endif()

    if(NOT EXISTS "${NXOPEN_INCLUDE_DIR}")
        message(FATAL_ERROR "NXOPEN_INCLUDE_DIR must point to the NX UGOPEN include folder (e.g., C:/Siemens/NX12.0/UGOPEN).")
    endif()

    if(NXOPEN_LIB_DIR STREQUAL "")
        if(EXISTS "${NXOPEN_INCLUDE_DIR}/lib")
            set(NXOPEN_LIB_DIR "${NXOPEN_INCLUDE_DIR}/lib")
        else()
            set(NXOPEN_LIB_DIR "${NXOPEN_INCLUDE_DIR}")
        endif()
    endif()

    find_library(NXOPEN_UFUN_LIB NAMES ufun libufun HINTS "${NXOPEN_LIB_DIR}")
    find_library(NXOPEN_NXOPEN_LIB NAMES nxopen libnxopen HINTS "${NXOPEN_LIB_DIR}")
    if(NOT NXOPEN_UFUN_LIB OR NOT NXOPEN_NXOPEN_LIB)
        message(FATAL_ERROR "NXOpen libraries not found. Set NXOPEN_LIB_DIR to the folder containing libufun/libnxopen.")
    endif()

    add_library(casting_auto_parting_nxopen SHARED src/NXOpenEntry.cpp)
    if(MSVC)
        target_compile_options(casting_auto_parting_nxopen PRIVATE /utf-8)
    endif()
    target_include_directories(casting_auto_parting_nxopen
        PRIVATE
            ${NXOPEN_INCLUDE_DIR}
    )
    target_link_libraries(casting_auto_parting_nxopen
        PRIVATE
            casting_auto_parting
            ${NXOPEN_UFUN_LIB}
            ${NXOPEN_NXOPEN_LIB}
    )
    set_target_properties(casting_auto_parting_nxopen PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
    if(WIN32)
        set_target_properties(casting_auto_parting_nxopen PROPERTIES
            PREFIX ""
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/nxopen"
        )
        foreach(config Debug Release RelWithDebInfo MinSizeRel)
            set_target_properties(casting_auto_parting_nxopen PROPERTIES
                RUNTIME_OUTPUT_DIRECTORY_${config} "${CMAKE_BINARY_DIR}/nxopen"
            )
        endforeach()
    endif()
endif()
