cmake_minimum_required(VERSION 3.20)

project(casting_auto_parting LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(casting_auto_parting
    src/AutoPartingPipeline.cpp
    src/Geometry.cpp
)

target_include_directories(casting_auto_parting
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(casting_auto_parting_demo src/main.cpp)
target_link_libraries(casting_auto_parting_demo PRIVATE casting_auto_parting)

option(NXOPEN_ENABLED "Build NXOpen entry point" OFF)
set(NXOPEN_INCLUDE_DIR "" CACHE PATH "NXOpen include directory")

if(NXOPEN_ENABLED)
    if(NOT EXISTS "${NXOPEN_INCLUDE_DIR}")
        message(FATAL_ERROR "NXOPEN_INCLUDE_DIR must be set to a valid NXOpen include path.")
    endif()
    add_library(casting_auto_parting_nxopen SHARED src/NXOpenEntry.cpp)
    target_include_directories(casting_auto_parting_nxopen
        PRIVATE
            ${NXOPEN_INCLUDE_DIR}
    )
    target_link_libraries(casting_auto_parting_nxopen PRIVATE casting_auto_parting)
endif()
