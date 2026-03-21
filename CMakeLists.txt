cmake_minimum_required(VERSION 3.16)
project(NX12SecondaryDev VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -------------------------------------------------------------------
# NX 12.0 installation root – override via cmake -DNX_ROOT=<path>
# Default Windows path shipped with NX 12.0
# -------------------------------------------------------------------
if(NOT DEFINED NX_ROOT)
    if(WIN32)
        set(NX_ROOT "C:/Program Files/Siemens/NX 12.0" CACHE PATH "NX 12.0 installation root")
    else()
        set(NX_ROOT "/usr/local/nx12" CACHE PATH "NX 12.0 installation root")
    endif()
endif()

set(UGOPEN_DIR "${NX_ROOT}/UGOPEN")

# Include search paths
include_directories(
    "${UGOPEN_DIR}"
    "${UGOPEN_DIR}/uf_retiring"
    include
)

# Link directories
link_directories(
    "${UGOPEN_DIR}"
)

# Collect sources
file(GLOB_RECURSE NX_SOURCES "src/*.cpp")

# Build as a shared library (DLL) – required for NX Open
add_library(${PROJECT_NAME} SHARED ${NX_SOURCES})

# Platform-specific NX Open libraries
if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        libufun
        libugopenint
    )
    # NX requires the output DLL to have no "lib" prefix
    set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        _USRDLL
        WNT
        _WIN64
    )
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE
        ufun
        ugopenint
    )
    set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib")
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        UNIX
        LINUX
    )
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE
    NX12
    UG_CALL_PROTO=1
)

# Install
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
