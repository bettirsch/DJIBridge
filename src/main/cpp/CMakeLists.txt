cmake_minimum_required(VERSION 3.10.2)
project(djiunity LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The board localizer has its own official AprilTag dependency. It is not
# referenced by the removed phone AR implementation.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_WRAPPER OFF CACHE BOOL "" FORCE)
set(APRILTAG_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../third_party/apriltag)
add_subdirectory(${APRILTAG_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/apriltag)

add_library(djiunity SHARED
        djiunity.cpp
        dji_board_vision.cpp)

target_include_directories(djiunity PRIVATE
        ${APRILTAG_SOURCE_DIR})

find_library(log-lib log)
find_library(android-lib android)
find_library(GLESv2-lib GLESv2)
find_library(EGL-lib EGL)

target_link_libraries(djiunity
        ${log-lib}
        ${android-lib}
        ${GLESv2-lib}
        ${EGL-lib}
        apriltag
)
