cmake_minimum_required(VERSION 3.10.2)
project(djiunity LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV's Android AAR publishes this target through Gradle Prefab.
find_package(OpenCV REQUIRED CONFIG)

file(GLOB APRILTAG_COMMON_SRC
        apriltag/common/*.c
)

add_library(djiunity SHARED
        djiunity.cpp
        djiunity_apriltag.cpp
        apriltag/apriltag.c
        apriltag/apriltag_pose.c
        apriltag/apriltag_quad_thresh.c
        apriltag/tagStandard41h12.c
        ${APRILTAG_COMMON_SRC}
)

target_include_directories(djiunity PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/apriltag
        ${CMAKE_CURRENT_SOURCE_DIR}/apriltag/common
        ${CMAKE_CURRENT_SOURCE_DIR}/apriltag/contrib
)

find_library(log-lib log)
find_library(android-lib android)
find_library(GLESv2-lib GLESv2)
find_library(EGL-lib EGL)
find_library(m-lib m)

target_link_libraries(djiunity
        ${log-lib}
        ${android-lib}
        ${GLESv2-lib}
        ${EGL-lib}
        ${m-lib}
        OpenCV::opencv_java4
)
