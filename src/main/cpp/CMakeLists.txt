cmake_minimum_required(VERSION 3.10.2)
project(djiunity LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(djiunity SHARED
        djiunity.cpp
)

find_library(log-lib log)
find_library(android-lib android)
find_library(GLESv2-lib GLESv2)
find_library(EGL-lib EGL)

target_link_libraries(djiunity
        ${log-lib}
        ${android-lib}
        ${GLESv2-lib}
        ${EGL-lib}
)
