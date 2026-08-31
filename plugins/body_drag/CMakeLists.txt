cmake_minimum_required(VERSION 3.16)
project(px4_gz_body_drag)

# Standalone: deliberately NOT wired into the PX4 build, so it cannot break
# `make px4_sitl`. Build it once with ./build.sh and put the result on
# GZ_SIM_SYSTEM_PLUGIN_PATH.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS profiler)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_library(BodyDrag SHARED BodyDrag.cc)
target_include_directories(BodyDrag PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(BodyDrag
  PRIVATE
    gz-plugin2::register
    gz-sim8::gz-sim8
    gz-common5::profiler
    gz-math7::gz-math7
    gz-msgs10::gz-msgs10
    gz-transport13::gz-transport13
)
