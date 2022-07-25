cmake_minimum_required(VERSION 3.10)

project(gtsam_tutorial)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Boost COMPONENTS serialization REQUIRED)
find_package(Boost COMPONENTS chrono REQUIRED)
find_package(Boost COMPONENTS date_time REQUIRED)
find_package(Boost COMPONENTS timer REQUIRED)

add_subdirectory(src)