cmake_minimum_required(VERSION 3.10)

project(gtsamTutorial)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
find_package(GTSAM REQUIRED)
