cmake_minimum_required(VERSION 3.10)

project (gtsamTutorial)

find_package(GTSAM REQUIRED)
find_package(Boost 1.50 REQUIRED)

add_executable(robotMotion_src robotMotion.cpp)
# add_executable(robotLocalization_src robotLocalization.cpp)

