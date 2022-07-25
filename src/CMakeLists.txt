# add_executable(robotMotion_src robotMotion.cpp)
add_executable(SimpleRotation_src SimpleRotation.cpp)

find_package(GTSAM REQUIRED)
# include_directories(${gtsam_INCLUDE_DIR})

# target_link_libraries (robotMotion_src LINK_PUBLIC gtsam)
target_link_libraries (SimpleRotation_src LINK_PUBLIC gtsam)
