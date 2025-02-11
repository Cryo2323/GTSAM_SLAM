cmake_minimum_required(VERSION 2.8.11)
project (HW3)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})


add_executable(gtsam2DSLAM gtsam2DSLAM.cpp)
add_executable(gtsam3DSLAM gtsam3DSLAM.cpp)

target_link_libraries(gtsam2DSLAM gtsam)
target_link_libraries(gtsam3DSLAM gtsam)

set_target_properties(gtsam2DSLAM PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
set_target_properties(gtsam3DSLAM PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

