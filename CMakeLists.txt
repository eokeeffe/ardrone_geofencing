cmake_minimum_required(VERSION 2.8.3)
project(ardrone_geofencing)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
find_package(Eigen3 REQUIRED)

## Declare a catkin package
catkin_package()

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/src
)

#add_executable(eigentest src/eigentest/main.cpp)
#target_link_libraries(eigentest ${catkin_LIBRARIES})

# Kalman filters
add_library(kalman src/KF.cpp src/KF.h)

# Main GeoFencing Application
add_executable(geofencing src/main.cpp src/main.h)
target_link_libraries(geofencing ${catkin_LIBRARIES})
#
