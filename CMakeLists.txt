cmake_minimum_required(VERSION 2.8.3)
project(ardrone_geofencing)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare a catkin package
catkin_package() 

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(geofencing src/main.cpp)
target_link_libraries(geofencing ${catkin_LIBRARIES})
