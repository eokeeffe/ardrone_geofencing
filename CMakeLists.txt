cmake_minimum_required(VERSION 2.8.3)
project(ardrone_geofencing)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
find_package(Eigen3 REQUIRED)

## Declare a catkin package
catkin_package()

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    # Other flags
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/src
)

#add_executable(eigentest src/eigentest/main.cpp)
#target_link_libraries(eigentest ${catkin_LIBRARIES})

# Kalman filters
add_library(kalman src/KF.cpp src/KF.h)

# PID Controllers
add_library(PID src/PID.cpp src/PID.h)

# Main GeoFencing Application
add_executable(geofencing src/main.cpp src/main.h)
target_link_libraries(geofencing ${catkin_LIBRARIES} kalman PID)
#

# Mark executables and/or libraries for installation
install(TARGETS geofencing
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
