cmake_minimum_required(VERSION 2.8.3)
project(fastrak_mocap_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp tf
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fastrak_allSensorRead 
  src/allSensorRead.cpp 
  src/fastrak_serialport.cpp
  )

target_link_libraries(fastrak_allSensorRead
  ${catkin_LIBRARIES}
  )

#############
## Install ##
#############
