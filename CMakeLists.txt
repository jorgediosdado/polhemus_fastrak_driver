cmake_minimum_required(VERSION 2.8.3)
project(polhemus_fastrak_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  tf2
  cmake_modules  
  leica_ts12
  std_msgs
  geometry_msgs
)

find_package(Eigen REQUIRED) 


################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp tf std_msgs calibration leica_ts12 tf2 cmake_modules message_runtime
	
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}	
)

add_executable(fastrak_driver 
  src/main.cpp 
  src/fastrak.cpp

  
  )

target_link_libraries(fastrak_driver
  ${catkin_LIBRARIES}
  )


#############
## Install ##
#############
