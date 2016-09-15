cmake_minimum_required(VERSION 2.8.3)
project(griffin_powermate)

set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PowermateEvent.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES keyboard
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

add_executable(griffin_powermate src/griffin_powermate.cpp)
target_link_libraries(griffin_powermate ${catkin_LIBRARIES})
add_dependencies(griffin_powermate griffin_powermate_generate_messages_cpp)
