cmake_minimum_required(VERSION 2.8.3)
project(temoto)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
  actionlib
  griffin_powermate
  jog_msgs
  keyboard_reader
  moveit_ros_planning_interface
  robotiq_2f_gripper_control
  std_srvs
  tf
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    actionlib
    griffin_powermate
    jog_msgs
    keyboard_reader
    moveit_ros_planning_interface
    robotiq_2f_gripper_control
    std_srvs
    tf
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

##########################################
# Gripper libraries are dynamically linked
##########################################
add_library(gripper_robotiq
  src/gripper_base_class
)
add_dependencies(gripper_robotiq ${catkin_EXPORTED_TARGETS})

##########################################
# This is where most of the action happens
##########################################
add_executable(start_teleop
  src/get_ros_params.cpp
  src/graphics_and_frames.cpp
  src/grippers.cpp
  src/gripper_robotiq.cpp
  src/move_robot.cpp
  src/navigate_robot.cpp
  src/start_teleop.cpp
)
add_dependencies(start_teleop ${catkin_EXPORTED_TARGETS})
target_link_libraries(start_teleop ${catkin_LIBRARIES})

add_executable(kb_fake_voice_commander src/kb_fake_voice_commander.cpp)		
target_link_libraries(kb_fake_voice_commander ${catkin_LIBRARIES} )		
add_dependencies(kb_fake_voice_commander ${catkin_EXPORTED_TARGETS})
