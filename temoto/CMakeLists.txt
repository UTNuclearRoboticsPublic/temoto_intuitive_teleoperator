cmake_minimum_required(VERSION 2.8.3)
project(temoto)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
  actionlib_msgs
  griffin_powermate
  jog_arm
  jog_msgs
  keyboard_reader
  robotiq_c_model_control
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  jog_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib_msgs geometry_msgs robotiq_c_model_control roscpp rospy std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

##########################################
# This is where most of the action happens
##########################################
add_executable(start_teleop
  src/get_ros_params.cpp
  src/graphics_and_frames.cpp
  src/grippers.cpp
  src/interpret_utterance.cpp
  src/make_compliant_adjustment.cpp
  src/move_robot.cpp
  src/navigate_robot.cpp
  src/start_teleop.cpp
)
add_dependencies(start_teleop temoto_generate_messages_cpp )
# Add one-off commands to trigger short, pre-planned sequences here
target_link_libraries(start_teleop ${catkin_LIBRARIES})

add_executable(kb_fake_voice_commander src/kb_fake_voice_commander.cpp)		
target_link_libraries(kb_fake_voice_commander ${catkin_LIBRARIES} )		
add_dependencies(kb_fake_voice_commander temoto_generate_messages_cpp keyboard_reader)