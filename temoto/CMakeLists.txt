cmake_minimum_required(VERSION 2.8.3)
project(temoto)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
  actionlib
  actionlib_msgs
  genmsg
  geometry_msgs
  griffin_powermate
  jog_arm
  jog_msgs
  keyboard_reader
  message_generation
  move_base_msgs
  moveit_core
  moveit_ros_planning_interface
  robotiq_c_model_control
  roscpp
  rospy
  std_msgs
  tf
  tf2
  tf2_ros
  view_controller_msgs
)

# To determine keyboard priority
find_package(X11 REQUIRED)

add_action_files(DIRECTORY action/temoto
   FILES
   PreplannedSequence.action
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

link_directories(${X11_LIBRARY_DIR})

add_library(check_for_temoto_keyboard_priority src/check_for_temoto_keyboard_priority.cpp)   
target_link_libraries(check_for_temoto_keyboard_priority ${catkin_LIBRARIES} ${X11_LIBRARIES} )   
add_dependencies(check_for_temoto_keyboard_priority temoto_generate_messages_cpp)

# Preplanned sequences:

add_library(close_gripper src/preplanned_sequences/close_gripper.cpp)
target_link_libraries(close_gripper ${catkin_LIBRARIES})

add_library(open_gripper src/preplanned_sequences/open_gripper.cpp)
target_link_libraries(open_gripper ${catkin_LIBRARIES})


##########################################
# This is where most of the action happens
##########################################
add_executable(start_teleop src/start_teleop.cpp src/interpret_utterance.cpp src/preplanned_sequences.cpp src/navigate_robot.cpp src/graphics_and_frames.cpp src/move_robot.cpp src/get_ros_params.cpp src/make_compliant_adjustment.cpp)
add_dependencies(start_teleop temoto_generate_messages_cpp )
# Add one-off commands to trigger short, pre-planned sequences here
target_link_libraries(start_teleop ${catkin_LIBRARIES} close_gripper open_gripper )

add_executable(kb_fake_voice_commander src/kb_fake_voice_commander.cpp)		
target_link_libraries(kb_fake_voice_commander ${catkin_LIBRARIES} )		
add_dependencies(kb_fake_voice_commander temoto_generate_messages_cpp)