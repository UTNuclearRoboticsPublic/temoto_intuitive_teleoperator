#ifndef TURN_HANDLE_CLOCKWISE_H
#define TURN_HANDLE_CLOCKWISE_H

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <tf/transform_datatypes.h>
#include "ur_script_compliance.h"

namespace turn_handle_clockwise
{
  int turn_handle_clockwise();

  void enable_compliance();

  int rotate();

  ros::NodeHandle nh;

  // arm compliance object. Sends ur commands on this topic
  ur_script_compliance right("/right_ur5_controller/right_ur5_URScript");

  moveit::planning_interface::MoveGroupInterface move_group("right_ur5");
}
#endif
