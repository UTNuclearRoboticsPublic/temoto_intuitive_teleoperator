#ifndef TURN_HANDLE_COUNTERCLOCKWISE_H
#define TURN_HANDLE_COUNTERCLOCKWISE_H

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <tf/transform_datatypes.h>
#include "ur_script_compliance.h"

namespace turn_handle_counterclockwise
{
  int turn_handle_counterclockwise();

  void enable_compliance(ur_script_compliance& right);

  int rotate(ros::NodeHandle* nhPtr);
}

#endif
