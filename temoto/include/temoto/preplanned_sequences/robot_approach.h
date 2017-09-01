#ifndef ROBOT_APPROACH_H
#define ROBOT_APPROACH_H

#include "move_base_to_manip/desired_robot_pose.h"
#include "ros/ros.h"
#include "temoto/temoto_common.h"

class robot_approach
{
public:
  // ___ CONSTRUCTOR ___
  robot_approach();

private:
  // Listen to the goal pose (also, a marker in RViz)
  void desired_pose_cb_(const temoto::Status::ConstPtr& msg);

  // A service which provides the goal pose to move_base_to_manip
  bool desired_pose_srvc(move_base_to_manip::desired_robot_pose::Request  &req,
         move_base_to_manip::desired_robot_pose::Response &res);

  geometry_msgs::PoseStamped desired_pose_;

  ros::NodeHandle n_;

  bool shutdown_now_ = true;

};

#endif
