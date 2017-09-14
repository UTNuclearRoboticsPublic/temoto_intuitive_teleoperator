
#ifndef ROBOT_PUSH_BUTTON_H
#define ROBOT_PUSH_BUTTON_H

#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/PointStamped.h"
#include <move_base_to_manip/desired_poseAction.h>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "vaultbot_irp/move_to_given_pose.h"

class robot_push_button
{
public:
  // ___ CONSTRUCTOR ___
  robot_push_button();

private:
  ros::NodeHandle n_;
  geometry_msgs::PoseStamped approach_pose_;
  geometry_msgs::PoseStamped button_pose_;
  tf::TransformListener listener_;

  // Callback for when a pt is clicked in RViz
  void clicked_pt_cb(const geometry_msgs::PointStamped::ConstPtr& button_point);

};

#endif
