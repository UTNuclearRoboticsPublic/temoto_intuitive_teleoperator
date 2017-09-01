#include "temoto/preplanned_sequences/robot_approach.h"

robot_approach::robot_approach()
{
  // Get the marker pose
  // Subscribe to temoto/end_effector_pose
  ros::Subscriber desired_pose_sub = n_.subscribe("temoto/status", 1, &robot_approach::desired_pose_cb_, this);

  // Wait for a pose msg
  while ( desired_pose_.header.frame_id== "" )
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  // move_base_to_manip should already be launched, e.g. from a launch file

  // Provide this pose for move_base_to_manip
  // Start a service. Wait until another node requests the desired pose, then shut down (if requested).
  ros::ServiceServer service = n_.advertiseService("desired_robot_pose", &robot_approach::desired_pose_srvc, this);
  while ( !shutdown_now_ && ros::ok() ) // Until the client(s) requests a shutdown
  {  
    ros::spinOnce();
    ros::Duration(1.).sleep();
    ROS_INFO_STREAM("[robot_approach] Waiting on move_base_to_manip. (was it launched in launch file?)");
  }
  ROS_INFO_STREAM("[robot_approach] Shutting down.");

}



// Listen to the goal pose (also, a marker in RViz)
void robot_approach::desired_pose_cb_(const temoto::Status::ConstPtr& msg)
{
  // Make this pose available to the rest of the class
  desired_pose_ = msg->commanded_pose;
}



// This service provides a desired pose to the requester, then shuts down the node.
bool robot_approach::desired_pose_srvc(move_base_to_manip::desired_robot_pose::Request  &req,
         move_base_to_manip::desired_robot_pose::Response &res)
{
  res.desired_robot_pose.header.frame_id = desired_pose_.header.frame_id;

  res.desired_robot_pose.pose.position.x = desired_pose_.pose.position.x;
  res.desired_robot_pose.pose.position.y = desired_pose_.pose.position.y;
  res.desired_robot_pose.pose.position.z = desired_pose_.pose.position.z;

  res.desired_robot_pose.pose.orientation.x = desired_pose_.pose.orientation.x;
  res.desired_robot_pose.pose.orientation.y = desired_pose_.pose.orientation.y;
  res.desired_robot_pose.pose.orientation.z = desired_pose_.pose.orientation.z;
  res.desired_robot_pose.pose.orientation.w = desired_pose_.pose.orientation.w;
  
  // If shutdown_now is true, this node will shut down
  shutdown_now_ = req.shutdown_service;

  return shutdown_now_;
}
