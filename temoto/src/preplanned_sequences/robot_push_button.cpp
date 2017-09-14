#include "temoto/preplanned_sequences/robot_push_button.h"

robot_push_button::robot_push_button()
{
  ROS_INFO_STREAM("----------------------------");
  ROS_INFO_STREAM("[robot_push_button] Select a point in RViz");
  ROS_INFO_STREAM("----------------------------");

  // Should start from a position that's roughly in front of the panel.
  
  /////////////////////////////////////////////////////////////////////
  // Select the button's pose in RViz. It's published on /clicked_point
  /////////////////////////////////////////////////////////////////////
  ros::Subscriber sub = n_.subscribe("/clicked_point", 1, &robot_push_button::clicked_pt_cb, this);

  // wait for user to push a button
  while ( button_pose_.header.frame_id == "" )
  {
  	ros::Duration(1.).sleep();
  	ROS_INFO_STREAM("[robot_push_button] Select a point in RViz");
  }


  /////////////////////////////////////////////////////
  // Set an approach pose a few cm's in front of target
  /////////////////////////////////////////////////////
  approach_pose_ = button_pose_;
  approach_pose_.pose.position.x += -0.03;


  ///////////////////////////////////////////////////
  // Approach the selected pose w/ move_base_to_manip
  // It will get close but maybe not completely there
  ///////////////////////////////////////////////////

  actionlib::SimpleActionClient<move_base_to_manip::desired_poseAction> ac("move_base_to_manip", true);
  ROS_INFO("[provide_target] Waiting for the move_base_to_manip action server to start.");
  ac.waitForServer();

  move_base_to_manip::desired_poseGoal goal;
  goal.desired_pose = approach_pose_;
  ROS_INFO("[provide_target] Sending pose goal...");
  ROS_INFO_STREAM(goal);
  ac.sendGoal(goal);


  ///////////////////////////////////
  // Plan a move to the approach pose
  ///////////////////////////////////

  ros::ServiceClient client = n_.serviceClient<vaultbot_irp::move_to_given_pose>("/arm_services/move_to_given_pose");

  vaultbot_irp::move_to_given_pose srv;
  srv.request.desired_robot_pose = approach_pose_;
  srv.request.move_group_name.data = "right_ur5_temoto";

  if (!client.call(srv))
  {
    ROS_WARN("[arm_services] Could not call /arm_services/move_to_left_laser_scan");
  }


  /////////////////////////////////////////////////////////////
  // For now, just move forward a set amount to push the button
  /////////////////////////////////////////////////////////////


  //////////////////////////////
  // Move back to the start pose
  //////////////////////////////

  ROS_INFO_STREAM("[robot_push_button] returning");
}

// Callback for when a pt is clicked in RViz
void robot_push_button::clicked_pt_cb(const geometry_msgs::PointStamped::ConstPtr& button_point)
{
  geometry_msgs::PointStamped ps;
  ps.header = button_point->header;
  ps.point = button_point->point;

  // Make sure it's in the base_link frame
  tf::StampedTransform transform;
  try{
  	listener_.waitForTransform(button_point->header.frame_id, "/base_link",
                              ros::Time::now(), ros::Duration(3.0));
    listener_.lookupTransform(button_point->header.frame_id, "/base_link",  
                               ros::Time(0), transform);
    }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  listener_.transformPoint("base_link", ps, ps);


  // Save in a class member
  button_pose_.header = ps.header;
  button_pose_.pose.position = ps.point;
  button_pose_.pose.orientation.w = 1;  // Don't change orientation

  ROS_INFO_STREAM("I heard: " << button_pose_ );
}
