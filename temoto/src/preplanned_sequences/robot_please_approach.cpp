#include "temoto/preplanned_sequences/robot_please_approach.h"

robot_please_approach::robot_please_approach()
{
  ROS_INFO_STREAM("----------------------------");
  ROS_INFO_STREAM("[robot_please_approach] Select a point in RViz");
  ROS_INFO_STREAM("----------------------------");

  // Should start from a position that's roughly in front of the target pose.
  
  /////////////////////////////////////////////////////////////////////
  // Select the button's pose in RViz. It's published on /clicked_point
  /////////////////////////////////////////////////////////////////////
  ros::Subscriber sub = n_.subscribe("/clicked_point", 1, &robot_please_approach::clicked_pt_cb, this);

  // wait for user to select a point.
  // The cb makes sure it's in a global frame (map)
  while ( button_pose_.header.frame_id == "" )
  {
  	ros::Duration(1.).sleep();
  	ROS_INFO_STREAM("[robot_please_approach] Select a point in RViz");
  }


  /////////////////////////////////////////////////////
  // Set an approach pose a few cm's in front of target
  /////////////////////////////////////////////////////
  approach_pose_ = button_pose_;
  approach_pose_.pose.position.x += -x_offset_;


  ///////////////////////////////////////////////////
  // Approach the selected pose w/ move_base_to_manip
  // It will get close but probably not completely there
  ///////////////////////////////////////////////////


  actionlib::SimpleActionClient<move_base_to_manip::desired_poseAction> ac("move_base_to_manip", true);
  ROS_INFO("[provide_target] Waiting for the move_base_to_manip action server to start.");
  ac.waitForServer();

  move_base_to_manip::desired_poseGoal mbtm_goal;
  mbtm_goal.desired_pose = approach_pose_;
  ROS_INFO("[robot_please_approach] Sending pose goal...");
  ROS_INFO_STREAM(mbtm_goal);
  ac.sendGoal(mbtm_goal);

  // Wait for the action server to return
  if ( ac.waitForResult(ros::Duration(30.0)) != true )
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("[robot_please_approach] Action finished: %s",state.toString().c_str());
  }


  /////////////////////////////////////////////////////
  // May need to plan a finer move to the approach pose
  /////////////////////////////////////////////////////
/*
  // get another click from the user
  button_pose_.header.frame_id = "";


  while ( button_pose_.header.frame_id == "" )
  {
  	ros::Duration(1.).sleep();
  	ROS_INFO_STREAM("[robot_please_approach] Select a point in RViz");
  }

  approach_pose_ = button_pose_;
  approach_pose_.pose.position.x += -x_offset_;


  // move there

  ROS_INFO_STREAM("[robot_please_approach] Moving the arm to the approach pose");

  goal.desired_pose = approach_pose_;
  ROS_INFO("[robot_please_approach] Sending pose goal...");
  ROS_INFO_STREAM(goal);
  ac.sendGoal(goal);

  // Wait for the action server to return
  if ( ac.waitForResult(ros::Duration(30.0)) != true )
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("[robot_please_approach] Action finished: %s",state.toString().c_str());
  }
*/

  /////////////////////////////////////////////////////////////
  // For now, just move forward a set amount to push the button
  /////////////////////////////////////////////////////////////

  geometry_msgs::PoseStamped goal_pose = approach_pose_;
  goal_pose.pose.position.x += x_offset_;

  mbtm_goal.desired_pose = approach_pose_;
  ROS_INFO("[robot_please_approach] Sending pose goal...");
  ROS_INFO_STREAM(mbtm_goal);
  ac.sendGoal(mbtm_goal);

  // Wait for the action server to return
  if ( ac.waitForResult(ros::Duration(30.0)) != true )
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("[robot_please_approach] Action finished: %s",state.toString().c_str());
  }

  //////////////////////////////
  // Move back to the start pose
  //////////////////////////////

  ROS_INFO_STREAM("[robot_please_approach] returning");
}

// Callback for when a pt is clicked in RViz
void robot_please_approach::clicked_pt_cb(const geometry_msgs::PointStamped::ConstPtr& button_point)
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
}
