// Copyright (c) 2015-2016, The University of Texas at Austin
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
// 
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
// 
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/** @file move_robot.cpp
 * 
 *  @brief ROS server that acts as an interface for ROS MoveIt! and publishes end-effector pose.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

#include "temoto/low_level_cmds.h"
#include "temoto/move_robot.h"

/** This method is executed when temoto/move_robot_service service is called.
 *  It updates the target member variable based on the client's request and sets the new_move_req flag to let main() know that moving of the robot has been requested.
 *  @param req temoto::Goal service request.
 *  @param res temoto::Goal service response.
 *  @return always true.
 */
bool MoveRobotInterface::serviceUpdate(temoto::Goal::Request  &req,
				       temoto::Goal::Response &res)
{
  // sets the action associated with target pose, e.g. low_level_cmds::PLAN
  req_action_type_ = req.action_type;
  
  target_pose_stamped_ = req.goal_pose;

  ROS_ERROR_STREAM( "[move_robot/serviceUpdate] target_pose_stamped_: " << target_pose_stamped_ );

  joint_deltas_.clear();
  for( int i=0; i< req.joint_deltas.size(); i++)
  {
    joint_deltas_.push_back( req.joint_deltas.at(i) );
  }
    					// by default do not use named_target.
  named_target_ = req.named_target;				// get named_target from the service request.
    
  // Set new_move_requested_ TRUE for main() to see it
  new_move_requested_ = true;
  
  return true;
} // end serviceUpdate

/** Plans and/or executes the motion of the robot.
 *  @param robot MoveGroup object of the robot.
 */
void MoveRobotInterface::requestMove()
{

  // Set current state as the start state for planner. For some reason the actual built-in function doesn't do that.
  movegroup_.setStartState( *(movegroup_.getCurrentState()) );
  
  // Get and print current position of the end effector
/*
  geometry_msgs::PoseStamped current_pose = movegroup_.getCurrentPose();
  ROS_INFO("[move_robot/requestMove] === CURRENT POSE ( as given by MoveGroup::getCurrentPose() ) ===");
  ROS_INFO("[move_robot/requestMove] Current pose frame: %s", current_pose.header.frame_id.c_str());
  ROS_INFO("[move_robot/requestMove] Current pose (posit x, y, z): (%f, %f, %f)", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  ROS_INFO("[move_robot/requestMove] Current pose (orien x, y, z, w): (%f, %f, %f, %f)", current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
*/
 
  ////////////////////////////////////////////////////////////////////////////////
  // Use a named, joint, or pose target. (whichever was populated in the Goal msg)
  // Named target:
  ////////////////////////////////////////////////////////////////////////////////
  if (named_target_ != "")
  {
    if ( movegroup_.setNamedTarget(named_target_) )		// check if setting named target was successful
    {
      ROS_INFO("[move_robot/requestMove] Failed to set named target. Please retry.");
      return;								// return if setNamedTarget failed
    }
    else
      ROS_INFO("[move_robot/requestMove] Using NAMED TARGET for planning and/or moving.");
  }

  //////////////
  // Pose target
  //////////////
  else if ( target_pose_stamped_.header.frame_id != "" )  // The Goal msg had a pose target
  {
    //ROS_INFO("[move_robot/requestMove] Found end effector link: %s", movegroup_.getEndEffectorLink().c_str());
    // use the stamped target pose to set the target pose for robot
    if ( !movegroup_.setPoseTarget(target_pose_stamped_ ) )	// check if set target pose failed
    {
      ROS_INFO("[move_robot/requestMove] Failed to set pose target. Please retry.");
      return;								// return if setPoseTarget failed
    }
    else
      ROS_INFO("[move_robot/requestMove] Using POSE TARGET for planning and/or moving.");
  }

  ///////////////
  // Joint target
  ///////////////
  else if ( joint_deltas_.size() != 0 )  // The Goal msg had a joint target
  {
    // Get the current joints
    std::vector<double> joints = movegroup_.getCurrentJointValues();

    // Add the deltas (another std::vector<double>)
    for (int i=0; i<joint_deltas_.size(); i++)
      joints.at(i) += joint_deltas_.at(i);

    // Move to the new target joints
    movegroup_.setJointValueTarget(joints);
    movegroup_.move();

    return;
  }

  // Continue with MoveGroup stuff for Cartesian moves

  // Set goal tolerance based on the actual shift from current position to target position.
  // Apply minimums, e.g. we can't control motion down to the nanometer level.
  geometry_msgs::PoseStamped current_pose_stamped = movegroup_.getCurrentPose();
  calculate_linear_tols(current_pose_stamped, target_pose_stamped_);
  calculate_ang_tols(current_pose_stamped, target_pose_stamped_);
  
  // Just checking what is the target pose
  geometry_msgs::PoseStamped current_target = movegroup_.getPoseTarget();
  //ROS_INFO("[move_robot/requestMove] Target pose frame: %s", current_target.header.frame_id.c_str());
  //ROS_INFO_STREAM("[move_robot/requestMove] Target pose: " << current_target.pose);

  // Based on action type: PLAN, EXECUTE PLAN, or PLAN&EXECUTE (aka GO)
  if ( req_action_type_ == low_level_cmds::PLAN )
  {
    //ROS_INFO("[move_robot/requestMove] Starting to plan ...");
    //ROS_INFO("[move_robot/requestMove] Planning frame: %s", movegroup_.getPlanningFrame().c_str());
    movegroup_.plan( latest_plan_ );				// Calculate plan and store it in latest_plan_.
    new_plan_available_ = true;					// Set new_plan_available_ to TRUE.
    //ROS_INFO("[move_robot/requestMove] DONE planning.");
  }
  else if ( req_action_type_ == low_level_cmds::EXECUTE )
  {
    //ROS_INFO("[move_robot/requestMove] Starting to execute last plan ...");
    if ( new_plan_available_ ) movegroup_.execute( latest_plan_ );	// If there is a new plan, execute latest_plan_.
    else ROS_INFO("[move_robot/requestMove] No plan to execute.");	// Else do nothing but printout "no plan"
    new_plan_available_ = false;					// Either case, set new_plan_available_ to FALSE.
    //ROS_INFO("[move_robot/requestMove] DONE executing the plan");
  }
  else if ( req_action_type_ == low_level_cmds::GO )
  {
    //ROS_INFO("[move_robot/requestMove] Starting to move (i.e. plan & execute) ...");
//     robot.move();						// Plan and execute.
    // Since move() has a bug of start state not being current state, I am going to plan and execute sequentally.
    moveit::planning_interface::MoveGroup::Plan move_plan;
    //printf("[move_robot/requestMove] Planning ...");
    movegroup_.plan( move_plan );
    //printf("[DONE] \n[move_robot/requestMove] and Executing ...\n");
    movegroup_.execute( move_plan );
    //ROS_INFO("[move_robot/requestMove] DONE moving.");
    new_plan_available_ = false;					// As any previous plan has become invalid, set new_plan_available_ to FALSE.
  }

  return;
} // end requestMove

void MoveRobotInterface::calculate_linear_tols(geometry_msgs::PoseStamped curr_pose, geometry_msgs::PoseStamped target_pose)
{
  double sum_of_squares = pow( curr_pose.pose.position.x - target_pose.pose.position.x,2) + pow( curr_pose.pose.position.y - target_pose.pose.position.y,2) + pow( curr_pose.pose.position.z - target_pose.pose.position.z,2);

  double delta_position = pow( sum_of_squares, 0.5);

  delta_position *= fractional_tolerance_;

  if ( delta_position < min_position_tolerance_ )
    delta_position = min_position_tolerance_;

  movegroup_.setGoalPositionTolerance( delta_position );

  return;
}

void MoveRobotInterface::calculate_ang_tols(geometry_msgs::PoseStamped curr_pose, geometry_msgs::PoseStamped target_pose)
{
  double sum_of_squares = pow( curr_pose.pose.orientation.x - target_pose.pose.orientation.x,2) + pow( curr_pose.pose.orientation.y - target_pose.pose.orientation.y,2) + pow( curr_pose.pose.orientation.z - target_pose.pose.orientation.z,2) + pow( curr_pose.pose.orientation.w - target_pose.pose.orientation.w,2);

  double delta_orientation = pow( sum_of_squares, 0.5);

  delta_orientation *= fractional_tolerance_;

  if ( delta_orientation < min_orientation_tolerance_ )
    delta_orientation = min_orientation_tolerance_;

  movegroup_.setGoalOrientationTolerance( delta_orientation );

  return;
}

/** Main */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_robot");
  // ROS Nodehandle for normal things
  ros::NodeHandle n;
  
  ros::Rate node_rate(60);

  // Using an async spinner. It is needed for moveit's MoveGroup::plan(), which would get stuck otherwise. Might be a bug.
  ros::AsyncSpinner spinner(0);
  spinner.start();
  
  // get user-specified name for the movegroup as a private parameter
  std::string move_group_name;
  n.getParam("temoto/movegroup", move_group_name);
  if ( move_group_name.empty() )
  {
    ROS_ERROR("[move_robot/main] No movegroup name was specified. Aborting.");
    return -1;
  }
  ROS_INFO("[move_robot/main] Retrieved '%s' from parameter server as a movegroup name.", move_group_name.c_str());
  
  MoveRobotInterface moveIF(move_group_name);
  
  // FYI
  ROS_INFO("[move_robot/main] Planning frame: %s", moveIF.movegroup_.getPlanningFrame().c_str());
  ROS_INFO("[move_robot/main] End effector link: %s", moveIF.movegroup_.getEndEffectorLink().c_str());
  ROS_INFO("[move_robot/main] End effector: %s", moveIF.movegroup_.getEndEffector().c_str());
  ROS_INFO("[move_robot/main] Goal position tolerance is: %.6f", moveIF.movegroup_.getGoalPositionTolerance());
  ROS_INFO("[move_robot/main] Goal orientation tolerance is: %.6f", moveIF.movegroup_.getGoalOrientationTolerance());
  ROS_INFO("[move_robot/main] Goal joint tolerance is: %.6f", moveIF.movegroup_.getGoalJointTolerance());
  
  // Set up service for move_robot_service; if there's a service request, executes serviceUpdate() function
  ros::ServiceServer service = n.advertiseService("temoto/move_robot_service", &MoveRobotInterface::serviceUpdate, &moveIF);
  ROS_INFO("[move_robot/main] Service 'temoto/move_robot_service' up and going. Ready to send move commands to %s.", move_group_name.c_str());
  
  // Set up publisher for the end effector location
  ros::Publisher pub_end_effector = n.advertise<geometry_msgs::PoseStamped>( "temoto/end_effector_pose", 1 );

  // For frame transformations
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transform_stamped;
  geometry_msgs::PoseStamped current_pose;

  // Wait for the moveGroup to initialize
  ros::Duration(2.).sleep();  // TODO: a better way
/*
  while ( current_pose.header.frame_id == "" )
  {
    ros::Duration(0.1).sleep();
    current_pose = moveIF.movegroup_.getCurrentPose();
  }
*/

  // MAIN LOOP
  while ( ros::ok() )
  {
    // check if there has been a service request for a new move
    if ( moveIF.new_move_requested_ )
    {
      moveIF.requestMove();			// plan and execute move using move_group
      moveIF.new_move_requested_ = false;	// set request flag to false
    }

    // get and publish current end effector pose. Ensure it's in base_link frame
    current_pose = moveIF.movegroup_.getCurrentPose();

    if ( current_pose.header.frame_id != "base_link" )
    {
      // Check for and remove a leading slash
      std::string frame = current_pose.header.frame_id;
      if ( frame[0] == '/' )
      {
        frame.erase(0,1);
        current_pose.header.frame_id = frame;
      }

      try
      {
        transform_stamped = tfBuffer.lookupTransform("base_link", frame, ros::Time(0));
        tf2::doTransform(current_pose, current_pose, transform_stamped);
      }
      catch (tf2::TransformException &ex) 
      {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
    }  // if not already in base_link

    pub_end_effector.publish( current_pose );
   
    ros::spinOnce();
    node_rate.sleep();
    
  } // end while

  return 0;
} // end main
