// Copyright (c) 2016, The University of Texas at Austin
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

// ROS includes
#include "ros/ros.h"
#include "navigate_robot.h"

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/** This method is executed when temoto/navigate_robot_service service is called.
 *  It updates target_pose_stamped based on the client's request and sets the new_move_req flag to let main() know that moving of the robot has been requested.
 *  @param req temoto::Goal service request.
 *  @param res temoto::Goal service response.
 *  @return always true.
 */
bool NavigateRobotInterface::serviceUpdate(temoto::Goal::Request  &req,
					   temoto::Goal::Response &res)
{
//  ROS_INFO("New service update requested.");
  navigation_goal_stamped_ = req.goal;
  new_navgoal_requested_ = true;
  return true;
} // end serviceUpdate

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigate_robot");

  //tell the action client that we want to spin a thread by default
//   MoveBaseClient ac("move_base", true);
  NavigateRobotInterface navigateIF("move_base");

  //wait for the action server to come up
  while(!navigateIF.move_base_aclient_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  navigateIF.move_base_aclient_.sendGoal(goal);

  navigateIF.move_base_aclient_.waitForResult();

  if(navigateIF.move_base_aclient_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}