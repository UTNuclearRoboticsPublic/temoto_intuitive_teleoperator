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
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/** @file navigate_robot.cpp
 *
 *  @brief ROS server that acts as an interface for ROS navigation stack.
 *
 *  @author karl.kruusamae(at)utexas.edu
 */

#include "temoto/navigate_robot.h"
#include "temoto/common_commands.h"

bool NavigateRobotInterface::navRequest(std::string action_type, geometry_msgs::PoseStamped goal_pose)
{
  ROS_WARN_STREAM("Received nav request.");

  // Wait for the action server to come up
  if (!move_base_aclient_.waitForServer(ros::Duration(5.0)))
  {
    ROS_WARN("[temoto/navigate_robot] The move_base action server is not "
             "available.");
    return false;
  }

  // check first if abort navigation has been requested.
  if (action_type == common_commands::ABORT)
  {
    move_base_aclient_.cancelGoal();
    return true;
  }
  else
  {
    navigation_goal_stamped_ = goal_pose;

    move_base_msgs::MoveBaseGoal mb_goal;

    mb_goal.target_pose = navigation_goal_stamped_;

    ROS_INFO("[temoto/navigate_robot] Sending navigation goal");

    move_base_aclient_.sendGoal(mb_goal);

    return true;
  }
}
