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

/** @file move_robot.cpp
 *
 *  @brief ROS server that acts as an interface for ROS MoveIt! and publishes
 * end-effector pose.
 *
 *  @author karl.kruusamae(at)utexas.edu
 */

#include "temoto/move_robot.h"
#include "temoto/low_level_cmds.h"

/** Plans and/or executes the motion of the robot.
 *  @param robot MoveGroup object of the robot.
 */
void MoveRobotInterface::requestMove()
{
  // Set current state as the start state for planner. For some reason the
  // actual built-in function doesn't do that.
  movegroup_.setStartState(*(movegroup_.getCurrentState()));

  ////////////////////////////////////////////////////////////////////////////////
  // Use a named, joint, or pose target. (whichever was populated in the Goal
  // msg)
  // Named target:
  ////////////////////////////////////////////////////////////////////////////////
  if (named_target_ != "")
  {
    if (movegroup_.setNamedTarget(named_target_))  // check if setting named target was successful
    {
      ROS_INFO("[move_robot/requestMove] Failed to set named target. Please retry.");
      return;  // return if setNamedTarget failed
    }
    else
      ROS_INFO("[move_robot/requestMove] Using NAMED TARGET for planning "
               "and/or moving.");
  }

  //////////////
  // Pose target
  //////////////
  else if (target_pose_stamped_.header.frame_id != "")  // The Goal msg had a pose target
  {
    // ROS_INFO("[move_robot/requestMove] Found end effector link: %s",
    // movegroup_.getEndEffectorLink().c_str());
    // use the stamped target pose to set the target pose for robot
    if (!movegroup_.setPoseTarget(target_pose_stamped_))  // check if set target pose failed
    {
      ROS_INFO("[move_robot/requestMove] Failed to set pose target. Please retry.");
      return;  // return if setPoseTarget failed
    }
    else
      ROS_INFO("[move_robot/requestMove] Using POSE TARGET for planning and/or "
               "moving.");
  }

  ///////////////
  // Joint target
  ///////////////
  else if (joint_deltas_.size() != 0)  // The Goal msg had a joint target
  {
    // Get the current joints
    std::vector<double> joints = movegroup_.getCurrentJointValues();

    // Add the deltas (another std::vector<double>)
    for (int i = 0; i < joint_deltas_.size(); i++)
      joints.at(i) += joint_deltas_.at(i);

    // Move to the new target joints
    movegroup_.setJointValueTarget(joints);
    movegroup_.move();

    return;
  }

  // Continue with MoveGroup stuff for Cartesian moves

  // Set goal tolerance based on the actual shift from current position to
  // target position.
  // Apply minimums, e.g. we can't control motion down to the nanometer level.
  geometry_msgs::PoseStamped current_pose_stamped = movegroup_.getCurrentPose();
  calculate_linear_tols(current_pose_stamped, target_pose_stamped_);
  calculate_ang_tols(current_pose_stamped, target_pose_stamped_);

  // Just checking what is the target pose
  geometry_msgs::PoseStamped current_target = movegroup_.getPoseTarget();

  // Based on action type: PLAN, EXECUTE PLAN, or PLAN&EXECUTE (aka GO)
  if (req_action_type_ == low_level_cmds::PLAN)
  {
    movegroup_.plan(latest_plan_);  // Calculate plan and store it in latest_plan_.
    new_plan_available_ = true;     // Set new_plan_available_ to TRUE.
  }
  else if (req_action_type_ == low_level_cmds::EXECUTE)
  {
    // ROS_INFO("[move_robot/requestMove] Starting to execute last plan ...");
    if (new_plan_available_)
      movegroup_.execute(latest_plan_);  // If there is a new plan, execute latest_plan_.
    else
      ROS_INFO("[move_robot/requestMove] No plan to execute.");  // Else do
                                                                 // nothing but
                                                                 // printout "no
                                                                 // plan"
    new_plan_available_ = false;                                 // Either case, set new_plan_available_ to FALSE.
  }
  else if (req_action_type_ == low_level_cmds::GO)
  {
    // Since move() has a bug of start state not being current state, I am going
    // to plan and execute sequentally.
    moveit::planning_interface::MoveGroupInterface::Plan move_plan;
    movegroup_.plan(move_plan);
    movegroup_.execute(move_plan);
    new_plan_available_ = false;  // As any previous plan has become invalid, set
                                  // new_plan_available_ to FALSE.
  }

  return;
}  // end requestMove

void MoveRobotInterface::calculate_linear_tols(geometry_msgs::PoseStamped curr_pose,
                                               geometry_msgs::PoseStamped target_pose)
{
  // double sum_of_squares = pow( curr_pose.pose.position.x -
  // target_pose.pose.position.x,2) + pow( curr_pose.pose.position.y -
  // target_pose.pose.position.y,2) + pow( curr_pose.pose.position.z -
  // target_pose.pose.position.z,2);

  // double delta_position = pow( sum_of_squares, 0.5);

  // delta_position *= fractional_tolerance_;

  // if ( delta_position < min_position_tolerance_ )
  //  delta_position = min_position_tolerance_;

  // movegroup_.setGoalPositionTolerance( delta_position );

  // ^Dynamically adjusting tol's wasn't helping. Back to basics.
  movegroup_.setGoalPositionTolerance(min_position_tolerance_);

  return;
}

void MoveRobotInterface::calculate_ang_tols(geometry_msgs::PoseStamped curr_pose,
                                            geometry_msgs::PoseStamped target_pose)
{
  // double sum_of_squares = pow( curr_pose.pose.orientation.x -
  // target_pose.pose.orientation.x,2) + pow( curr_pose.pose.orientation.y -
  // target_pose.pose.orientation.y,2) + pow( curr_pose.pose.orientation.z -
  // target_pose.pose.orientation.z,2) + pow( curr_pose.pose.orientation.w -
  // target_pose.pose.orientation.w,2);

  // double delta_orientation = pow( sum_of_squares, 0.5);

  // delta_orientation *= fractional_tolerance_;

  // if ( delta_orientation < min_orientation_tolerance_ )
  //  delta_orientation = min_orientation_tolerance_;

  // movegroup_.setGoalOrientationTolerance( delta_orientation );

  // ^Dynamically adjusting tol's wasn't helping. Back to basics.
  movegroup_.setGoalOrientationTolerance(min_orientation_tolerance_);

  return;
}