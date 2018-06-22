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

/** @file move_robot.h
 *
 *  @brief ROS server that acts as an interface for ROS MoveIt! and publishes
 * end-effector pose.
 *
 *  @author karl.kruusamae(at)utexas.edu
 */

// ROS includes
#include "moveit/move_group_interface/move_group_interface.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// other includes
#include "string.h"

#ifndef MOVE_ROBOT_H
#define MOVE_ROBOT_H

class MoveRobotInterface
{
public:
  MoveRobotInterface(std::string mg_name) : movegroup_(mg_name)
  {
    movegroup_.setPlannerId("RRTConnectkConfigDefault" /*"RRTstarkConfigDefault"*/);
    movegroup_.setMaxVelocityScalingFactor(0.5);
    movegroup_.setPlanningTime(1.9);
  };

  moveit::planning_interface::MoveGroupInterface movegroup_;

  void requestMove();

  void requestCartesianMove();

  void calculate_linear_tols(geometry_msgs::PoseStamped curr_pose, geometry_msgs::PoseStamped target_pose);

  void calculate_ang_tols(geometry_msgs::PoseStamped curr_pose, geometry_msgs::PoseStamped target_pose);

  geometry_msgs::PoseStamped target_pose_stamped_;                    ///< Target pose for the robot, if applicable.
  std::vector<double> joint_deltas_;                                  ///< Joint target, if applicable.
  std::string named_target_;                                          ///< Named target for the robot.
  moveit::planning_interface::MoveGroupInterface::Plan latest_plan_;  ///< Latest motion plan.
  std::string req_action_type_ = "";                                  ///< Action type associated with target
                                                                      /// request, i.e. PLAN, EXECUTE PLAN, or
  /// PLAN&EXECUTE.

  // Public variables describing the state of MoveRobotInterface
  bool new_plan_available_ = false;  ///< After calculating a new motion plan,
                                     /// is_new_plan is set to 1; after executing
  /// the plan, is_new_plan is set to 0.
  bool new_move_requested_ = false;  ///< If new move has been requested by a
                                     /// client, it is set to 1; after calling
  /// move(), it is set to 0.
  double fractional_tolerance_ = 0.1;  ///< Allow a small % error for motion
                                       /// planning. % of the distance from
  /// current to target
  double min_position_tolerance_ = 0.001;    ///< Don't ask for tighter position tolerances than this.
  double min_orientation_tolerance_ = 0.01;  ///< Don't ask for tighter orientation tolerances than this.
};

#endif
