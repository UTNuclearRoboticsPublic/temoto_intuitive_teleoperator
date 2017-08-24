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

/** @file move_robot.h
 * 
 *  @brief ROS server that acts as an interface for ROS MoveIt! and publishes end-effector pose.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

// ROS includes
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "moveit/move_group_interface/move_group.h"
#include "ros/callback_queue.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// temoto includes
#include "temoto/temoto_common.h"

// other includes
#include "string.h"

#ifndef MOVE_ROBOT_H
#define MOVE_ROBOT_H

class MoveRobotInterface {
 public:
   MoveRobotInterface(std::string mg_name) :
    movegroup_(mg_name)
   {
     use_named_target_ = false;
     new_plan_available_ = false;
     req_action_type_ = "";
     new_move_requested_ = false;
     movegroup_.setPlannerId("RRTConnectkConfigDefault"/*"RRTstarkConfigDefault"*/);
   };
   
   moveit::planning_interface::MoveGroup movegroup_;
   
   /** Callback function */
   bool serviceUpdate(temoto::Goal::Request  &req, temoto::Goal::Response &res);

   void requestMove();

   void requestCartesianMove();

   void calculate_linear_tols(geometry_msgs::PoseStamped curr_pose, geometry_msgs::PoseStamped target_pose);

   void calculate_ang_tols(geometry_msgs::PoseStamped curr_pose, geometry_msgs::PoseStamped target_pose);

   geometry_msgs::PoseStamped target_pose_stamped_;		///< Target pose for the robot.
   std::string named_target_;					///< Named target for the robot.
   moveit::planning_interface::MoveGroup::Plan latest_plan_;	///< Latest motion plan.
   std::string req_action_type_;					///< Action type associated with target request, i.e. PLAN (0x01), EXECUTE PLAN (0x02), or PLAN&EXECUTE (0x03).
   
   // Public variables describing the state of MoveRobotInterface
   bool use_named_target_;		///< When named target is requested, use_named_target is set to true.
   bool new_plan_available_;		///< After calculating a new motion plan, is_new_plan is set to 1; after executing the plan, is_new_plan is set to 0.
   bool new_move_requested_; 		///< If new move has been requested by a client, it is set to 1; after calling move(), it is set to 0.
   double fractional_tolerance_ = 0.1;  ///< Allow a small % error for motion planning. % of the distance from current to target
   double min_position_tolerance_ = 0.001;  ///< Don't ask for tighter position tolerances than this.
   double min_orientation_tolerance_ = 0.01; ///< Don't ask for tighter orientation tolerances than this.
};

#endif
