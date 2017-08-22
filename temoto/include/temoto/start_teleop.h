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

/** @file start_teleop.h
 * 
 *  @brief Central node for TEMOTO teleoperator. Subscribes to relevant messages, calls move and navigation interfaces, and publishes system status.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

// ROS includes
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

// temoto includes
#include "temoto/PreplannedSequenceAction.h"  // Define an action. This is how a preplanned sequence gets triggered
#include "temoto/low_level_cmds.h"
#include "temoto/temoto_common.h"
#include "leap_motion_controller/Set.h"
#include "griffin_powermate/PowermateEvent.h"

// Other includes
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <string>

#ifndef START_TELEOP_H
#define START_TELEOP_H

class Teleoperator
{
public:
  // Constructor
  
  Teleoperator(ros::NodeHandle& n);
  
  // Callback functions
  
  void callRobotMotionInterface(std::string action_type);
  
  void callRobotMotionInterfaceWithNamedTarget(std::string action_type, std::string named_target);

  // Helper functions

  geometry_msgs::Quaternion extractOnlyPitch(geometry_msgs::Quaternion msg);
  
  geometry_msgs::Quaternion oneEightyAroundOperatorUp(geometry_msgs::Quaternion operator_input_quaternion_msg);
  
  temoto::Status setStatus();
  
  //Callback functions
  
  void processAbsolutePoseCmd(leap_motion_controller::Set leap_data);

  void processIncrementalPoseCmd(sensor_msgs::Joy pose_cmd);
  
  void processPowermate(griffin_powermate::PowermateEvent powermate);  // TODO rename to more general case, e.g. processScaleFactor
  
  void updateEndEffectorPose(geometry_msgs::PoseStamped end_effector_pose);

  void updatePreplannedFlag(temoto::PreplannedSequenceActionResult sequence_result);
  
  void processVoiceCommand(temoto::Command voice_command);

  void triggerROSAction(temoto::Command& voice_command);
  
  // Public members
  ros::ServiceClient move_robot_client_;		///< Service client for temoto/move_robot_service is global.
  ros::ServiceClient navigate_robot_client_;		///< Service client for temoto/navigate_robot_srv is global.
  ros::ServiceClient tf_change_client_;			///< Service client for requesting changes of control mode, i.e., change of orientation for current_cmd_frame frame.
  bool manipulate_ = true;		/// Is manipulation enabled?
  bool absolute_pose_input_ = true;	/// Specify whether incoming pose commands are absolute or relative
  std::string temoto_pose_cmd_topic_;   /// Topic of incoming pose cmds

private:
  /// Local TransformListener for transforming poses
  tf::TransformListener transform_listener_;

  /// Scaling factor
  double pos_scale_;
  double rot_scale_;
  
  /// Amplification of input hand motion. (Scaling factor scales the amplification.)
  int8_t AMP_HAND_MOTION_;
  
  /// Offsett zero position of the Leap Motion Controller.
  const double OFFSET_X_ = 0;
  const double OFFSET_Y_ = 0.2;		// Height of zero
  const double OFFSET_Z_ = 0;

  /// Latest pose value received for the end effector.
  geometry_msgs::PoseStamped current_pose_;
  
  /// For absolute pose commands, as from LeapMotion
  geometry_msgs::PoseStamped absolute_pose_cmd_;
  // For incremental pose commands, as from the SpaceMouse. These need to be integrated before adding to absolute_pose_cmd_
  geometry_msgs::Vector3 incremental_position_cmd_;
  geometry_msgs::Vector3 incremental_orientation_cmd_;

  // ~*~ VARIABLES DESCRIBING THE STATE ~*~
  // NATURAL control: robot and human are oriented the same way, i.e., the first person perspective
  // INVERTED control: the human operator is facing the robot so that left and right are inverted.
  bool using_natural_control_;		///< Mode of intepration for hand motion: 'true' - natural, i.e., human and robot arms are the same; 'false' - inverted.
  bool orientation_locked_;		///< Hand orientation info is to be ignored if TRUE.
  bool position_limited_;		///< Hand position is restricted to a specific direction/plane if TRUE.
  bool position_fwd_only_;		///< TRUE when hand position is restricted to back and forward motion. Is only relevant when position_limited is 'true'.
  bool secondary_hand_before_;		///< Presence of secondary hand during the previous iteration of Leap Motion's callback processAbsolutePoseCmd(..).
  bool navigate_to_goal_;		///< TRUE: interpret absolute_pose_cmd_ as 2D navigation goal; FALSE: absolute_pose_cmd_ is the motion planning target for robot EEF.
  bool primary_hand_is_left_;		///< TRUE unless user specified right hand as the primary hand.
  uint8_t control_state_;		///< 1 -> manipulate only; 2 -> navigate only; 3 -> manipulate&navigate
  bool executing_preplanned_sequence_ = false;

  // ROS publishers
  ros::Publisher pub_abort_;

  // ROS services/actions
  actionlib::SimpleActionClient<temoto::PreplannedSequenceAction> preplanned_sequence_client_;  // Used to trigger a preplanned sequence
};
#endif
