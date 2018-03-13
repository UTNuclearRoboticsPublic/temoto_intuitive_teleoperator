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
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

// temoto includes
#include "temoto/graphics_and_frames.h"
#include "temoto/interpret_utterance.h"
#include "temoto/navigate_robot.h"
#include "temoto/preplanned_sequences.h"
#include "temoto/PreplannedSequenceAction.h"  // Define an action. This is how a preplanned sequence gets triggered
#include "temoto/low_level_cmds.h"
#include "temoto/temoto_common.h"
#include "griffin_powermate/PowermateEvent.h"

// Other includes
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
  
  temoto::Status setStatus();

  void setScale();
  
  //Callback functions

  void processJoyCmd(sensor_msgs::Joy pose_cmd);
  
  void processPowermate(griffin_powermate::PowermateEvent powermate);  // TODO rename to more general case, e.g. processScaleFactor
  
  void updateEndEffectorPose(geometry_msgs::PoseStamped end_effector_pose);

  void updatePreplannedFlag(temoto::PreplannedSequenceActionResult sequence_result);
  
  void processVoiceCommand(temoto::Command voice_command);

  void triggerSequence(temoto::Command& voice_command);

  void nav_collision_cb(const std_msgs::Float64::ConstPtr& msg);
  
  // Public members
  ros::ServiceClient move_robot_client_;		///< Service client for temoto/move_robot_service is global.
  bool manipulate_ = true;		/// Is manipulation enabled?
  std::string temoto_pose_cmd_topic_;   /// Topic of incoming pose cmds
  bool in_jog_mode_ = false;			///< If true, send new joints/poses immediately. Otehrwise, pt-to-pt motion
  bool navT_or_manipF_ = false;		///< TRUE: interpret absolute_pose_cmd_ as 2D navigation goal; FALSE: absolute_pose_cmd_ is the motion planning target for robot EEF.
  bool executing_preplanned_sequence_ = false;  ///< TRUE blocks other Temoto cmds
  Visuals graphics_and_frames_;  // Publish markers to RViz and adjust cmd frame

private:
  // Other Temoto classes (each encapsulating its own functionality)
  Interpreter interpreter;  // Interpret voice commands
  preplanned_sequence sequence_;  // Process the cmds that trigger short, predefined actions, e.g. open gripper
  NavigateRobotInterface navigateIF_;  // Send motion commands to the base

  /// Local TransformListener for transforming poses
  tf::TransformListener transform_listener_;

  /// Scaling factor
  double pos_scale_;
  double rot_scale_;
  
  /// Amplification of input hand motion. (Scaling factor scales the amplification.)
  int8_t AMP_HAND_MOTION_;

  /// Latest pose value received for the end effector.
  geometry_msgs::PoseStamped current_pose_;
  
  /// For absolute pose commands
  geometry_msgs::PoseStamped absolute_pose_cmd_;
  // For incremental pose commands, as from the SpaceMouse. These need to be integrated before adding to absolute_pose_cmd_
  geometry_msgs::Vector3 incremental_position_cmd_;
  geometry_msgs::Vector3 incremental_orientation_cmd_;

  // The jogger takes TwistStamped msgs
  geometry_msgs::TwistStamped jog_twist_cmd_;

  // ~*~ VARIABLES DESCRIBING THE STATE ~*~
  // NATURAL control: robot and human are oriented the same way, i.e., the first person perspective
  // INVERTED control: the human operator is facing the robot so that left and right are inverted.
  bool naturalT_or_invertedF_control_ = true;	///< Mode of intepration for hand motion: 'true' - natural, i.e., human and robot arms are the same; 'false' - inverted.
  bool reset_integrated_cmds_ = false;		///< TRUE ==> reset the integration of incremental (e.g. SpaceNav cmds). Typically set to true when switching between nav/manip modes.

  // ROS publishers
  ros::Publisher pub_abort_, pub_jog_arm_cmds_, pub_jog_base_cmds_;

  // ROS subscribers
  ros::Subscriber sub_pose_cmd_, sub_voice_commands_, sub_executing_preplanned_, sub_end_effector_, sub_scaling_factor_;

  // Scale speed cmds when near obstacles
  ros::Subscriber sub_nav_spd_;
  double nav_speed_fraction_ = 1.;

  // ROS services/actions
  actionlib::SimpleActionClient<temoto::PreplannedSequenceAction> preplanned_sequence_client_;  // Used to trigger a preplanned sequence
};
#endif
