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

/** @file start_teleop.h
 *
 *  @brief Central node for TEMOTO teleoperator. Subscribes to relevant
 * messages, calls move and navigation interfaces, and publishes system status.
 *
 *  @author karl.kruusamae(at)utexas.edu
 */

// ROS includes
#include "geometry_msgs/PoseStamped.h"
#include "griffin_powermate/PowermateEvent.h"
#include "control_msgs/JointJog.h"
#include "map"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sound_play/sound_play.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

// temoto includes
#include "temoto/get_ros_params.h"
#include "temoto/graphics_and_frames.h"
#include "temoto/grippers.h"
#include "temoto/temoto_commands.h"
#include "temoto/move_robot.h"
#include "temoto/navigate_robot.h"

#ifndef START_TELEOP_H
#define START_TELEOP_H

class Teleoperator
{
public:
  Teleoperator();

  bool callRobotMotionInterface(std::string action_type);

  void setGraphicsFramesStatus(bool adjust_camera);

  Visuals graphics_and_frames_;  // Publish markers to RViz and adjust cmd frame

  bool temoto_sleep_ = false;  // If set to true, temoto will spin without sending commands.

  /*
    track the control state of the robot
  */
  enum control_mode_t
  {
    JOG,
    POINT_TO_POINT
  };
  control_mode_t cur_control_mode_ = POINT_TO_POINT;

  /*
    navigation: interpret absolute_pose_cmd_ as 2D navigation goal;
    manipulation: absolute_pose_cmd_ is the motion planning target for robot EEF.
  */
  enum teleop_mode_t
  {
    NAVIGATION,
    MANIPULATION
  };
  teleop_mode_t cur_teleop_mode_ = MANIPULATION;

private:
  void powermateCallback(griffin_powermate::PowermateEvent powermate);

  void processStringCommand(std_msgs::String voice_command);

  void navCollisionCallback(const std_msgs::Float64::ConstPtr& msg);

  void sleepCallback(const std_msgs::Bool::ConstPtr& msg);

  bool calculateTransform(std::string source_frame, std::string target_frame,
                          geometry_msgs::TransformStamped& transform);

  void initializeGraphics();

  void resetEEGraphicPose();

  void setScale();

  void switchEE();

  void toggleCompliance(std_srvs::Trigger::Request &req,
                        std_srvs::Trigger::Response &res);

  void spaceNavCallback(sensor_msgs::Joy pose_cmd);

  void xboxCallback(sensor_msgs::Joy pose_cmd);

  void processIncrementalPoseCmd(double x_pos, double y_pos, double z_pos, double x_ori, double y_ori, double z_ori);

  bool enable_navigation_ = true;               // Is navigation enabled?
  bool enable_manipulation_ = true;             // Is manipulation enabled?
  std::string temoto_spacenav_pose_cmd_topic_;  // Topic of incoming pose cmds
  std::string temoto_xbox_pose_cmd_topic_;      // The incoming xbox pose cmds
  std::string base_frame_ = "base_link";        // Frame of robot base

  // Toggle between these camera topics. "transparent" hides the overlay
  std::vector<std::string> image_topics_;

  std::size_t current_image_topic_index_ = 0;

  int current_movegroup_ee_index_ = 0;  // What is the active movegroup/ee pair?

  ros::NodeHandle n_;

  // Other Temoto classes (each encapsulating its own functionality)
  NavigateRobotInterface nav_interface_;  // Send motion commands to the base

  // Instance of SoundClient used for text-to-speech synthesis
  sound_play::SoundClient sound_client_;

  // Scaling factor
  double pos_scale_;
  double rot_scale_;

  // Scaling factor maxs
  double pos_scale_max_, rot_scale_max_;

  // For absolute pose commands
  geometry_msgs::PoseStamped absolute_pose_cmd_;
  // For incremental pose commands, as from the SpaceMouse. These need to be
  // integrated before adding to absolute_pose_cmd_
  geometry_msgs::Vector3Stamped incremental_position_cmd_;
  geometry_msgs::Vector3Stamped incremental_orientation_cmd_;

  // TwistStamped msgs for Cartesian jogging
  geometry_msgs::TwistStamped jog_twist_cmd_, nav_twist_cmd_;

  // For joint jogging
  control_msgs::JointJog joint_jog_cmd_;

  // ROS publishers
  ros::Publisher pub_abort_, pub_jog_base_cmds_, pub_current_image_topic_;

  // ROS subscribers
  ros::Subscriber sub_spacenav_pose_cmd_, sub_xbox_pose_cmd_, sub_voice_commands_, sub_scaling_factor_,
      sub_temoto_sleep_;

  // Debounce buttons
  ros::Duration button_debounce_timeout_;  // Seconds
  ros::Time most_recent_button_press_;

  // Scale speed cmds when near obstacles
  ros::Subscriber sub_nav_spd_;
  double nav_speed_fraction_ = 1.;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Vectors of data for each end-effector. These params are read from the config file.
  struct endEffectorParameters
  {
    // All move_groups/ee's the user might want to control (specified in yaml)
    std::vector<std::string> ee_names;

    // Jogging publishers for each end-effector. Ptr needed because the MoveGroup
    // name is determined at run time
    std::vector<std::shared_ptr<ros::Publisher>> jog_publishers, joint_jog_publishers;

    // Wrist jogging requires the name of each wrist joint
    std::vector<std::string> wrist_joint_names;

    // Send motion commands to each end-effector. Ptr needed because the MoveGroup name is
    // determined at run time
    std::vector<std::unique_ptr<MoveRobotInterface>> arm_interface_ptrs;

    // A list of services to toggle compliance for each end-effector
    std::vector<std::shared_ptr<ros::ServiceClient>> toggle_compliance_services;

    // A list of services to bias the compliance wrench-to-joint-velocity publishers
    std::vector<std::shared_ptr<ros::ServiceClient>> bias_compliance_services;

    // A list of MoveIt "named targets" -- default home poses
    std::vector<std::string> home_pose_names;

    // An object which sends commands to the grippers
    std::vector<std::unique_ptr<grippers::Grippers>> gripper_interface_ptrs;

    // A list of gripper topic for each end-effector
    std::vector<std::string> gripper_topics;
  };
  endEffectorParameters end_effector_parameters_;
};
#endif
