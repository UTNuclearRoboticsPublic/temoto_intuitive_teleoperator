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

/** @file graphics_and_frames.cpp
 *
 *  @brief Node that handles RViz graphics and tf frames for the controllers.
 * It's useful to do
 *  both in the same place because both graphics and tf frames depend on knowing
 * the hand pose.
 *
 *  @author karl.kruusamae(at)utexas.edu
 */

#include "temoto/graphics_and_frames.h"

/** Creates the initial CameraPlacment message that is used for positioning
 * point-of-view (POV) camera in RViz.
 */
void Visuals::initCameraFrame()
{
  // Set target frame and animation time
  point_of_view_.target_frame = base_frame_;  // Use base_frame_ because it does
                                              // not move with the end effector
                                              // ==> will remain upright
  point_of_view_.time_from_start = ros::Duration(0.5);

  // Position of the camera relative to target_frame
  point_of_view_.eye.header.frame_id = base_frame_;

  // Target_frame-relative point for the focus
  point_of_view_.focus.header.frame_id = base_frame_;

  // Target_frame-relative vector that maps to "up" in the view plane.
  point_of_view_.up.header.frame_id = base_frame_;
}

/** Creates the initial marker that visualizes hand pose */
void Visuals::initHandPoseMarker()
{
  cmd_pose_marker_.header.frame_id = latest_status_.commanded_pose.header.frame_id;
  cmd_pose_marker_.header.stamp = ros::Time();
  cmd_pose_marker_.ns = "hand_pose_marker";
  cmd_pose_marker_.id = 0;

  cmd_pose_marker_.pose = latest_status_.commanded_pose.pose;

  cmd_pose_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  cmd_pose_marker_.mesh_resource = manip_stl_names_.at(latest_status_.current_movegroup_ee_index);
  cmd_pose_marker_.action = visualization_msgs::Marker::ADD;
  cmd_pose_marker_.scale.x = 1;
  cmd_pose_marker_.scale.y = 1;
  cmd_pose_marker_.scale.z = 1;

  // Orange.
  cmd_pose_marker_.color.r = 1.0;
  cmd_pose_marker_.color.g = 0.5;
  cmd_pose_marker_.color.b = 0.0;
  cmd_pose_marker_.color.a = 1;

  return;
}

bool Visuals::crunch()
{
  // ============================================================
  // ==  CHECK FOR REASONABLE EE POSE  ==========================
  // ============================================================

  // An uninitialized w-component of quaternion is a good bet that the pose was
  // uninitialized
  if (latest_status_.end_effector_pose.pose.orientation.w == 0.)
  {
    ROS_WARN_THROTTLE(5, "[rviz_visual] Waiting for the initial end effector pose.");
    ROS_WARN_THROTTLE(5, "[rviz_visual] For distributed systems, are your clocks synched?");
    return false;
  }

  // ============================================================
  // ==  CAMERA POSE  ===========================================
  // ============================================================
  // Adjust the camera if mode has changed from manipulation to navigation
  if (latest_status_.in_navigation_mode != prev_status_.in_navigation_mode)
    adjust_camera_ = true;

  // Adjust camera in NAVIGATION mode
  if (latest_status_.in_navigation_mode && adjust_camera_)
  {
    // For NAVIGATION/NATURAL +X is considered to be 'UP'.
    point_of_view_.up.vector.x = 1;
    point_of_view_.up.vector.z = 0;

    // Camera is positioned directly above the origin of nav frame (usually
    // base_link)
    point_of_view_.eye.point.x = 0;
    point_of_view_.eye.point.y = 0;
    point_of_view_.eye.point.z = 12. + 10. * latest_status_.scale_by;  // Never closer than 2 m, max distance at 12 m

    // Focus camera at the origin
    point_of_view_.focus.point.x = 0;
    point_of_view_.focus.point.y = 0;

    pub_pov_camera_.publish(point_of_view_);  // publish the modified CameraPlacement message
    adjust_camera_ = false;                   // set adjust_camera 'false'
  }
  // Adjust camera in MANIPULATION mode
  else if (!latest_status_.in_navigation_mode && adjust_camera_)
  {
    point_of_view_.up.vector.x = 0;
    point_of_view_.up.vector.z = 1;

    // Camera will be behind end effector, somewhat elevated
    point_of_view_.eye.point.x = latest_status_.end_effector_pose.pose.position.x -
                                 EYE_DISPLACEMENT_FRONT_;  // Distance backwards from the end effector
    point_of_view_.eye.point.y = latest_status_.end_effector_pose.pose.position.y;  // Align with end effector
    point_of_view_.eye.point.z = latest_status_.end_effector_pose.pose.position.z +
                                 EYE_DISPLACEMENT_TOP_;  // Distance upwards from the end effector

    // Look at the distance of VIRTUAL_VIEW_SCREEN from the origin of end
    // effector frame, i.e. the palm of robotiq gripper
    point_of_view_.focus.point = latest_status_.end_effector_pose.pose.position;

    pub_pov_camera_.publish(point_of_view_);  // publish a CameraPlacement msg
    adjust_camera_ = false;                   // set adjust_camera 'false'

  }  // else if (!latest_status.in_navigation_mode && adjust_camera)

  // ============================================================
  // ==  VISUALIZATION MARKERS  =================================
  // ============================================================

  // Setting markers & frames in NAVIGATION mode
  // In Nav mode, set temoto_command_frame coincident with the base frame
  if (latest_status_.in_navigation_mode)
  {
    // In Nav mode, set temoto_command_frame coincident with the base frame
    tf2::Vector3 base_origin(0, 0, 0);
    tf2::Quaternion base_rotation(0, 0, 0, 1);
    command_frame_tf_.setRotation(base_rotation);
    command_frame_tf_.setOrigin(base_origin);

    geometry_msgs::TransformStamped command_frame_tf_msg;
    command_frame_tf_msg.header.stamp = ros::Time::now();
    command_frame_tf_msg.header.frame_id = base_frame_;
    command_frame_tf_msg.child_frame_id = "temoto_command_frame";
    command_frame_tf_msg.transform = toMsg(command_frame_tf_);
    tf_br_.sendTransform(command_frame_tf_msg);

    // ==  HAND POSE BOX MARKER  ============================= //
    // Resize of the hand pose marker to robot base dimensions
    cmd_pose_marker_.type = visualization_msgs::Marker::CUBE;
    cmd_pose_marker_.scale.x = 1.0;
    cmd_pose_marker_.scale.y = 0.5;
    cmd_pose_marker_.scale.z = 0.1;

    cmd_pose_marker_.pose = latest_status_.commanded_pose.pose;

    pub_rviz_marker_.publish(cmd_pose_marker_);
  }
  // Setting markers & frames in MANIPULATION mode
  // In Manip mode, set temoto_command_frame coincident with the base frame
  else
  {
    // ==  HAND POSE MARKER  ================================= //
    cmd_pose_marker_.header.stamp = ros::Time();
    cmd_pose_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    cmd_pose_marker_.mesh_resource = manip_stl_names_.at(latest_status_.current_movegroup_ee_index);
    cmd_pose_marker_.scale.x = 1;
    cmd_pose_marker_.scale.y = 1;
    cmd_pose_marker_.scale.z = 1;

    // Hand pose marker
    cmd_pose_marker_.header = latest_status_.commanded_pose.header;
    cmd_pose_marker_.pose = latest_status_.commanded_pose.pose;

    pub_rviz_marker_.publish(cmd_pose_marker_);

    tf2::Vector3 spacenav_origin(cmd_pose_marker_.pose.position.x, cmd_pose_marker_.pose.position.y,
                                 cmd_pose_marker_.pose.position.z);
    tf2::Quaternion spacenav_rotation(cmd_pose_marker_.pose.orientation.x, cmd_pose_marker_.pose.orientation.y,
                                      cmd_pose_marker_.pose.orientation.z, cmd_pose_marker_.pose.orientation.w);
    command_frame_tf_.setRotation(spacenav_rotation);
    command_frame_tf_.setOrigin(spacenav_origin);

    geometry_msgs::TransformStamped command_frame_tf_msg;
    command_frame_tf_msg.header.stamp = ros::Time::now();
    command_frame_tf_msg.header.frame_id = latest_status_.moveit_planning_frame;
    command_frame_tf_msg.child_frame_id = "temoto_command_frame";
    command_frame_tf_msg.transform = toMsg(command_frame_tf_);
    tf_br_.sendTransform(command_frame_tf_msg);

    // update the goal position in moveit plugin to display real-time IK
    // Need to enable external comms in MoveIt's RViz plugin for this to work
    geometry_msgs::PoseStamped move_goal_msg;
    move_goal_msg.header.frame_id = latest_status_.moveit_planning_frame;
    move_goal_msg.header.stamp = ros::Time::now();
    move_goal_msg.pose.position.x = command_frame_tf_.getOrigin().x();
    move_goal_msg.pose.position.y = command_frame_tf_.getOrigin().y();
    move_goal_msg.pose.position.z = command_frame_tf_.getOrigin().z();
    move_goal_msg.pose.orientation = tf2::toMsg(command_frame_tf_.getRotation());
    pub_update_rviz_goal_.publish(move_goal_msg);
  }  // end setting markers in MANIPULATION mode

  prev_status_ = latest_status_;

  return true;
}  // end Visuals::crunch()
