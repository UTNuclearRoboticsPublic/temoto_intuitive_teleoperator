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

/** @file rviz_visual.h
 *
 *  @brief Handles visual markers and point-of-view camera in RViz for
 *         temoto teleoperation.
 *
 *  @author karl.kruusamae(at)utexas.edu
 */

// ROS includes
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "temoto/get_ros_params.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "view_controller_msgs/CameraPlacement.h"

#ifndef RVIZ_VISUAL_H
#define RVIZ_VISUAL_H

struct temoto_status
{
  bool in_navigation_mode;
  geometry_msgs::PoseStamped commanded_pose;
  double scale_by;
  geometry_msgs::PoseStamped end_effector_pose;
  uint current_movegroup_ee_index = 0;
  std::string moveit_planning_frame;
};

class Visuals
{
public:
  // ___ CONSTRUCTOR ___
  Visuals()
  {
    // Get the STL's for the manip/nav hand markers from launch file, if any
    // First, how many ee's are there?
    int num_ee = 1;
    if (!nh_.getParam("/temoto/num_ee", num_ee))
      ROS_ERROR("[start_teleop/Teleoperator] num_ee was not specified in yaml. "
                "Aborting.");

    for (int i = 0; i < num_ee; i++)
    {
      std::string stl_name =
          get_ros_params::getStringParam("/temoto/ee/ee" + std::to_string(i) + "/manip_stl", nh_);
      manip_stl_names_.push_back(stl_name);
    }

    // Get all the relevant frame names from parameter server
    nh_.param<std::string>("/temoto/base_frame", base_frame_, "base_link");

    // Initialize point-of-view camera placement and all the required markers
    initCameraFrames();
    initHandPoseMarker();

    adjust_camera_ = true;

    // Publishers
    pub_update_rviz_goal_ =
        nh_.advertise<geometry_msgs::PoseStamped>("/rviz/moveit/move_marker/goal_right_ur5_ee_link", 1);
    pub_rviz_marker_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    pub_pov_camera_ = nh_.advertise<view_controller_msgs::CameraPlacement>("/rviz/camera_placement", 3, true);
  };

  // Updates RViz point-of-view and visualization markers as needed.
  // Return true if successful
  bool crunch();

  // Initializes camera placement to a preset pose in frame specified by
  // frame_id
  void initCameraFrames();

  // ROS camera placement message that defines user's point-of-view in RViz.
  view_controller_msgs::CameraPlacement point_of_view_;

  // Latest recieved full system status published by start_teleop node.
  temoto_status latest_status_;

  // This is set TRUE, if it is needed to adjust the position of the
  // point-of-view (POW) camera in RViz.
  bool adjust_camera_;

private:
  ros::NodeHandle nh_;

  // ___ INITIALIZERS ___

  /** Creates the initial marker that visualizes hand pose as a flattened box.
   */
  void initHandPoseMarker();

  // ___ CLASS VARIABLES AND CONSTANTS ___

  // A list of end-effector CAD representations
  std::vector<std::string> manip_stl_names_;

  // Previous status. Used in checking if control mode has changed (e.g. nav to
  // manip)
  temoto_status prev_status_;

  // Navigation control frame.
  std::string base_frame_;

  // Arrow-shaped marker that visualizes target displacement of the robot.
  visualization_msgs::Marker displacement_arrow_;

  // Text marker for displaying the length of displacement represented by
  // displacement_arrow_.
  visualization_msgs::Marker distance_as_text_;

  // Flattened-box-shaped marker that represents the position and orientation of
  // the operator's primary hand.
  visualization_msgs::Marker cmd_pose_marker_;

  // Translucent box or area that is centered around the starting pose while
  // displacement_arrow_ always points to one of its corners.
  visualization_msgs::Marker active_range_box_;

  // A number that is used when some distance is required between robot and any
  // marker or camera.
  const double EYE_DISPLACEMENT_FRONT_ = 1.;

  // A number that is used when some distance is required between robot and any
  // marker or camera.
  const double EYE_DISPLACEMENT_TOP_ = 1.;

  // Show the hand marker frame in RViz
  tf2_ros::TransformBroadcaster tf_br_;
  tf2::Transform command_frame_tf_;

  // Publisher to update the goal state in rviz motion planning plugin
  // Need to enable external comms in MoveIt for this to work
  ros::Publisher pub_update_rviz_goal_;

  // Publish visual markers to RViz
  ros::Publisher pub_rviz_marker_;

  // Publisher of CameraPlacement messages (this is picked up by
  // rviz_animated_view_controller).
  // When latch is true, the last message published is saved and automatically
  // sent to any future subscribers that connect. Using it to set camera during
  // rviz startup.
  ros::Publisher pub_pov_camera_;
};

#endif