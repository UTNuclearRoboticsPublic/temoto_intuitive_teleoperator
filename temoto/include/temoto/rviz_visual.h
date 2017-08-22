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

/** @file rviz_visual.h
 * 
 *  @brief Node that handles visual markers and point-of-view camera in RViz for
 *         temoto teleoperation.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

// ROS includes
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "std_srvs/Empty.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "view_controller_msgs/CameraPlacement.h"

// temoto includes
#include "temoto/temoto_common.h"
#include "griffin_powermate/PowermateEvent.h"

#ifndef RVIZ_VISUAL_H
#define RVIZ_VISUAL_H

class Visuals
{
public:
  // ___ CONSTRUCTOR ___
  Visuals()
  {
    // NodeHandle for accessing private parameters
    ros::NodeHandle pn("~");

    // Get the STL's for the manip/nav hand markers from launch file, if any
    std::string manip_stl;
    pn.param<std::string>("manip_stl", manip_stl_, "");

    // Get all the relevant frame names from parameter server
    pn.param<std::string>("/temoto_frames/human_input", human_frame_, "current_cmd_frame");
    pn.param<std::string>("/temoto_frames/end_effector", eef_frame_, "temoto_end_effector");
    pn.param<std::string>("/temoto_frames/base_frame", base_frame_, "base_link");

    // Initialize point-of-view camera placement and all the required markers
    initCameraFrames();
    initDisplacementArrow();
    initDistanceAsText();
    initHandPoseMarker();
    initActiveRangeBox();
    
    // Initial state setup
    adjust_camera_ = true;
    camera_is_aligned_ = true;
    latest_known_camera_mode_ = 0;
  };
  
  /** Callback function for /temoto/status. Looks for changes that require setting adjust_camera_ TRUE. */
  void updateStatus (temoto::Status status);
  
  /** Callback function for /griffin_powermate/events. Sets adjust_camera_ TRUE. */
  void powermateWheelEvent (griffin_powermate::PowermateEvent powermate);
  
  /** Updates RViz point-of-view and visualization markers as needed. */
  void crunch(ros::Publisher &marker_publisher, ros::Publisher &pow_publisher);

  /** Initializes camera placement to a preset pose in frame specified by frame_id */
  void initCameraFrames();
  
  /** ROS camera placement message that defines user's point-of-view in RViz. */
  view_controller_msgs::CameraPlacement point_of_view_;

  /** Latest recieved full system status published by start_teleop node. */
  temoto::Status latest_status_;

private:
  // ___ INITIALIZERS ___
  
  /** Creates the initial marker that visualizes hand movement as a displacement arrow. */
  void initDisplacementArrow();
  
  /** Creates the initial marker that displays front-facing text. */
  void initDistanceAsText();

  /** Creates the initial marker that visualizes hand pose as a flattened box. */
  void initHandPoseMarker();
  
  /** Creates the initial marker for an active range box around the robot where target position is always in one of the corners. */
  void initActiveRangeBox();
  
  // ___ HELPER FUNCTIONS ___
  
  /** Calculates the distance between two points in meters or millimeters and returns it as a string. */
  std::string getDistanceString (std::vector <geometry_msgs::Point> & twoPointVector);

  // ___ CLASS VARIABLES AND CONSTANTS ___

  /** STL files for the hand markers. */
  std::string manip_stl_;;
  
  /** Human input frame. */
  std::string human_frame_;
  
  /** Motion-planning control frame. */
  std::string eef_frame_;
  
  /** Navigation control frame. */
  std::string base_frame_;

  /** This is set TRUE, if it is needed to adjust the position of the point-of-view (POW) camera in RViz.*/
  bool adjust_camera_;

  /** Camera is algined with end effector (i.e., TRUE) as opposed to being in top-down point-of-view (i.e., FALSE).*/
  bool camera_is_aligned_;
  
  /** A state variable for keeping track of where rviz_visual node thinks the camera is. */
  uint8_t latest_known_camera_mode_; 	// 0-unknown, 1-natural, 2-inverted, 11-navigation natural, 12-navigation inverted
  
  /** Arrow-shaped marker that visualizes target displacement of the robot. */
  visualization_msgs::Marker displacement_arrow_;
  
  /** Text marker for displaying the length of displacement represented by displacement_arrow_. */
  visualization_msgs::Marker distance_as_text_;
  
  /** Flattened-box-shaped marker that represents the position and orientation of the operator's primary hand. */
  visualization_msgs::Marker cmd_pose_marker_;
  
  /** Translucent box or area that is centered around the starting pose while displacement_arrow_ always points to one of its corners. */
  visualization_msgs::Marker active_range_box_;

  /** A number that is used when some distance is required between robot and any marker or camera. */
  const double EYE_DISPLACEMENT_FRONT_ = 1.;

  /** A number that is used when some distance is required between robot and any marker or camera. */
  const double EYE_DISPLACEMENT_TOP_ = 1.;

  /** Show the hand marker frame in RViz **/
  tf::TransformBroadcaster tf_br_;
  tf::Transform hand_marker_tf_;

  /** ROS transform listener **/
  tf::TransformListener tf_listener_;
};

#endif
