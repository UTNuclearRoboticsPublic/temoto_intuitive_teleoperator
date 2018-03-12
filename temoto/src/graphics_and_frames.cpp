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

/** @file graphics_and_frames.cpp
 * 
 *  @brief Node that handles RViz graphics and tf frames for the controllers. It's useful to do
 *  both in the same place because both graphics and tf frames depend on knowing the hand pose.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

#include "temoto/graphics_and_frames.h"

/** Callback function for /temoto/status.
 *  Looks for any changes that would require re-adjustment of the point-of-view camera.
 *  Stores the received status in a class variable latest_status_.
 *  @param status temoto::Status message
 */
void Visuals::updateStatus (temoto::Status status)
{
  // Before overwriting previous status, checks if switch between camera views is necesassry due to switch between navigation and manipulation modes.
  if (status.in_navigation_mode != latest_status_.in_navigation_mode)
  {
    adjust_camera_ = true;
    //ROS_INFO("[rviz_visual/updateStatus] adjust_camera is set 'true' due change from MANIPULATION to NAVIGATION or vice versa.");
  }
  
  // Overwrite latest_status values with the new status.
  latest_status_ = status;

  return;
}

/** Creates the initial CameraPlacment message that is used for positioning point-of-view (POV) camera in RViz.
 */
void Visuals::initCameraFrames()
{
  // Set target frame and animation time
  point_of_view_.target_frame = base_frame_;  // Use base_frame_ because it does not move with the end effector ==> will remain upright
  point_of_view_.time_from_start = ros::Duration(0.5);

  // Position of the camera relative to target_frame
  point_of_view_.eye.header.frame_id = base_frame_;

  // Target_frame-relative point for the focus
  point_of_view_.focus.header.frame_id = base_frame_;

  // Target_frame-relative vector that maps to "up" in the view plane.
  point_of_view_.up.header.frame_id = base_frame_;
}

/** Creates the initial marker that visualizes hand movement as a displacement arrow. */
void Visuals::initDisplacementArrow()
{
  displacement_arrow_.header.frame_id = human_frame_; // x is horizontal, y is vertical, z is forward-backward // "temoto_end_effector"; // x is forward, y is horizontal, z is vertical //
  displacement_arrow_.header.stamp = ros::Time();
  displacement_arrow_.ns = "displacement_arrow";
  displacement_arrow_.id = 0;
  displacement_arrow_.type = visualization_msgs::Marker::ARROW;
  displacement_arrow_.action = visualization_msgs::Marker::ADD;

  // Defining start and end points for the arrow marker, the actual values don't matter here as they will be overwritten in the main
  geometry_msgs::Point start_point, end_point;
  start_point.x = 0.0;
  start_point.y = 0.0;
  start_point.z = 0.0;
  end_point.x = 0.0;
  end_point.y = -0.2;
  end_point.z = 0.0;

  // The point at index 0 is assumed to be the start point, and the point at index 1 is assumed to be the end. 
  displacement_arrow_.points.push_back(start_point);
  displacement_arrow_.points.push_back(end_point);

  // scale.x is the shaft diameter, and scale.y is the head diameter. If scale.z is not zero, it specifies the head length. 
  displacement_arrow_.scale.x = 0.001;
  displacement_arrow_.scale.y = 0.002;
  displacement_arrow_.scale.z = 0;
  
  // Make the arrow visible by setting alpha to 1.
  displacement_arrow_.color.a = 1.0;  
  // make the arrow orange
  displacement_arrow_.color.r = 1.0;
  displacement_arrow_.color.g = 0.5;
  displacement_arrow_.color.b = 0.0;
  
  return;
} // end Visuals::initDisplacementArrow()

/** Creates the initial marker that displays front-facing text. */
void Visuals::initDistanceAsText()
{
  distance_as_text_.header.frame_id = human_frame_;
  distance_as_text_.header.stamp = ros::Time();
  distance_as_text_.id = 0;
  distance_as_text_.ns = "distance_as_text";
  distance_as_text_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  distance_as_text_.action = visualization_msgs::Marker::ADD;
  distance_as_text_.text = "loading ...";
  
  // set the position of text; the actual values don't matter here as they will be overwritten in the main
  distance_as_text_.pose.position.x = 0.0;
  distance_as_text_.pose.position.y = 0.2;
  distance_as_text_.pose.position.z = 0.0;
  
  // Text height
  distance_as_text_.scale.z = 0.1;
  
  // Text visible & blue
  distance_as_text_.color.a = 1;
  distance_as_text_.color.b = 1;
  
  return;
} // end Visuals::initDistanceAsText()

/** Creates the initial marker that visualizes hand pose */
void Visuals::initHandPoseMarker()
{
  cmd_pose_marker_.header.frame_id = "base_link";
  cmd_pose_marker_.header.stamp = ros::Time();
  cmd_pose_marker_.ns = "hand_pose_marker";
  cmd_pose_marker_.id = 0;

  if ( manip_stl_ == "" )  // No end-effector CAD model was specified.
    manip_stl_ = "package://temoto_support/meshes/manip_cube.stl";

  cmd_pose_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  cmd_pose_marker_.mesh_resource = manip_stl_;
  cmd_pose_marker_.action = visualization_msgs::Marker::ADD;
  cmd_pose_marker_.scale.x = 0.001;
  cmd_pose_marker_.scale.y = 0.001;
  cmd_pose_marker_.scale.z = 0.001;

  // Orange.
  cmd_pose_marker_.color.r = 1.0;
  cmd_pose_marker_.color.g = 0.5;
  cmd_pose_marker_.color.b = 0.0;
  cmd_pose_marker_.color.a = 1;
  
  return;
} // end Visuals::initHandPoseMarker()

/** Creates the initial marker for an active range box around the robot where target position is always in one of the corners. */
void Visuals::initActiveRangeBox()
{
  active_range_box_.header.frame_id = human_frame_;
  active_range_box_.header.stamp = ros::Time();
  active_range_box_.ns = "active_range_box";
  active_range_box_.id = 0;
  active_range_box_.type = visualization_msgs::Marker::CUBE;
  active_range_box_.action = visualization_msgs::Marker::ADD;

  active_range_box_.scale.x = 0.2;	// side
  active_range_box_.scale.y = 0.2;	// thickness
  active_range_box_.scale.z = 0.2;	// depth
  
  // begin at the position of end effector
  active_range_box_.pose.position.x = 0.0;
  active_range_box_.pose.position.y = 0.0;
  active_range_box_.pose.position.z = 0.0;

  // Make the box barely visible by setting alpha to 0.2.
  active_range_box_.color.a = 0.2;  
  // Make the box red.
  active_range_box_.color.r = 1.0;
  active_range_box_.color.g = 0.0;
  active_range_box_.color.b = 0.0;
  
  return;
} // end Visuals::initActiveRangeBox()

void Visuals::crunch(ros::Publisher &marker_publisher, ros::Publisher &pov_publisher)
{
  // ============================================================ 
  // ==  VISUALIZATION MARKERS  =================================
  // ============================================================

  // Setting markers & frames in NAVIGATION mode
  if (latest_status_.in_navigation_mode)
  {
    
    // ==  HAND POSE BOX MARKER  ============================= //
    // Resize of the hand pose marker to robot base dimensions
    cmd_pose_marker_.type = visualization_msgs::Marker::CUBE;
    cmd_pose_marker_.scale.x = 1.0;  // Shaft diameter
    cmd_pose_marker_.scale.y = 0.5;
    cmd_pose_marker_.scale.z = 0.1;

    // Hand pose marker
    cmd_pose_marker_.pose = latest_status_.commanded_pose.pose;

    marker_publisher.publish( cmd_pose_marker_ );

  }
  // Setting markers & frames in MANIPULATION mode
  else
  {

    // ==  HAND POSE MARKER  ================================= //
    cmd_pose_marker_.header.stamp = ros::Time();
    cmd_pose_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    cmd_pose_marker_.mesh_resource = manip_stl_;
    cmd_pose_marker_.scale.x = 0.001;
    cmd_pose_marker_.scale.y = 0.001;
    cmd_pose_marker_.scale.z = 0.001;

    // Hand pose marker
    cmd_pose_marker_.header = latest_status_.commanded_pose.header;
    cmd_pose_marker_.pose = latest_status_.commanded_pose.pose;

    marker_publisher.publish( cmd_pose_marker_ );

    spacenav_tf_.setOrigin( tf::Vector3(cmd_pose_marker_.pose.position.x, cmd_pose_marker_.pose.position.y, cmd_pose_marker_.pose.position.z) );
    spacenav_tf_.setRotation(  tf::Quaternion(cmd_pose_marker_.pose.orientation.x, cmd_pose_marker_.pose.orientation.y, cmd_pose_marker_.pose.orientation.z, cmd_pose_marker_.pose.orientation.w)  );
    tf_br_.sendTransform(tf::StampedTransform(spacenav_tf_, ros::Time::now(), "base_link", "spacenav"));


	// update the goal position in moveit plugin to display real-time IK
    // Need to enable external comms in MoveIt for this to work
	geometry_msgs::PoseStamped move_goal_msg;
	move_goal_msg.header.frame_id = "base_link";
	move_goal_msg.header.stamp = ros::Time::now();
	move_goal_msg.pose.position.x = spacenav_tf_.getOrigin().x();
	move_goal_msg.pose.position.y = spacenav_tf_.getOrigin().y();
	move_goal_msg.pose.position.z = spacenav_tf_.getOrigin().z();
	quaternionTFToMsg( spacenav_tf_.getRotation(), move_goal_msg.pose.orientation );
    pub_update_rviz_goal_.publish(move_goal_msg);

  } // end setting markers in MANIPULATION mode
    

  // ============================================================ 
  // ==  CAMERA POSE  ===========================================
  // ============================================================
  // Setting 'adjust_camera_' triggers repositioning of the point-of-view (POV) camera.

  // Adjust camera in NAVIGATION mode
  if (latest_status_.in_navigation_mode && adjust_camera_)
  {
    // For NAVIGATION/NATURAL +X is considered to be 'UP'.
    point_of_view_.up.vector.x = 1;
    point_of_view_.up.vector.z = 0;

    // Camera is positioned directly above the origin of base_link
    point_of_view_.eye.point.x = 0;
    point_of_view_.eye.point.y = 0;
    point_of_view_.eye.point.z = 12. + 10.*latest_status_.scale_by;	// Never closer than 2 m, max distance at 12 m

    // Focus camera at the origin of base_link
    point_of_view_.focus.point.x = 0;
    point_of_view_.focus.point.y = 0;

    latest_known_camera_mode_ = 11;					// set latest_known_camera_mode to 11, i.e navigation natural
    pov_publisher.publish( point_of_view_ );				// publish the modified CameraPlacement message
    adjust_camera_ = false;						// set adjust_camera 'false'
  }
  // Adjust camera in MANIPULATION mode
  else if (!latest_status_.in_navigation_mode && adjust_camera_)
  {
	  if (camera_is_aligned_)					// here alignment means the so-called bird's eye view
	  {
			point_of_view_.up.vector.x = 0;
			point_of_view_.up.vector.z = 1;

			// Camera will be behind temoto_end_effector, somewhat elevated
			point_of_view_.eye.point.x = latest_status_.end_effector_pose.pose.position.x - EYE_DISPLACEMENT_FRONT_;// Distance backwards from the end effector
			point_of_view_.eye.point.y = latest_status_.end_effector_pose.pose.position.y;				// Align with end effector
			point_of_view_.eye.point.z = latest_status_.end_effector_pose.pose.position.z + EYE_DISPLACEMENT_TOP_;// Distance upwards from the end effector

			// Look at the distance of VIRTUAL_VIEW_SCREEN from the origin temoto_end_effector frame, i.e. the palm of robotiq gripper
			point_of_view_.focus.point = latest_status_.end_effector_pose.pose.position;
	  }
	  
	  latest_known_camera_mode_ = 1;				// set latest_known_camera_mode to 1, i.e. natural
	  pov_publisher.publish( point_of_view_ );			// publish a CameraPlacement msg
	  adjust_camera_ = false;					// set adjust_camera 'false'
      
  } // else if (!latest_status.in_navigation_mode && adjust_camera)
} // end Visuals::crunch()

/** Main */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rviz_visual");
  ros::NodeHandle n;
  
  // Setting the node rate (Hz)
  ros::Rate node_rate(30);

  Visuals rviz_visuals;
  
  ros::Subscriber sub_status = n.subscribe("temoto/status", 1, &Visuals::updateStatus, &rviz_visuals);
  
  // Publisher of CameraPlacement messages (this is picked up by rviz_animated_view_controller).
  // When latch is true, the last message published is saved and automatically sent to any future subscribers that connect. Using it to set camera during rviz startup.
  ros::Publisher pub_pov_camera = n.advertise<view_controller_msgs::CameraPlacement>( "/rviz/camera_placement", 3, true );

  // Publisher on /visualization_marker to depict the hand pose with several rviz markers (this is picked up by rviz)
  ros::Publisher pub_marker = n.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );

  // Wait for initial end-effector position to set camera position

  while ((rviz_visuals.latest_status_.end_effector_pose.pose.position.x==0) && (rviz_visuals.latest_status_.end_effector_pose.pose.position.y==0) && (rviz_visuals.latest_status_.end_effector_pose.pose.position.z==0) )
  {
    ROS_WARN("[rviz_visual] Waiting for the initial end effector pose.");
    ROS_WARN("[rviz_visual] For distributed systems, are your clocks synched?");
    ros::spinOnce();
    ros::Duration(1.).sleep();
  }

  rviz_visuals.crunch(pub_marker, pub_pov_camera);
 
  // Publish the initial CameraPlacement; so that rviz_animated_view_controller might _latch_ on to it
  pub_pov_camera.publish( rviz_visuals.point_of_view_ );
  
  while ( ros::ok() )
  {
    // Update point-of-view camera pose and all the visualization markers
    rviz_visuals.crunch(pub_marker, pub_pov_camera);
    
    ros::spinOnce();
    node_rate.sleep();
  }
  
} // end main()
