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

// ROS includes
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "view_controller_msgs/CameraPlacement.h"
#include "visualization_msgs/Marker.h"

#include "geometry_msgs/Pose.h"

// temoto includes
#include "temoto_common.h"
#include "griffin_powermate/PowermateEvent.h"

#define VIRTUAL_SCREEN_FRONT 0		///< A number that is used when some distance is required between robot and any marker or camera.
#define VIRTUAL_SCREEN_TOP 0.2		///< A number that is used when some distance is required between robot and any marker or camera.

bool adjust_camera = true;	///< This is set 'true', if it is needed to adjust the position of the point-of-view camera in RViz.
bool camera_is_aligned = true;	///< Camera is algined with end effector (i.e. 'true') as opposed to being top-view (i.e. 'false').
temoto::Status latest_status;	///< latest recieved full system status published by start_teleop node

/** The actual service call function for temoto/adjust_rviz_camera.
 *  @param req empty service request.
 *  @param res empty service response.
 *  @return always true.
 */
bool adjustCameraPlacement (std_srvs::Empty::Request  &req,
			    std_srvs::Empty::Response &res)
{
  ROS_INFO("[rviz_visual/adjust_rviz_camera] adjust_camera is set 'true' due to service request.");
  // Set adjust_camera to TRUE.
  adjust_camera = true;
  return true;
}

/** Callback function for /temoto/status.
 *  Looks for any changes that would require re-adjustment of the point-of-view camera.
 *  Stores the received status in a global variable latest_status.
 *  @param status temoto::Status message
 */
void updateStatus (temoto::Status status)
{
  // Before overwriting previous status, checks if switch between camera views is necesassry due to switch between navigation and manipulation modes.
  if (status.in_navigation_mode && !latest_status.in_navigation_mode)
  {
    adjust_camera = true;
    ROS_INFO("[rviz_visual/updateStatus] adjust_camera is set 'true' due change from MANIPULATION to NAVIGATION.");
  }
  else if (!status.in_navigation_mode && latest_status.in_navigation_mode)
  {
    adjust_camera = true;
    ROS_INFO("[rviz_visual/updateStatus] adjust_camera is set 'true' due change from NAVIGATION to MANIPULATION.");
  }

  // Before overwriting previous status, checks if switch between camera views is necesassry due to limited directions.
  if (status.position_forward_only && !latest_status.position_forward_only)
  {
    adjust_camera = true;
    camera_is_aligned = false;	// Change to top-view as operator has switched to forward only motion pattern
    ROS_INFO("[rviz_visual/updateStatus] adjust_camera is set 'true' due change from 'not fwd only' to 'fwd only'.");
  }
  else if (!status.position_forward_only && latest_status.position_forward_only)
  {
    adjust_camera = true;
    camera_is_aligned = true;	// Change to the so-called aligned view because operator has stopped using forward only
    ROS_INFO("[rviz_visual/updateStatus] adjust_camera is set 'true' due change from 'fwd only' to 'not fwd only'.");
  }
  
  // Checks if switch between camera views is necesassry due to change in control mode.
  if (status.in_natural_control_mode && !latest_status.in_natural_control_mode)
  {
    adjust_camera = true;
    ROS_INFO("[rviz_visual/updateStatus] adjust_camera is set 'true' due to switch to 'natural control mode'.");
  }
  else if (!status.in_natural_control_mode && latest_status.in_natural_control_mode)
  {
    adjust_camera = true;
    ROS_INFO("[rviz_visual/updateStatus] adjust_camera is set 'true' due to switch to 'inverted control mode'.");
  }
  
  // Checks if switch between camera views is necesassry due to change in (un)limiting directions.
  if (status.position_unlimited != latest_status.position_unlimited) adjust_camera = true;
  
  // Check difference between the known and new end effector positions
  std::vector <geometry_msgs::Point> now_and_before;				// Vector that contains current and target points
  now_and_before.push_back(status.end_effector_pose.pose.position);		// Add the just received position of end effector
  now_and_before.push_back(latest_status.end_effector_pose.pose.position);	// Add the previous known position of end effector
  double shift = calculateDistance(now_and_before);				// Calculate the linear distance between the two positions
  if (shift > 0.001) adjust_camera = true;					// If the difference is more than 1 mm, set adjust_camera to true.
  
  // Overwrite latest_status values with the new status.
  latest_status = status;
  return;
}

/** Callback function for /griffin_powermate/events.
 *  Sets adjust_camera flag if griffin powermate turn knob was rotated, i.e., any turn knob rotation triggers re-adjustement of camera placement.
 *  @param powermate griffin_powermate::PowermateEvent message from Griffin Powermate.
 */
void powermateWheelEvent (griffin_powermate::PowermateEvent powermate)
{
  if (abs(powermate.direction)) adjust_camera = true;
  return;
}

/** Calculates the distance between two points in meters or millimeters and returns it as a string.
 *  @param twoPointVector vector containing two points.
 *  @return string containing the distance between two points in meters or millimeters followed by ' m' or ' mm', respectively.
 */
std::string getDistanceString (std::vector <geometry_msgs::Point> & twoPointVector)
{
  std::string  distance_as_text;
  std::string  units;
  int precision = 2;
  // Calculate the distance between the first two points in the input vector of points
//  double distance = sqrt( pow(twoPointVector[1].x-twoPointVector[0].x, 2) + pow(twoPointVector[1].y-twoPointVector[0].y, 2) + pow(twoPointVector[1].z-twoPointVector[0].z, 2));
  double distance = calculateDistance(twoPointVector);
  if (distance < 1)	// if distance is less than 1 m, use mm instead
  {
    // Covnvert the distance m -> mm
    distance = distance*1000;
    units = " mm";
    if (distance >= 10) precision = 1;
  }
  else			// otherwise, use meters as units
  {
    units = " m";
  }
  // use the number of fractional digits specified by presion to put distance into stringstream and add units.
  std::ostringstream sstream;
  sstream << std::fixed << std::setprecision(precision) << distance << units;

  // stringstream to string
  distance_as_text = sstream.str();
  return distance_as_text;
}

/** A class to encapsulate view_controller_msgs::CameraPlacement messages with some helper function.
 */
class POWCamera
{
public:
  // default constructor 
  POWCamera()
  {
    // initialize camera placement message to frame_id "temoto_end_effector"
    initCameraPlacement("temoto_end_effector");
  }
  
  // ros camera placement message that defines placement of our POWCamera instance
  view_controller_msgs::CameraPlacement placement;
  
  // initialized camera placement to a preset pose in frame specified by frame_id
  void initCameraPlacement(std::string frame_id);
  
  // changes the frame_id of target_frame and every pose in CameraPlacement message 
  void changeTargetFrameTo(std::string frame_id);
  
};

/** Creates the initial CameraPlacment message that is used for positioning point-of-view camera in RViz.
 */
void POWCamera::initCameraPlacement(std::string frame_id)
{
  // Set target frame and animation time
  placement.target_frame = frame_id;
  placement.time_from_start = ros::Duration(0.5);

  // Position of the camera relative to target_frame
  placement.eye.header.frame_id = placement.target_frame;
  placement.eye.point.x = -2;
  placement.eye.point.y = 0;
  placement.eye.point.z = 0;

  // Target_frame-relative point for the focus
  placement.focus.header.frame_id = placement.target_frame;
  placement.focus.point.x = 0;
  placement.focus.point.y = 0;
  placement.focus.point.z = 0;

  // Target_frame-relative vector that maps to "up" in the view plane.
  placement.up.header.frame_id = placement.target_frame;
  placement.up.vector.x = 0;
  placement.up.vector.y = 0;
  placement.up.vector.z = 1;
  
}

/** Changes target_frame and header.frame_ids of every pose in placement to frame_id.
 */
void POWCamera::changeTargetFrameTo(std::string frame_id)
{
  placement.target_frame = frame_id;
  placement.eye.header.frame_id = placement.target_frame;
  placement.focus.header.frame_id = placement.target_frame;
  placement.up.header.frame_id = placement.target_frame;
}


/** Creates the initial marker that visualizes hand movement as an shift arrow.
 *  It is called only once when rviz_visual node is started.
 *  @return visualization_msgs::Marker.
 */
visualization_msgs::Marker initShiftArrow()
{
  visualization_msgs::Marker arrow;
  arrow.header.frame_id = "leap_motion"; // x is horizontal, y is vertical, z is forward-backward // "temoto_end_effector"; // x is forward, y is horizontal, z is vertical //
  arrow.header.stamp = ros::Time();
  arrow.ns = "shift_arrow";
  arrow.id = 0;
  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.action = visualization_msgs::Marker::ADD;
  // Defining start and end points for the arrow marker, the actual values don't matter here as they will be overwritten in the main
  geometry_msgs::Point start_point, end_point;
  start_point.x = 0.0;
  start_point.y = 0.0;
  start_point.z = 0.0;
  end_point.x = 0.0;
  end_point.y = -0.2;
  end_point.z = 0.0;
  // The point at index 0 is assumed to be the start point, and the point at index 1 is assumed to be the end. 
  arrow.points.push_back(start_point);
  arrow.points.push_back(end_point);
  // scale.x is the shaft diameter, and scale.y is the head diameter. If scale.z is not zero, it specifies the head length. 
  arrow.scale.x = 0.001;
  arrow.scale.y = 0.002;
  arrow.scale.z = 0;
  
  // Make the arrow visible by setting alpha to 1.
  arrow.color.a = 1.0;  
  // make the arrow orange
  arrow.color.r = 1.0;
  arrow.color.g = 0.5;
  arrow.color.b = 0.0;
  
  return arrow;
}

/** Creates the initial marker that displays front-facing text.
 *  It is called only once when rviz_visual node is started.
 *  @return visualization_msgs::Marker.
 */
visualization_msgs::Marker initDisplayDistance()
{
  visualization_msgs::Marker display_text;
  display_text.header.frame_id = "leap_motion";
  display_text.header.stamp = ros::Time();
  display_text.id = 1;
  display_text.ns = "distance_as_label";
  display_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  display_text.action = visualization_msgs::Marker::ADD;
  display_text.text = "loading ...";
  // set the position of text; the actual values don't matter here as they will be overwritten in the main
  display_text.pose.position.x = 0.0;
  display_text.pose.position.y = 0.2;
  display_text.pose.position.z = 0.0;
  // Text height
  display_text.scale.z = 0.1;
  // Text visible
  display_text.color.a = 1;
  display_text.color.b = 1;	// it's blue
  
  return display_text;
} // end initDisplayDistance

/** Creates the initial marker that visualizes hand pose as a flattened box.
 *  It is called only once when rviz_visual node is started.
 *  @return visualization_msgs::Marker.
 */
visualization_msgs::Marker initHandPoseMarker()
{
  visualization_msgs::Marker hand_proxy;
  hand_proxy.header.frame_id = "leap_motion";
  hand_proxy.header.stamp = ros::Time();
  hand_proxy.ns = "hand_pose_box";
  hand_proxy.id = 0;
  hand_proxy.type = visualization_msgs::Marker::CUBE;
  hand_proxy.action = visualization_msgs::Marker::ADD;

  hand_proxy.scale.x = 0.06;	// side
  hand_proxy.scale.y = 0.02;	// thickness
  hand_proxy.scale.z = 0.15;	// depth
  
  // begin at the position of end effector
  hand_proxy.pose.position.x = 0.0;
  hand_proxy.pose.position.y = 0.0;
  hand_proxy.pose.position.z = 0.0;

  // Make the box visible by setting alpha to 1.
  hand_proxy.color.a = 1;  
  // Make the box orange.
  hand_proxy.color.r = 1.0;
  hand_proxy.color.g = 0.5;
  hand_proxy.color.b = 0.0;
  
  return hand_proxy;
}

/** Creates the initial marker for a visual aid box around the robot where target position is one of the corners.
 *  It is called only once when rviz_visual node is started.
 *  @return visualization_msgs::Marker.
 */
visualization_msgs::Marker initTargetAid3D()
{
  visualization_msgs::Marker aid_box;
  aid_box.header.frame_id = "leap_motion";
  aid_box.header.stamp = ros::Time();
  aid_box.ns = "aid_box";
  aid_box.id = 0;
  aid_box.type = visualization_msgs::Marker::CUBE;
  aid_box.action = visualization_msgs::Marker::ADD;

  aid_box.scale.x = 0.2;	// side
  aid_box.scale.y = 0.2;	// thickness
  aid_box.scale.z = 0.2;	// depth
  
  // begin at the position of end effector
  aid_box.pose.position.x = 0.0;
  aid_box.pose.position.y = 0.0;
  aid_box.pose.position.z = 0.0;

  // Make the box barely visible by setting alpha to 0.2.
  aid_box.color.a = 0.2;  
  // Make the box red.
  aid_box.color.r = 1.0;
  aid_box.color.g = 0.0;
  aid_box.color.b = 0.0;
  
  return aid_box;
}

/** Creates the initial marker that visualizes cartesian path by connecting wayposes with lines.
 *  It is called only once when rviz_visual node is started.
 *  @return visualization_msgs::Marker.
 */
visualization_msgs::Marker initCartesianWaypoints()
{
  visualization_msgs::Marker waypoints;
  waypoints.header.frame_id = "base_link"/*"leap_motion"*/;
  waypoints.header.stamp = ros::Time();
  waypoints.ns = "waypoints";
  waypoints.id = 0;
  waypoints.type = visualization_msgs::Marker::LINE_STRIP;
  waypoints.action = visualization_msgs::Marker::ADD;

  waypoints.scale.x = 0.01;	// width of the line

  // Make the line visible by setting alpha to 1.
  waypoints.color.a = 1;  
  // Make the line strip blue.
  waypoints.color.r = 0.0;
  waypoints.color.g = 0.0;
  waypoints.color.b = 1.0;
  
  return waypoints;
}

/** Main method. */
int main(int argc, char **argv)
{
  // ROS init
  ros::init(argc, argv, "rviz_visual");
  // ROS node handle
  ros::NodeHandle n;
  // Setting the node rate at 1 kHz
  ros::Rate node_rate(1000);
  
  // ROS subscriber on /temoto/status
  ros::Subscriber sub_status = n.subscribe("temoto/status", 1, updateStatus);

  // ROS subscriber on /griffin_powermate. Detecting griffin powermate events.
  ros::Subscriber sub_dial = n.subscribe("/griffin_powermate/events", 1, powermateWheelEvent);
  
  // Set up service /temoto/adjust_rviz_camera; if there's a service request, executes adjustCameraPlacement() function
  ros::ServiceServer srv_visual = n.advertiseService("temoto/adjust_rviz_camera", adjustCameraPlacement);
  
  // Publisher of CameraPlacement messages (this is picked up by rviz_animated_view_controller).
  // When latch is true, the last message published is saved and automatically sent to any future subscribers that connect. Using it to set camera during rviz startup.
  ros::Publisher pub_cam = n.advertise<view_controller_msgs::CameraPlacement>( "/rviz/camera_placement", 3, true );

  // Instantiate a POWCamera that holds the initial CameraPlacement message.
  POWCamera pow_cam;

  // The following is related to getting visual markers to represent target pose in RViz
  // publish on /visualization_marker to depict the hand pose with several rviz markers (this is picked up by rviz)
  ros::Publisher pub_marker = n.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
  visualization_msgs::Marker shift_arrow = initShiftArrow();
  visualization_msgs::Marker display_distance = initDisplayDistance();
  visualization_msgs::Marker hand_pose = initHandPoseMarker();
  visualization_msgs::Marker aid3D = initTargetAid3D();
  visualization_msgs::Marker cartesian_path = initCartesianWaypoints();
  
  // publish the initial CameraPlacement; so that rviz_animated_view_controller might _latch_ on to it
  pub_cam.publish( pow_cam.placement );

  // This is where rviz_visual thinks the camera is.
  uint8_t latest_known_camera_mode = 0; 	// 0-unknown, 1-natural, 2-inverted, 11-navigation natural, 12-navigation inverted
  
  while (ros::ok()) {
    
    // ============================================================ 
    // ==  VISUALIZATION MARKERS  =================================
    // ============================================================
    
    // Setting markers in NAVIGATION mode
    if (latest_status.in_navigation_mode)
    {
      /* ==  HAND POSE BOX  ==================================== */
      // Resize of the hand pose marker to robot base dimensions
      hand_pose.scale.x = 0.5;
      hand_pose.scale.z = 1.0; 
      
      // Hand pose marker (relative to leap_motion frame)
      hand_pose.pose = latest_status.live_hand_pose.pose;
      
      // Paint the marker based on restricted motion latest_status
      if (latest_status.orientation_free) hand_pose.color.g = 0.0;	// if hand orientation is to be considered, paint the hand pose marker red 
      else hand_pose.color.g = 0.5;					// else, the marker is orange
      
      // In NAVIGATION, the hand pose marker must always be visible
      hand_pose.color.a = 1;
      
      // Publish hand_pose marker
      pub_marker.publish( hand_pose );
      
      /* ==  TEXT LABEL  ======================================= */
      // Update display_distance parameters (display_distance operates relative to leap_motion frame)
      std::vector<geometry_msgs::Point> temp;
      geometry_msgs::Point zero_point;
      temp.push_back(zero_point);
      temp.push_back(latest_status.live_hand_pose.pose.position);
      display_distance.text = getDistanceString(temp);			// get the linear distance from zero to hand position end point as string
      display_distance.scale.z = 0.05 + latest_status.scale_by/8;	// scale the display text based on scale_by value
      display_distance.pose.position = temp[1];				// text is placed at the hand position
      display_distance.pose.position.y = 1; 				// raise text to the top 1 m
      
      pub_marker.publish( display_distance );				// publish info_text visualization_marker; this is picked up by rivz
    }
    // Setting markers in MANIPULATION mode
    else
    {
      /* ==  ARROW  ============================================ */
      // Update arrow marker properties and make hand tracking visible in rviz
      // Arrow marker is described in leap_motion frame.
      // Start point of the arrow is always at (0, 0, 0).
      shift_arrow.points[1] = latest_status.live_hand_pose.pose.position;// set the end point of the arrow to actual hand position
      if (latest_status.in_natural_control_mode)
      {
	shift_arrow.points[0].z = -VIRTUAL_SCREEN_FRONT;		// shift the marker in front of the FT sensor and gripper
	shift_arrow.points[1].z -= VIRTUAL_SCREEN_FRONT;		// shift the marker in front of the FT sensor and gripper
      }
      else
      {
	shift_arrow.points[0].z = VIRTUAL_SCREEN_FRONT;			// shift the marker in front of the FT sensor and gripper
	shift_arrow.points[1].z += VIRTUAL_SCREEN_FRONT;		// shift the marker in front of the FT sensor and gripper
      }

      // Change arrow's thickness based on scaling factor
      shift_arrow.scale.x = 0.001 + latest_status.scale_by/1000;	// shaft diameter when start and end point are defined
      shift_arrow.scale.y = 0.002 + latest_status.scale_by/100;		// head diameter when start and end point are defined
      shift_arrow.scale.z = 0;						// automatic head length
      shift_arrow.color.a = 1;						// the arrow is not transparent

      // A tweak for extreme close-ups to make the arrow out of scale but more informative
      if (latest_status.scale_by < 0.01 && camera_is_aligned /*&& !latest_status.in_natural_control_mode*/)// only when the rviz camera is in the front
      {
	shift_arrow.points[0].z -= 0.02;				// shift arrow marker away from the camera because camera is at the VIRTUAL_VIEW_SCREEN
	shift_arrow.points[1].z -= 0.02;     
	shift_arrow.scale.x = 0.0001;					// make the shaft thinner
	shift_arrow.scale.y = 0.0003;					// make the arrow head thinner
	shift_arrow.color.a = 0.6;					// and make the arrow a bit transparent
      }
      
      // Change arrow's appearance when camera in on the top, looking down
      if (!camera_is_aligned && !latest_status.position_unlimited)	// if camera is on the top and facing down, the arrow marker must be made more visible
      {
	shift_arrow.scale.x = 0.15;					// shaft is now quite fat
	shift_arrow.scale.y = 0.18;					// head is also made large
	shift_arrow.color.a = 0.4;					// the arrow marker is transparent
      }

      if (latest_status.position_unlimited) shift_arrow.color.g = 0.0;	// if hand position input is unresricted, paint the arrow red 
      else shift_arrow.color.g = 0.5;					// else, the arrow is orange
      
      pub_marker.publish( shift_arrow );				// publish shift_arrow visualization_marker; this is picked up by rviz
      
      /* ==  3D VISUAL AID  ==================================== */
      // Use the same origin/pivot point as the start point of the arrow marker
      aid3D.pose.position = shift_arrow.points[0];
      // Dimensions of the box are determined by the hand position
      aid3D.scale.x = latest_status.live_hand_pose.pose.position.x * 2;
      aid3D.scale.y = latest_status.live_hand_pose.pose.position.y * 2;
      aid3D.scale.z = latest_status.live_hand_pose.pose.position.z * 2;
      // If setting pose in one plane only, give the aid box thickness of the arrow arrow
      if (!latest_status.position_forward_only && !latest_status.position_unlimited) aid3D.scale.z = shift_arrow.scale.x;
      // Coloring the visual aid box
      aid3D.color = shift_arrow.color;					// use the same color as arrow
      aid3D.color.a = 0.2;						// make it transparent
      
      pub_marker.publish( aid3D );					// publish the visual aid box
      
      /* ==  TEXT LABEL  ======================================= */
      // Update display_distance parameters (display_distance operates relative to leap_motion frame)
      display_distance.text = getDistanceString(shift_arrow.points);	// get the distance between start and end point in mm as string
      if (!latest_status.in_natural_control_mode)			// INVERTED CONTROL MODE
      {
	display_distance.scale.z = 0.001 + latest_status.scale_by/20;	// scale the display text based on scale_by value
	display_distance.pose.position = shift_arrow.points[1];		// text is positioned at the end of the arrow marker
      }
      else								// NATURAL CONTROL MODE
      {
	display_distance.scale.z = 0.010 + latest_status.scale_by/20;	// scale the display text based on scale_by value
	display_distance.pose.position = shift_arrow.points[1];		// text is positioned at the end of the arrow marker
	if (latest_status.scale_by < 0.1 && camera_is_aligned) display_distance.scale.z = 0.001 + latest_status.scale_by/20;	// extreme close-up
      }
      // A tweak for when the camera is on the top facing down; lift text above the arrow
      if (!camera_is_aligned) display_distance.pose.position.y = 0.7*shift_arrow.scale.y; // lift text to the top of arrows head
      // A tweak for bringing the text in front of the hand pose orientation marker for better visibility
      if (latest_status.orientation_free && camera_is_aligned) display_distance.pose.position.z += 0.1;   // shift text in front of the hand pose rotation marker
      
      pub_marker.publish( display_distance );				// publish info_text visualization_marker; this is picked up by rivz
      
      /* ==  HAND POSE BOX  ==================================== */
      // Size of the hand pose marker
      hand_pose.scale.x = 0.06;
      hand_pose.scale.z = 0.15; 
      // Hand pose marker (relative to leap_motion frame)
      hand_pose.pose = latest_status.live_hand_pose.pose;
      
      // Shift the marker in front of the ft sensor and robotiq gripper
      if (latest_status.in_natural_control_mode) hand_pose.pose.position.z -= VIRTUAL_SCREEN_FRONT;
      else hand_pose.pose.position.z += VIRTUAL_SCREEN_FRONT;
      
      // Paint the marker based on restricted motion molatest_status.des
      if (latest_status.orientation_free) hand_pose.color.g = 0.0;	// if hand orientation is to be considered, paint the hand pose marker red 
      else hand_pose.color.g = 0.5;					// else, the marker is orange

      // SPECIAL CASE! Hide hand pose marker when orientation is to be ignored.
      if (/*!latest_status.in_natural_control_mode &&*/ !latest_status.orientation_free) hand_pose.color.a = 0;	// make the marker invisible
      else hand_pose.color.a = 1;
      
      pub_marker.publish( hand_pose );
      
      /* ==  CARTESIAN WAYPOINTS  ============================== */
      // Empty any previous waypoints from the visualized cartesian path
      cartesian_path.points.clear();

      // Take only position members of available cartesian wayposes and push them to cartesian_path.
      for (int i = 0; i < latest_status.cartesian_wayposes.size(); ++i)
      {
	cartesian_path.points.push_back( latest_status.cartesian_wayposes.at(i).position );
      }

      // Publish cartesian_path as LINE_STRIP marker
      pub_marker.publish( cartesian_path );
      
    } // end setting markers in MANIPULATION mode
    
    // ============================================================ 
    // ==  CAMERA POSE  ===========================================
    // ============================================================
    // Setting 'adjust_camera' triggers repositioning of the point-of-view camera.
    
    // Adjust camera in NAVIGATION mode
    if (latest_status.in_navigation_mode && adjust_camera)
    {
      // During NAVIGATION camera moves relative to 'base_link' frame.
      pow_cam.changeTargetFrameTo("base_link");
      
      ROS_INFO("NAVIGATION/NATURAL: Switching to top view.");
      
      // For NAVIGATION/NATURAL +X is considered to be 'UP'.
      pow_cam.placement.up.vector.x = 1;
      pow_cam.placement.up.vector.z = 0;

      // Camera is positioned directly above the origin of base_link
      pow_cam.placement.eye.point.x = 0;
      pow_cam.placement.eye.point.y = 0;
      pow_cam.placement.eye.point.z = 2 + 8*latest_status.scale_by;	// Never closer than 2 m, max distance at 10 m

      // Focus camera at the origin of base_link
      pow_cam.placement.focus.point.x = 0;
      
      latest_known_camera_mode = 11;					// set latest_known_camera_mode to 11, i.e navigation natural
      pub_cam.publish( pow_cam.placement );				// publish the modified CameraPlacement message
      adjust_camera = false;						// set adjust_camera 'false'     
    }
    // Adjust camera in MANIPULATION mode
    else if (!latest_status.in_navigation_mode && adjust_camera)
    {
      // During MANIPULATION camera akways moves relative to 'temoto_end_effector' frame.
      pow_cam.changeTargetFrameTo("temoto_end_effector");
      
      // Adjust camera for NATURAL CONTROL MODE
      if (latest_status.in_natural_control_mode)
      {
	if (camera_is_aligned)					// here alignment means the so-called bird's view
	{
	  ROS_INFO("NATURAL: Switching to aligned view.");
	  // Set +Z as 'UP'
	  pow_cam.placement.up.vector.x = 0;
	  pow_cam.placement.up.vector.z = 1;

	  // Camera will be behind temoto_end_effector, somewhat elevated
	  pow_cam.placement.eye.point.x = -2*latest_status.scale_by;	// Distance backwards from the end effector
	  pow_cam.placement.eye.point.y = 0;				// Align with end effector
	  pow_cam.placement.eye.point.z = 0.2 + 2*latest_status.scale_by;// Distance upwards from the end effector
	  // if constrained to a plane and scaled down align camrea with the end effector in z-direction
	  if (!latest_status.position_unlimited && latest_status.scale_by < 0.1) pow_cam.placement.eye.point.z = 0;
	  
	  // Look at the distance of VIRTUAL_VIEW_SCREEN from the origin temoto_end_effector frame, i.e. the palm of robotiq gripper
	  pow_cam.placement.focus.point.x = VIRTUAL_SCREEN_FRONT;
	}
	else							// natural control mode top view
	{
	  ROS_INFO("NATURAL: Switching to top view.");
	  // In the latest_known_camera_mode = 1;top view of natural control mode, +X is considered to be 'UP'.
	  pow_cam.placement.up.vector.x = 1;
	  pow_cam.placement.up.vector.z = 0;

	  // Camera is positioned directly above the virtual FRONT view screen.
	  pow_cam.placement.eye.point.x = VIRTUAL_SCREEN_FRONT;				// Above the virtual FRONT view screen
	  pow_cam.placement.eye.point.y = 0;
	  pow_cam.placement.eye.point.z = VIRTUAL_SCREEN_TOP + 1.5*latest_status.scale_by;	// Never closer than virtual TOP view screen, max distance at 1.5 m

	  // Look at the distance of VIRTUAL_VIEW_SCREEN from the origin temoto_end_effector frame, i.e. look at virtual FRONT view screen
	  pow_cam.placement.focus.point.x = VIRTUAL_SCREEN_FRONT;
	}
	latest_known_camera_mode = 1;				// set latest_known_camera_mode to 1, i.e natural
	pub_cam.publish( pow_cam.placement );			// publish a CameraPlacement msg
	adjust_camera = false;					// set adjust_camera 'false'
      } // if adjust_camera in_natural_control_mode
      
      // adjust camera for INVERTED CONTROL MODE
      if (!latest_status.in_natural_control_mode)
      {
	if (camera_is_aligned)					// here alignment means the camera is in the front facing the end effector
	{
	  ROS_INFO("INVERTED: Switching to aligned view.");
	  // Set +Z as 'UP'
	  pow_cam.placement.up.vector.z = 1;
	  pow_cam.placement.up.vector.x = 0;

	  // Position camera on the x-axis no closer than virtual FRONT view screen, max distance at 1.5 m.
	  pow_cam.placement.eye.point.x = VIRTUAL_SCREEN_FRONT + 1.5*latest_status.scale_by;
	  pow_cam.placement.eye.point.y = 0;				// move camera to align with end effector along the y-axis
	  pow_cam.placement.eye.point.z = 0;				// move camera to align with end effector along the z-axis
	  if (latest_status.position_unlimited) pow_cam.placement.eye.point.z = 0.1 + 1*latest_status.scale_by;	// shift camera upwards ... 

	  // Look at the origin of temoto_end_effector, i.e. all zeros
	  pow_cam.placement.focus.point.x = VIRTUAL_SCREEN_FRONT;
	}
	else							// else means camera should be in the top position
	{
	  ROS_INFO("INVERTED: Switching to top view.");
	  // In the top view of inverted control mode, -X is considered 'UP'.
	  pow_cam.placement.up.vector.z = 0;
	  pow_cam.placement.up.vector.x = -1;

	  // Camera is positioned directly above the virtual FRONT view screen.
	  pow_cam.placement.eye.point.x = VIRTUAL_SCREEN_FRONT;					// Above the virtual FRONT view screen
	  pow_cam.placement.eye.point.y = 0;
	  pow_cam.placement.eye.point.z = VIRTUAL_SCREEN_TOP + 1.5*latest_status.scale_by;	// Never closer than virtual TOP view screen, max distance at 1.5 m
	  
	  // Look at the distance of VIRTUAL_VIEW_SCREEN from the origin temoto_end_effector frame, i.e. look at virtual FRONT view screen
	  pow_cam.placement.focus.point.x = VIRTUAL_SCREEN_FRONT;
	}
	latest_known_camera_mode = 2;				// set latest_known_camera_mode to 2, i.e inverted
	pub_cam.publish( pow_cam.placement );			// publish a CameraPlacement msg
	adjust_camera = false;					// set adjust_camera 'false'
      } // if adjust_camera not in_natural_control_mode
    } // else if (!latest_status.in_navigation_mode && adjust_camera)
    
    // Doublecheck    
    // If camera IS NOT in natural mode but system IS in natural mode, try adjusting the pow camera.
    if (latest_known_camera_mode!=1 && latest_status.in_natural_control_mode)
    {
      adjust_camera = 1;
      // unless in recognized NAVIGATION mode, then all was OK
      if (latest_known_camera_mode == 11 && latest_status.in_navigation_mode) adjust_camera = 0;
    }
    // If camera IS NOT in inverted mode but the system IS in inverted mode, try adjusting the pow camera.
    if (latest_known_camera_mode!=2 && !latest_status.in_natural_control_mode) adjust_camera = 1;
    // If camera IS NOT in NAVIGATION natural mode but system IS in NAVIGATION natural mode, try adjusting the pow camera.
    if (latest_known_camera_mode!=11 && latest_status.in_navigation_mode) adjust_camera = 1;
    
    ros::spinOnce();
    node_rate.sleep();
    
  } // end while()
} // end main()