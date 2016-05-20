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
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"

// temoto includes
#include "temoto_include.h"
#include "leap_motion_controller/LeapMotionOutput.h"

// Other includes
#include "math.h"
#include <cstdlib>
#include <iostream>

#define HEIGHT_OF_ZERO 0.2		///< Height of zero position above Leap Motion Controller.

double scale_by = 1;			///< Scaling factor, it is normalized to 1.
int8_t AMP_HAND_MOTION = 10; 		///< Amplification of hand motion.

ros::ServiceClient move_client;		///< This service client for temoto/move_robot_service is global, so that callback funtions could see it.
ros::ServiceClient tf_client;		///< Service client for requesting changes of control mode, i.e., change of orientation for leap_motion frame. 

geometry_msgs::PoseStamped current_pose;///< Latest pose value received for the end effector.
geometry_msgs::PoseStamped desired_pose;///< Properly scaled target pose in reference to leap motion frame.
std::vector<geometry_msgs::Pose> wayposes;	///< Vector of desired wayposes for a Cartesian move of the end-effector. Specified in the frame of the end-effector.

// 'natural' robot and human are oriented the same way; 'inverted' means the human operator is facing the robot so that left and right are inverted.
bool using_natural_control = true;	///< Mode of intepration for hand motion: 'true' - natural, i.e., human and robot arms are the same; 'false' - inverted.

bool orientation_locked = false;	///< 'True' if hand orientation info is to be ignored.
bool position_limited = true;		///< 'True' if hand position is restricted to a specific direction/plane.
bool position_fwd_only = false;		///< 'True' when hand position is restricted to back and forward motion. Is only relevant when position_limited is 'true'.
bool right_hand_before = false;		///< Presense of right hand during the previous iteration of Leap Motion's callback processLeap(..).


/** Function that actually makes the service call to temoto/move_robot_service.
 *  @param action_type determines what is requested from MoveGroup, i.e. PLAN (0x01), EXECUTE PLAN (0x02), or PLAN&EXECUTE (0x03). 
 */
void callPlanAndMove(uint8_t action_type) {
  // Create a service request
  temoto::Goal move;	
  move.request.goal = desired_pose;		// set desired_pose as the requested pose for the motion planner
  move.request.action_type = action_type;	// set action_type

  // Adjust orientation for inverted control mode, i.e. translate leap_motion to leap_motion_on_robot
  if (!using_natural_control) {							// fix orientation for inverted view only
    tf::Quaternion invert_palm_rotation(0, 1, 0, 0);				// 180° turn around Y axis
    tf::Quaternion palm_orientation;						// incoming palm orientation
    tf::quaternionMsgToTF(move.request.goal.pose.orientation, palm_orientation);// convert incoming quaternion msg to tf qauternion
    tf::Quaternion final_rotation = palm_orientation * invert_palm_rotation;	// apply invert_palm_rotation to incoming palm rotation
    final_rotation.normalize();							// normalize quaternion
    tf::quaternionTFToMsg(final_rotation, move.request.goal.pose.orientation);	// convert tf quaternion to quaternion msg
  }
  
  if (move_client.call(move)) {			// call for the temoto/move_robot_service
    ROS_INFO("Successfully called temoto/move_robot_service");
  } else {
    ROS_ERROR("Failed to call temoto/move_robot_service");
  }
  return;
} // end callPlanAndMove

/** Function that actually makes the service call to /temoto/move_robot_service.
 *  @param action_type determines what is requested from MoveGroup, i.e. PLAN (0x01), EXECUTE PLAN (0x02), or PLAN&EXECUTE (0x03).
 *  @param named_target uses this named target for target pose.
 */
void callPlanAndMoveNamedTarget(uint8_t action_type, std::string named_target) {
  temoto::Goal move;				// create a service request
  move.request.action_type = action_type;	// set action_type
  move.request.named_target = named_target;	// set named_target as the goal
  
  if (move_client.call(move)) {			// call for the service to move SIA5
    ROS_INFO("Successfully called temoto/move_robot_service");
  } else {
    ROS_ERROR("Failed to call temoto/move_robot_service");
  }
  return;
} // end callPlanAndMoveNamedTarget

/** Function that makes the service call to /temoto/move_robot service.
 * 
 */
void computeCartesian(std::string frame_id) {
  temoto::Goal move;				// create a service request
  move.request.action_type = 0x04;		// set action_type
  move.request.cartesian_wayposes = wayposes;
  move.request.cartesian_frame = frame_id;
  if (move_client.call(move)) {			// call for the service to move SIA5
    ROS_INFO("Successfully called temoto/move_robot_service for Cartesian move.");
  } else {
    ROS_ERROR("Failed to call temoto/move_robot_service for Cartesian move.");
  }
  return;
}

/** Callback function for /leapmotion_general subscriber.
 *  It updates target pose based on latest left palm pose (any scaling and/or relevant limitations are also being applied).
 *  Presence of the right hand is used to lock and unlock forward motion.
 *  KEY_TAP gesture detection is currenly unimplemented.
 *  @param leap_data temoto::Leapmsg published by leap_motion node
 */
void processLeap(leap_motion_controller::LeapMotionOutput leap_data) {
  
  // NOTE: I am now using voice commands to switch between control modes because key taps are not very well understood by LeapSDK.
  // Right hand KeyTapGesture to switch between view modes
//   if (leap_data.right_hand_key_tap) {
//     ROS_INFO("LEAP has detected KeyTapGesture on right hand!");
//     temoto::ChangeTf switch_tf;
//     switch_tf.request.leap_motion_natural = !using_natural_control;		// request a change of view mode
//     if ( tf_client.call( switch_tf ) ) using_natural_control = !using_natural_control;// if request successful, change the value of view mode in this node
//   } // end if lefthand_key_tap

  
  // If position data is to be limited AND right_hand is detected AND right hand wasn't there before, toggle position_fwd_only.
  if (position_limited && leap_data.right_hand && !right_hand_before) {
    (position_fwd_only) ? position_fwd_only = false : position_fwd_only = true;	// toggles position_fwd_only value between true and false
    right_hand_before = true;							// sets right_hand_before true;
    ROS_INFO("RIGHT HAND DETECTED, position_fwd_only is now %d", position_fwd_only);
  } else if (leap_data.right_hand == false) right_hand_before = false;		// if right hand is not detected, set right_hand_before false;
  

  // Leap Motion Controller uses different coordinate orientation from the ROS standard for SIA5:
  //				SIA5	LEAP
  //		forward 	x	z
  //		right		y	x
  //		down		z	y

  // *** Following calculations are done in "leap_motion" frame, i.e. incoming leap_data pose is relative to leap_motion frame.

  // Working variable for potential target pole calculated from the hand pose coming from Leap Motion Controller (i.e. stamped to leap_motion frame).
  geometry_msgs::PoseStamped scaled_pose;
  // Header is copied without a change.
  scaled_pose.header = leap_data.header;
  // Reads the position of left palm in meters, amplifies to translate default motion; scales to dynamically adjust range; shifts for better usability.
  scaled_pose.pose.position.x = scale_by*AMP_HAND_MOTION*leap_data.left_palm_pose.position.x;
  scaled_pose.pose.position.y = scale_by*AMP_HAND_MOTION*(leap_data.left_palm_pose.position.y - HEIGHT_OF_ZERO);
  scaled_pose.pose.position.z = scale_by*AMP_HAND_MOTION*leap_data.left_palm_pose.position.z;
  // Orientation of left palm is copied unaltered, i.e., is not scaled
  scaled_pose.pose.orientation = leap_data.left_palm_pose.orientation;
  // Apply any relevant limitations to direction and/or orientation
  if (position_limited && position_fwd_only) {		// if position is limited and position_fwd_only is true
    scaled_pose.pose.position.x = 0;
    scaled_pose.pose.position.y = 0;
  } else if (position_limited && !position_fwd_only) {	// if position is limited and position_fwd_only is false, i.e. consider only sideways position change
    scaled_pose.pose.position.z = 0;
  }
  
  if (orientation_locked) {				// if palm orientation is to be ignored 
    // Overwrite orientation with identity quaternion.
    scaled_pose.pose.orientation.w = 1;
    scaled_pose.pose.orientation.x = 0;
    scaled_pose.pose.orientation.y = 0;
    scaled_pose.pose.orientation.z = 0;
  }
  
    // == Additional explanation of the above coordinates and origins ==
    // While the motion planner of sia5 uses base_link as origin, it is not convenient for the operator as s/he likely wishes to see the motion relative to end effector.
    // leap_motion frame has been attached to the end effector and, thus, hand motion is visualized relative to the pose of end effector.
    // If hand is moved to the origin of hand motion system, the target position is at current position. This makes the UI more intuitive for the operator, imho.
    // Scaling down drags the marker and the corresponding target position towards the actual origin of hand motion systems (i.e. Leap Motion Controller).
    // Leap Motion Controller has forward and left-right origins in the middle of the sensor display, which is OK, but up-down origin is on the keyboard.
    // Subtracting HEIGHT_OF_ZERO sets the up-down origin at HEIGHT_OF_ZERO above the keyboard and allows negative values which represent downward motion relative to end effector.

  // Setting properly scaled and limited pose as the desired_pose
  desired_pose.pose = scaled_pose.pose;
  desired_pose.header.frame_id = leap_data.header.frame_id;
  desired_pose.header.stamp = ros::Time::now();

  // Print position info to terminal
  printf("Scale motion by %f; move SIA5 by (x=%f, y=%f, z=%f) mm; position_fwd_only=%d\n",
	   AMP_HAND_MOTION*scale_by, desired_pose.pose.position.x*1000, desired_pose.pose.position.y*1000, desired_pose.pose.position.z*1000, position_fwd_only);

  return;
} // end processLeap

/** Callback function for powermate subscriber.
 *  It either reacts to dial being pressed or it updates the scaling factor.
 *  @param powermate temoto::Dial message published by powermate_dial node.
 */
void processPowermate(temoto::Dial powermate) {
  
  if (powermate.push_ev_occured) { 		// if push event (i.e. press or depress) has occured_occured
    if (powermate.pressed == 1) {		// if the dial has been pressed
      ROS_INFO("Powermate Dial has been pressed");
      callPlanAndMove(0x03);			// makes the service request to move the robot; requests plan&execute
    } else {					// if the dial has been depressed
      ROS_INFO("Powermate Dial has been depressed");
    }
    return;
  } else {					// if push event did not occur, it had to be dialing
    // Calculate step size depening on the current scale_by value. Negating direction means that clock-wise is zoom-in/scale-down and ccw is zoom-out/scale-up
    // The smaller the scale_by, the smaller the step. log10 gives the order of magnitude
    double step = (-powermate.direction)*pow( 10,  floor( log10( scale_by ) ) - 1 );
    if (step < -0.09) step = -0.01;		// special case for when scale_by is 1; to ensure that step is never larger than 0.01
    scale_by = scale_by + step;			// increase/decrease scale_by
    if (scale_by > 1) scale_by = 1;		// to ensure that scale_by is never larger than 1
    // Print position info to terminal
    printf("Scale motion by %f; move SIA5 by (x=%f, y=%f, z=%f) mm; position_fwd_only=%d\n",
	    AMP_HAND_MOTION*scale_by, desired_pose.pose.position.x*1000, desired_pose.pose.position.y*1000, desired_pose.pose.position.z*1000, position_fwd_only);
    return;
  } // else
} // end processPowermate()

/** Callback function for /temoto/end_effector_pose.
 *  Sets the received pose of an end effector as current_pose.
 *  @param end_eff_pose geometry_msgs::PoseStamped for end effector.
 */
void processEndEffector(geometry_msgs::PoseStamped end_effector_pose) {
  current_pose = end_effector_pose;		// sets the position of end effector as current pose
  return;
} // end processEndeffector()

/** Callback function for /temoto/voice_commands.
 *  Executes published voice command.
 *  @param voice_command contains the specific command as an unsigned integer.
 */
void executeVoiceCommand(temoto::Command voice_command) {
  // TODO check for namespace
  if (voice_command.cmd == 0xff) {
    ROS_INFO("Voice command received! Aborting ...");
    // TODO
  } else if (voice_command.cmd == 0x00) {
    ROS_INFO("Voice command received! Stopping ...");
    // TODO
  } else if (voice_command.cmd == 0x01) {
    ROS_INFO("Voice command received! Planning ...");
    callPlanAndMove(0x01);
  } else if (voice_command.cmd == 0x02) {
    ROS_INFO("Voice command received! Executing last plan ...");
    callPlanAndMove(0x02);
  } else if (voice_command.cmd == 0x03) {
    ROS_INFO("Voice command received! Planning and moving ...");
    callPlanAndMove(0x03);
  } else if (voice_command.cmd == 0xf1) {
    ROS_INFO("Voice command received! Planning to home ...");
    callPlanAndMoveNamedTarget(0x01, "home_pose");
  } else if (voice_command.cmd == 0xf3) {
    ROS_INFO("Voice command received! Planning and moving to home ...");
    callPlanAndMoveNamedTarget(0x03, "home_pose");
  } else if (voice_command.cmd == 0x10) {
    ROS_INFO("Voice command received! Using natural control mode ...");
    temoto::ChangeTf switch_tf;
    switch_tf.request.leap_motion_natural = true;			// request a change of view mode
    if ( tf_client.call( switch_tf ) ) using_natural_control = true;	// if request successful, change the value of view mode in this node
  } else if (voice_command.cmd == 0x11) {
    ROS_INFO("Voice command received! Using inverted control mode ...");
    temoto::ChangeTf switch_tf;
    switch_tf.request.leap_motion_natural = false;			// request a change of view mode
    if ( tf_client.call( switch_tf ) ) using_natural_control = false;	// if request successful, change the value of view mode in this node
  } else if (voice_command.cmd == 0x20) {
    ROS_INFO("Voice command received! Acknowledging 'Free directions' ...");
    position_limited = false;
    position_fwd_only = false;						// if input position is not limited, then there cannot be "forward only" mode
  } else if (voice_command.cmd == 0x21) {
    ROS_INFO("Voice command received! Acknowledging 'Limit directions' ...");
    position_limited = true;
  } else if (voice_command.cmd == 0x22) {
    ROS_INFO("Voice command received! Considering hand rotation/orientation ...");
    orientation_locked = false;
  } else if (voice_command.cmd == 0x23) {
    ROS_INFO("Voice command received! Ignoring hand rotation/orientation ...");
    orientation_locked = true;
  } else if (voice_command.cmd == 0x34) {				// Restart (delete all and add new) Cartesian waypoints 
    ROS_INFO("Voice command received! Started defining new Cartesian path ...");
    wayposes.clear();						// Clear existing wayposes
    wayposes.push_back(desired_pose.pose);			// Add a waypose
    computeCartesian(desired_pose.header.frame_id.c_str());	// Try to compute Cartesian path
  } else if (voice_command.cmd == 0x35) {				// Add Cartesian waypoint to the end 
    ROS_INFO("Voice command received! Adding a pose to Cartesian path ...");
    wayposes.push_back(desired_pose.pose);			// Add a waypose
    computeCartesian(desired_pose.header.frame_id.c_str());	// Try to compute Cartesian path
  } else if (voice_command.cmd == 0x36) {				// Remove the last Cartesian wayposet 
    ROS_INFO("Voice command received! Removing the last Cartesian waypose  ...");
    wayposes.pop_back();					// Remove last waypose
    computeCartesian(desired_pose.header.frame_id.c_str());	// Try to compute Cartesian path
  } else if (voice_command.cmd == 0x37) {				// Remove the last Cartesian wayposet 
    ROS_INFO("Voice command received! Removing the last Cartesian waypose  ...");
    wayposes.clear();						// Clear all wayposes
  }
  
  else {
    ROS_INFO("Voice command received! Unknown voice command.");
  }
  return;
}

/** Puts all the latest global variable values into temoto/status message.
 *  @return temoto::Status message.
 */
temoto::Status getStatus() {
  temoto::Status status;
  status.header.stamp = ros::Time::now();
  status.header.frame_id = "start_teleop";
  status.scale_by = scale_by;					// Latest scale_by value
  status.live_hand_pose = desired_pose;				// Latest hand pose stamped
  status.cartesian_wayposes = wayposes;				// Latest cartesian wayposes
  status.in_natural_control_mode = using_natural_control;
  status.orientation_free = !orientation_locked;
  status.position_unlimited = !position_limited;
  status.end_effector_position = current_pose.pose.position;	// should it be a POSE or even stamped pose instead?
  status.position_forward_only = position_fwd_only;			// needs renaming

  return status;
} // end getStatus()

/** Main method. */
int main(int argc, char **argv) {

  ros::init(argc, argv, "start_teleop");								// ROS init
  ros::NodeHandle n;											// ROS handle
 
  move_client = n.serviceClient<temoto::Goal>("temoto/move_robot_service");				// ROS client for /temoto/move_robot_service
  tf_client = n.serviceClient<temoto::ChangeTf>("temoto/change_tf");

  ros::Subscriber sub_powermate = n.subscribe<temoto::Dial>("griffin_powermate", 10, processPowermate);	// ROS subscriber on /griffin_powermate
  ros::Subscriber sub_leap = n.subscribe("leapmotion_general", 10, processLeap);			// ROS subscriber on /leapmotion_general
  ros::Subscriber sub_end_effector = n.subscribe("temoto/end_effector_pose", 0, processEndEffector);	// ROS subscriber on /temoto/end_effector_pose
  ros::Subscriber sub_voicecommands = n.subscribe("temoto/voice_commands", 1, executeVoiceCommand);	// ROS subscriber on /temoto/voice_commands
  
  ros::Publisher pub_status = n.advertise<temoto::Status>("temoto/status", 3);				// ROS publisher on /temoto/status
 
  ROS_INFO("Starting teleoperation ...");
  while (ros::ok()) {
    
    pub_status.publish( getStatus() );		// publish status current
      
    ros::spinOnce();				// spins once to update subscribers or something like that
  } // end while
  
  return 0;
} // end main