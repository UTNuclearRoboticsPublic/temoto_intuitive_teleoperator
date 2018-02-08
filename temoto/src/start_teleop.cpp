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

/** @file start_teleop.cpp
 * 
 *  @brief Central node for TEMOTO teleoperator. Subscribes to relevant messages, calls move and navigation interfaces, and publishes system status.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

#include "temoto/start_teleop.h"

/** Constructor for Teleoperator.
 */
Teleoperator::Teleoperator(ros::NodeHandle& n)
 : preplanned_sequence_client_("temoto/preplanned_sequence", true)  // true--> don't block the thread
  //, q_incremental_(0., 0., 0., 1.) // initially, add no rotation. New incremental rotation commands from e.g. the SpaceMouse will be added here.
{
  std::string primary_hand;
  n.param<std::string>("/temoto/primary_hand", primary_hand, "left");
  n.param<std::string>("/temoto/temoto_pose_cmd_topic", temoto_pose_cmd_topic_, "temoto_pose_cmd_topic");

  // By default navigation is turned OFF
  bool navigate;
  n.param<bool>("/temoto/navigate", navigate, false);
  // By default manipulation is turned ON
  n.param<bool>("/temoto/manipulate", manipulate_, true);
  // What pose input device?
  n.param<bool>("/temoto/leap_input", leap_input_, false);
  n.param<bool>("/temoto/spacenav_input", spacenav_input_, false);
  if (spacenav_input_ == leap_input_)
    ROS_ERROR_STREAM("[start_teleop/Teleoperator()] 1 and only 1 pose input device can be enabled.");

  pub_abort_ = n.advertise<std_msgs::String>("temoto/abort", 1, true);
  // TODO: parameterize this topic
  pub_jog_arm_cmds_ = n.advertise<geometry_msgs::TwistStamped>("/jog_arm_server/delta_jog_cmds", 1);
  pub_jog_base_cmds_ = n.advertise<geometry_msgs::Twist>("/temoto/base_cmd_vel", 1);

  sub_nav_spd_ = n.subscribe("/nav_collision_warning/spd_fraction", 1, &Teleoperator::nav_collision_cb, this);

  absolute_pose_cmd_.header.frame_id = "base_link";
  absolute_pose_cmd_.pose.position.x = 0; absolute_pose_cmd_.pose.position.y = 0; absolute_pose_cmd_.pose.position.z = 0;
  absolute_pose_cmd_.pose.orientation.x = 0; absolute_pose_cmd_.pose.orientation.y = 0; absolute_pose_cmd_.pose.orientation.z = 0; absolute_pose_cmd_.pose.orientation.w = 1;

  jog_twist_cmd_.header.frame_id = "temoto_end_effector";

  // Set initial scale on incoming commands
  setScale();

  // Free hand motion
  position_limited_ = false;
  position_fwd_only_ = false;
  

  // client for /temoto/move_robot_service
  move_robot_client_ = n.serviceClient<temoto::Goal>("temoto/move_robot_service");
  // client for /temoto/navigate_robot_srv
  navigate_robot_client_ = n.serviceClient<temoto::Goal>("temoto/navigate_robot_srv");
  // client for requesting change of transform between operator's hand frame and the robot's tool/planning frame
  tf_change_client_ = n.serviceClient<temoto::ChangeTf>("temoto/change_human2robot_tf");
  // client for starting and waiting on a preplanned sequence

  // Setting up control_state, i.e., whether teleoperator is controlling navigation, manipulation, or both.
  if (manipulate_ && navigate)
  {
    navT_or_manipF_ = false;		// if navigation AND manipulation are enabled, start out in manipulation mode.
    AMP_HAND_MOTION_ = 100;		// 100 for navigation
  }
  else if (navigate && !manipulate_)
  {
    navT_or_manipF_ = true;		// if only navigation is enabled, navT_or_manipF_ is TRUE
    AMP_HAND_MOTION_ = 100;		// 100 for navigation
  }
  else if (manipulate_ && !navigate)
  {
    navT_or_manipF_ = false;		// if only manipulation is enabled, navT_or_manipF_ is FALSE
    AMP_HAND_MOTION_ = 1;		// for manipulation
  }

  // Set up primary hand
  if (primary_hand == "right") {
    primary_hand_is_left_ = false;
  }
  else
  {
    primary_hand_is_left_ = true;
  }
}

/** Function that actually makes the service call to appropriate robot motion interface.
 *  Currently, there are MoveRobotInterface and NavigateRobotInterface.
 *  @param action_type determines what is requested from MoveGroup, i.e. PLAN, EXECUTE PLAN, or GO (aka plan and execute). 
 */
void Teleoperator::callRobotMotionInterface(std::string action_type)
{
  // Create a service request
  temoto::Goal motion;	
  motion.request.action_type = action_type;	// set action_type

  motion.request.goal_pose = absolute_pose_cmd_;

  // =================================================
  // === Calling NavigateRobotInterface ==============
  // =================================================
  if (navT_or_manipF_) // if in NAVIGATION mode
  {
    // If operator requested ABORT
    if (action_type == low_level_cmds::ABORT)
    {
      ROS_INFO("[start_teleop/callRobotMotionInterface] Requesting robot to stop navigation.");
      // make a service request to stop the robot
      if ( !navigate_robot_client_.call( motion ) )
      {
        ROS_ERROR("[start_teleop/callRobotMotionInterface] Failed to call temoto/navigate_robot_srv");
      }
      return;
    } // end: if (action_type == "abort")
    // If operator requested the robot to move to a goal pose
    else if (action_type == low_level_cmds::GO)
    {
      // Jogging
      if (in_jog_mode_)
      {
        // It feels better to control yaw via left-right motion on the SpaceNav
        if( spacenav_input_ )
        {
          jog_twist_cmd_.twist.angular.z = jog_twist_cmd_.twist.linear.y;
        }

        // Scale velocity if close to obstacle
        jog_twist_cmd_.twist.linear.x *= nav_speed_fraction_;
        jog_twist_cmd_.twist.linear.y *= nav_speed_fraction_;
        jog_twist_cmd_.twist.linear.z *= nav_speed_fraction_;

        jog_twist_cmd_.twist.angular.x *= nav_speed_fraction_;
        jog_twist_cmd_.twist.angular.y *= nav_speed_fraction_;
        jog_twist_cmd_.twist.angular.z *= nav_speed_fraction_;

        pub_jog_base_cmds_.publish(jog_twist_cmd_.twist);
        return;
      }

      double lm_roll, lm_pitch, lm_yaw;

      tf::Quaternion quat_current_cmd_frame;
      tf::quaternionMsgToTF(motion.request.goal_pose.pose.orientation, quat_current_cmd_frame);
      tf::Matrix3x3(quat_current_cmd_frame).getRPY(lm_roll, lm_pitch, lm_yaw);
      
      // Translate current_cmd_frame pose to base_link
      geometry_msgs::PoseStamped goal_in_baselink;
      // absolute_pose_cmd_ is given in current_cmd_frame frame and shall be transformed into base_link
      transform_listener_.transformPose("base_link", motion.request.goal_pose, goal_in_baselink);
    

      double bl_roll, bl_pitch, bl_yaw;
      // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
      tf::Quaternion quat_base_link;
      tf::quaternionMsgToTF(goal_in_baselink.pose.orientation, quat_base_link);

      // the tf::Quaternion has a method to access roll, pitch, and yaw
      tf::Matrix3x3(quat_base_link).getRPY(bl_roll, bl_pitch, bl_yaw);
      
      quat_base_link.setRPY(0., 0., bl_yaw);
      quat_base_link.normalize();
      tf::quaternionTFToMsg(quat_base_link, goal_in_baselink.pose.orientation);
      
      // Set goal_in_baselink as the target goal
      motion.request.goal_pose = goal_in_baselink;
      
      // make a service request to navigate_robot_srv
      if ( !navigate_robot_client_.call( motion ) )
      {
	      ROS_ERROR("[start_teleop/callRobotMotionInterface] Failed to call temoto/navigate_robot_srv");
      }
      return;
    } // else if (action_type == "go")
  } // if (navT_or_manipF_)
  // =================================================
  // === Calling MoveRobotInterface ==================
  // =================================================
  else if ( !navT_or_manipF_ )    // If Teleoperator is in MANIPULATION mode
  { 
    // Jogging
    if (in_jog_mode_)
    {
      pub_jog_arm_cmds_.publish(jog_twist_cmd_);
      return;
    }
    // Point-to-point motion
    else
    {
      // Adjust orientation for inverted control mode
      if ( !naturalT_or_invertedF_control_)
      {
        // rotate orientation 180 around UP-vector
        motion.request.goal_pose.pose.orientation = oneEightyAroundOperatorUp( motion.request.goal_pose.pose.orientation ) ;
      }
         
      // Call temoto/move_robot_service
      if ( !move_robot_client_.call( motion ) )
      {
        ROS_ERROR("[start_teleop/callRobotMotionInterface] Failed to call temoto/move_robot_service");
      }
      return;
    }
  }
  // =================================================
  // === Unexpected case == NEITHER NAV NOR MANIP ====
  // =================================================
  else
  {
    ROS_INFO("[start_teleop/callRobotMotionInterface] Request unavailable for current list of robot interfaces.");
  }

  // Reset the hand marker to be at the EE
  reset_integrated_cmds_ = true;

  return;
} // end callRobotMotionInterface

/** Function that actually makes the service call to /temoto/move_robot_service.
 *  @param action_type determines what is requested from MoveGroup, i.e. PLAN, EXECUTE PLAN, or GO (aka plan and execute).
 *  @param named_target uses this named target for target pose.
 */
// void Teleoperator::callPlanAndMoveNamedTarget(uint8_t action_type, std::string named_target)
void Teleoperator::callRobotMotionInterfaceWithNamedTarget(std::string action_type, std::string named_target)
{
  if ( !navT_or_manipF_ )			// currenly implemented only for manipulation mode
  {
    temoto::Goal motion;			// create a service request
    motion.request.action_type = action_type;	// set action_type
    motion.request.named_target = named_target;	// set named_target as the goal
    
    // call for the service to move robot group
    if ( move_robot_client_.call( motion ) )
    {
      ROS_INFO("Successfully called temoto/move_robot_service");
    }
    else
    {
      ROS_ERROR("Failed to call temoto/move_robot_service");
    }
  }
  else
  {
    ROS_INFO("[start_teleop/callRobotMotionInterfaceWithNamedTarget] Not available for NavigateRobotInterface.");
  }
  return;
} // end callRobotMotionInterfaceWithNamedTarget

/** Alters a quaternion so that the pitch is preserved while roll and yaw are set to zero.
 * 
 *  @param quaternion_msg incoming quaternion as a geometry_msgs.
 *  @return normalized quaternion as a geometry_msgs that contains only pitch information.
 */
geometry_msgs::Quaternion Teleoperator::extractOnlyPitch(geometry_msgs::Quaternion quaternion_msg)
{
  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(quaternion_msg, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  
  // modify quaternion by setting roll and yaw to zero
//   ROS_INFO("[start_teleop/extractOnlyPitch] PITCH: %.2f", pitch);
  quat.setRPY(0, pitch, 0);
  quat.normalize();
  
  // the modified quaternion is converted to a geometry_msgs::Quaternion
  geometry_msgs::Quaternion outbound_msg;
  tf::quaternionTFToMsg(quat, outbound_msg);
  return outbound_msg;
}

/** Callback function for relative position commands.
 *  Add the new command to the current RViz marker pose.
 *  @param pose_cmd sensor_msgs::Joy a joystick command containing pose and button info
 */
void Teleoperator::processJoyCmd(sensor_msgs::Joy pose_cmd)
{
  // Ensure incoming data is in the right frame
  if ( current_pose_.header.frame_id != "base_link" )
  {
    ROS_WARN_THROTTLE(2, "[start_teleop] The current pose is not being published in the base_link frame.");
    return;
  }

  // If rotational components >> translational, ignore translation (and vice versa)
  double trans_mag = pow( pose_cmd.axes[0]*pose_cmd.axes[0] + pose_cmd.axes[1]*pose_cmd.axes[1] + pose_cmd.axes[2]*pose_cmd.axes[2], 2 );
  double rot_mag = pow( pose_cmd.axes[3]*pose_cmd.axes[3] + pose_cmd.axes[4]*pose_cmd.axes[4] + pose_cmd.axes[5]*pose_cmd.axes[5], 2);
  if ( trans_mag > 2.*rot_mag )
  {
    pose_cmd.axes[3] = 0.; pose_cmd.axes[4] = 0.; pose_cmd.axes[5] = 0.;
  }
  else if ( rot_mag > 2.*trans_mag )
  {
    pose_cmd.axes[0] = 0.; pose_cmd.axes[1] = 0.; pose_cmd.axes[2] = 0.;
  }

  // Should we reset the command integrations?
  // Can be used to reset the hand marker, or when switching betw. manip & nav modes
  if (reset_integrated_cmds_)
  {
    absolute_pose_cmd_.pose.orientation.x = 0.; absolute_pose_cmd_.pose.orientation.y = 0.; absolute_pose_cmd_.pose.orientation.z = 0.; absolute_pose_cmd_.pose.orientation.w = 1.;

    absolute_pose_cmd_.pose.position.x = 0.; absolute_pose_cmd_.pose.position.y = 0.; absolute_pose_cmd_.pose.position.z = 0.;

    incremental_orientation_cmd_.x = 0.; incremental_orientation_cmd_.y = 0.; incremental_orientation_cmd_.z = 0.;
    incremental_position_cmd_.x = 0.; incremental_position_cmd_.y = 0.; incremental_position_cmd_.z = 0.;

    // Reset the flag
    reset_integrated_cmds_ = false;
  }

  ///////////////////////////////////////////////////////////
  // JOGGING
  // Can use the same type of command for arm and base teleop
  ///////////////////////////////////////////////////////////
  if (in_jog_mode_)
  {
    jog_twist_cmd_.header.stamp = ros::Time::now();
    jog_twist_cmd_.twist.linear.x = pos_scale_*pose_cmd.axes[0];
    jog_twist_cmd_.twist.linear.y = pos_scale_*pose_cmd.axes[1];
    jog_twist_cmd_.twist.linear.z = pos_scale_*pose_cmd.axes[2];
    jog_twist_cmd_.twist.angular.x = rot_scale_*pose_cmd.axes[3];
    jog_twist_cmd_.twist.angular.y = rot_scale_*pose_cmd.axes[4];
    jog_twist_cmd_.twist.angular.z = rot_scale_*pose_cmd.axes[5];
  }

  ////////////////////////////
  // POINT-TO-POINT NAVIGATION
  ////////////////////////////
  if ( navT_or_manipF_ && !in_jog_mode_ )
  {
    // ORIENTATION
    // new incremental yaw command
    // For nav mode, we want to stay in the plane ==> ignore roll & pitch
    incremental_orientation_cmd_.z = rot_scale_*pose_cmd.axes[5];  // about z axis

    tf::Quaternion q_previous_cmd, q_incremental;
    q_incremental = tf::createQuaternionFromRPY(incremental_orientation_cmd_.x, incremental_orientation_cmd_.y, incremental_orientation_cmd_.z);

    // Add the incremental command to the previously commanded orientation
    quaternionMsgToTF(absolute_pose_cmd_.pose.orientation , q_previous_cmd);  // Get the current orientation
    q_previous_cmd *= q_incremental;  // Calculate the new orientation
    q_previous_cmd.normalize();
    quaternionTFToMsg(q_previous_cmd, absolute_pose_cmd_.pose.orientation);  // Stuff it back into the pose cmd

    // POSITION
    // Ignore Z (out of plane)

    // Integrate the incremental cmd. It persists even if the robot moves
    incremental_position_cmd_.x += pos_scale_*pose_cmd.axes[0];  // X is fwd/back in base_link
    incremental_position_cmd_.y += pos_scale_*pose_cmd.axes[1];   // Y is left/right

    // Incoming position cmds are in the spacenav frame
    // So convert them to base_link like everything else

    geometry_msgs::Vector3Stamped incoming_position_cmd;
    incoming_position_cmd.header.frame_id = "spacenav";
    incoming_position_cmd.vector.x = incremental_position_cmd_.x; incoming_position_cmd.vector.y  = incremental_position_cmd_.y; incoming_position_cmd.vector.z = incremental_position_cmd_.z;
    if ( transform_listener_.waitForTransform("spacenav", "base_link", ros::Time::now(), ros::Duration(0.01)) )
      transform_listener_.transformVector("base_link", incoming_position_cmd, incoming_position_cmd);
    else
    {
      ROS_WARN_THROTTLE(2, "[temoto/start_teleop] TF between base_link and spacenav timed out.");
    }

    // Ignore Z
    incoming_position_cmd.vector.z = 0.;

    // Unlike manipulation mode, the center of the robot base is defined to be the origin (0,0,0). So we don't need to add anything else here.
    absolute_pose_cmd_.pose.position.x = incoming_position_cmd.vector.x;
    absolute_pose_cmd_.pose.position.y = incoming_position_cmd.vector.y;
    absolute_pose_cmd_.pose.position.z = incoming_position_cmd.vector.z;
  }


  //////////////////////////////
  // POINT-TO-POINT MANIPULATION
  //////////////////////////////
  if ( !navT_or_manipF_ && !in_jog_mode_ )
  {
    // Integrate for point-to-point motion

    // ORIENTATION
    // new incremental rpy command
    incremental_orientation_cmd_.x = rot_scale_*pose_cmd.axes[3];  // about x axis
    incremental_orientation_cmd_.y = rot_scale_*pose_cmd.axes[4];  // about y axis
    incremental_orientation_cmd_.z = rot_scale_*pose_cmd.axes[5];  // about z axis

    tf::Quaternion q_previous_cmd, q_incremental;
    q_incremental = tf::createQuaternionFromRPY(incremental_orientation_cmd_.x, incremental_orientation_cmd_.y, incremental_orientation_cmd_.z);

    // Add the incremental command to the previously commanded orientation
    quaternionMsgToTF(absolute_pose_cmd_.pose.orientation , q_previous_cmd);  // Get the current orientation
    q_previous_cmd *= q_incremental;  // Calculate the new orientation
    q_previous_cmd.normalize();
    quaternionTFToMsg(q_previous_cmd, absolute_pose_cmd_.pose.orientation);  // Stuff it back into the pose cmd

    // POSITION

    // Integrate the incremental cmd. It persists even if the robot moves
    incremental_position_cmd_.x += pos_scale_*pose_cmd.axes[0];  // X is fwd/back in base_link
    incremental_position_cmd_.y += pos_scale_*pose_cmd.axes[1];   // Y is left/right
    incremental_position_cmd_.z += pos_scale_*pose_cmd.axes[2];  // Z is up/down

    // Incoming position cmds are in the spacenav frame
    // So convert them to base_link like everything else

    geometry_msgs::Vector3Stamped incoming_position_cmd;
    incoming_position_cmd.header.frame_id = "spacenav";
    incoming_position_cmd.vector.x = incremental_position_cmd_.x; incoming_position_cmd.vector.y  = incremental_position_cmd_.y; incoming_position_cmd.vector.z = incremental_position_cmd_.z;
    if ( transform_listener_.waitForTransform("spacenav", "base_link", ros::Time::now(), ros::Duration(0.01)) )
    {
      transform_listener_.transformVector("base_link", incoming_position_cmd, incoming_position_cmd);
    }
    else
    {
      ROS_WARN_THROTTLE(2, "[temoto/start_teleop] transform_listener_ waitForTransform returned false");
    }

    absolute_pose_cmd_.pose.position.x = current_pose_.pose.position.x + incoming_position_cmd.vector.x;
    absolute_pose_cmd_.pose.position.y = current_pose_.pose.position.y + incoming_position_cmd.vector.y;
    absolute_pose_cmd_.pose.position.z = current_pose_.pose.position.z + incoming_position_cmd.vector.z;
  }

  return;
} // end processJoyCmd

/** Callback function for absolute position commands.
 *  For now, this is based on a LeapMotion controller.
 *  It updates target pose based on latest left palm pose (any scaling and/or relevant limitations are also being applied).
 *  Presence of the right hand is used to lock and unlock forward motion.
 *  KEY_TAP gesture detection is currenly unimplemented.
 *  @param leap_data temoto::Leapmsg published by leap_motion node
 */

// == Explanation of the frames ==
// For navigation, leap_motion_frame is attached to base_link.
// For manipulation, leap_motion_frame is attached to the end effector.
// This switch occurs in graphics_and_frames.cpp
// Scaling down drags the marker and the corresponding target position towards the actual origin of hand motion systems (i.e. Leap Motion Controller).
// Leap Motion Controller has forward and left-right origins in the middle of the sensor display, which is OK, but up-down origin is on the keyboard.
// Subtracting HEIGHT_OF_ZERO sets the up-down origin at HEIGHT_OF_ZERO above the keyboard and allows negative values which represent downward motion relative to end effector.

void Teleoperator::processLeapCmd(leap_motion_controller::Set leap_data)
{
  // First, set up primary and secondary hand.
  geometry_msgs::Pose primary_hand;
  geometry_msgs::Pose secondary_hand;
  bool primary_hand_is_present;
  bool secondary_hand_is_present;
  if ( primary_hand_is_left_ )
  {
    // copy left hand data to primary_hand and right hand data to secondary_hand
    primary_hand = leap_data.left_hand.palm_pose.pose;
    secondary_hand = leap_data.right_hand.palm_pose.pose;
    primary_hand_is_present = leap_data.left_hand.is_present;
    secondary_hand_is_present = leap_data.right_hand.is_present;
  }
  else
  {
    // copy right hand data to primary_hand and left hand data to secondary_hand
    primary_hand = leap_data.right_hand.palm_pose.pose;
    secondary_hand = leap_data.left_hand.palm_pose.pose;
    primary_hand_is_present = leap_data.right_hand.is_present;
    secondary_hand_is_present = leap_data.left_hand.is_present;
  }
  
  // If position data is to be limited AND secondary hand is detected AND secondary hand wasn't there before, toggle position_fwd_only_.
  if (position_limited_ && secondary_hand_is_present && !secondary_hand_before_)
  {
    (position_fwd_only_) ? position_fwd_only_ = false : position_fwd_only_ = true;	// toggles position_fwd_only value between TRUE and FALSE
    secondary_hand_before_ = true;							// sets secondary_hand_before_ TRUE;
    ROS_INFO("SECONDARY HAND DETECTED, position_fwd_only_ is now %d", position_fwd_only_);
  }
  else if (secondary_hand_is_present == false) secondary_hand_before_ = false;		// if secondary hand is not detected, set secondary_hand_before_ FALSE;
  

  // Leap Motion Controller coordinate orientation:
  //		        LEAP
  //		forward  -z
  //		right		 x
  //		up   		 y

  // *** Following calculations are done in "current_cmd_frame" frame, i.e. incoming leap_data pose is relative to current_cmd_frame frame.

  // Working variable for potential target pose calculated from the hand pose coming from Leap Motion Controller (i.e. stamped to current_cmd_frame frame).
  geometry_msgs::PoseStamped scaled_pose;
  // Header is copied without a change.
  scaled_pose.header = leap_data.header;
  // Reads the position of primary palm in meters, amplifies to translate default motion; scales to dynamically adjust range; offsets for better usability.
  scaled_pose.pose.position.x = pos_scale_*AMP_HAND_MOTION_*(primary_hand.position.x - OFFSET_X_);
  scaled_pose.pose.position.y = pos_scale_*AMP_HAND_MOTION_*(primary_hand.position.y - OFFSET_Y_);
  scaled_pose.pose.position.z = pos_scale_*AMP_HAND_MOTION_*(primary_hand.position.z - OFFSET_Z_);


  ////////////////////////////////////////////
  // ORIENTATION
  // of primary palm is not scaled
  ////////////////////////////////////////////

  // Extract the rpy from the command and save it to be applied after coordinate frame is corrected
  tf::Quaternion q_cmd(primary_hand.orientation.x, primary_hand.orientation.y, primary_hand.orientation.z, primary_hand.orientation.w);

  // Set orientation to match SpaceNav frame, no user-commanded rotation applied yet.
  scaled_pose.pose.orientation.x = -0.5; scaled_pose.pose.orientation.y = 0.5; scaled_pose.pose.orientation.z = 0.5; scaled_pose.pose.orientation.w = 0.5;

  // Appy the user-commanded rpy
  tf::Quaternion q_orig, q_new;
  quaternionMsgToTF(scaled_pose.pose.orientation , q_orig);  // Get the unmodified orientation of 'scaled_pose'
  q_new = q_cmd*q_orig;  // Calculate the new orientation
  q_new.normalize();
  quaternionTFToMsg(q_new, scaled_pose.pose.orientation);  // Stuff the new rotation back into the pose. This requires conversion into a msg type 


  // Applying relevant limitations to direction and/or orientation
  if (navT_or_manipF_ && primary_hand_is_present)				// if in navigation mode, UP-DOWN motion of the hand is to be ignored
  {
    scaled_pose.pose.position.y = 0;
    // preserve only pitch (rotation around UP vector) of hand orientation
    scaled_pose.pose.orientation = extractOnlyPitch( scaled_pose.pose.orientation );
  }
  else if (position_limited_ && position_fwd_only_) 	// if position is limited and position_fwd_only_ is true
  {
    scaled_pose.pose.position.x = 0;
    scaled_pose.pose.position.y = 0;
  }
  else if (position_limited_ && !position_fwd_only_)	// if position is limited and position_fwd_only_ is false, i.e. consider only sideways position change
  {
    scaled_pose.pose.position.z = 0;
  }

  if (orientation_locked_)				// if palm orientation is to be ignored 
  {
    // Overwrite orientation with identity quaternion.
    scaled_pose.pose.orientation.x = 0;
    scaled_pose.pose.orientation.y = 0;
    scaled_pose.pose.orientation.z = 0;
    scaled_pose.pose.orientation.w = 1;
  }

  // Setting properly scaled and limited pose as the absolute_pose_cmd_
  absolute_pose_cmd_.pose = scaled_pose.pose;
  absolute_pose_cmd_.header.frame_id = scaled_pose.header.frame_id;
  absolute_pose_cmd_.header.stamp = ros::Time(0);

  return;
} // end processLeapCmd

/** Callback function for Griffin Powermate events subscriber.
 *  It either reacts to push button being pressed or it updates the scaling factor.
 *  @param powermate temoto::Dial message published by griffin_powermate node.
 */
void Teleoperator::processPowermate(griffin_powermate::PowermateEvent powermate)
{
  if (powermate.push_state_changed)		// if push event (i.e. press or depress) has occured_occured
  {
    if (powermate.is_pressed)			// if the push button has been pressed
    {
      callRobotMotionInterface(low_level_cmds::EXECUTE);		// makes the service request to move the robot, according to prior plan
    }
    return;
  }
  else						// if push event did not occur, it had to be turning
  {
    // Calculate step size depending on the current pos_scale_ value. Negating direction means that clock-wise is zoom-in/scale-down and ccw is zoom-out/scale-up
    // The smaller the pos_scale_, the smaller the step. log10 gives the order of magnitude
    double step = (-powermate.direction)*pow( 10,  floor( log10( pos_scale_ ) ) - 1. );
    if (step < -0.09) step = -0.01;		// special case for when scale is 1; to ensure that step is never larger than 0.01
    pos_scale_ = pos_scale_ + step;		// increase/decrease scale

    step = (-powermate.direction)*pow( 10,  floor( log10( rot_scale_ ) ) - 1. );
    rot_scale_ = rot_scale_ + step;

    // cap the scales
    if (pos_scale_ > 1.) pos_scale_ = 1.;
    if (rot_scale_ > 0.01) rot_scale_ = 0.01;

    return;
  } // else
} // end processPowermate()

/** Callback function for /temoto/end_effector_pose.
 *  Sets the received pose of an end effector as current_pose_.
 *  @param end_eff_pose geometry_msgs::PoseStamped for end effector.
 */
void Teleoperator::updateEndEffectorPose(geometry_msgs::PoseStamped end_effector_pose)
{
  current_pose_ = end_effector_pose;		// sets the position of end effector as current pose
  return;
} // end processEndeffector()


/** Callback function for /nav_collision_warning/spd_fraction
 *  Scales teleoperated velocity cmds when an obstacle is close.
 *  @param msg from the nav_collision_warning node
 */
void Teleoperator::nav_collision_cb(const std_msgs::Float64::ConstPtr& msg)
{
  nav_speed_fraction_ = msg->data;
}


/** Callback function for /temoto/preplanned_sequence/result
 *  Sets the "executing_preplanned_sequence_" flag. If true, other Temoto commands are blocked.
 * @param sequence_result true indicates a preplanned sequence has completed
*/
void Teleoperator::updatePreplannedFlag(temoto::PreplannedSequenceActionResult sequence_result)
{
  // Successfully completed the preplanned sequence ==> false for blocking other temoto commands
  executing_preplanned_sequence_ = !sequence_result.result.success;
  if ( !executing_preplanned_sequence_ )
    ROS_INFO_STREAM("The preplanned sequence is complete.");
}


/** This function fixes the quaternion of a pose input in 'inverted control mode'.
 *  @param operator_input_quaternion_msg is the original quaternion during 'inverted control mode'.
 *  @return geometry_msgs::Quaternion quaternion_msg_out is the proper quaternion current_cmd_frame frame.
 */
geometry_msgs::Quaternion Teleoperator::oneEightyAroundOperatorUp(geometry_msgs::Quaternion operator_input_quaternion_msg)
{
  geometry_msgs::Quaternion quaternion_msg_out;
  // Adjust orientation for inverted control mode
  tf::Quaternion invert_rotation(0, 1, 0, 0);				// 180° turn around Y axis, UP in current_cmd_frame frame
  tf::Quaternion operator_input;					// incoming operator's palm orientation
  tf::quaternionMsgToTF(operator_input_quaternion_msg, operator_input);	// convert incoming quaternion msg to tf quaternion
  tf::Quaternion final_rotation = operator_input * invert_rotation;	// apply invert_rotation to incoming palm rotation
  final_rotation.normalize();						// normalize resulting quaternion
  tf::quaternionTFToMsg(final_rotation, quaternion_msg_out);		// convert tf quaternion to quaternion msg
  return quaternion_msg_out;
}


/** Callback function for /temoto/voice_commands.
 *  Executes published voice command.
 *  @param voice_command contains the specific command as an unsigned integer.
 */
void Teleoperator::processVoiceCommand(temoto::Command voice_command)
{
  //////////////////////////////////////////////////
  //  Stop jogging (jogging preempts other commands)
  //////////////////////////////////////////////////

  if (voice_command.cmd_string == "stop jogging")
  {
    ROS_INFO("Switching out of jog mode");
    in_jog_mode_ = false;
    setScale();

    return;
  }

  /////////////////////////
  //  Handle ABORT commands
  /////////////////////////
  if (voice_command.cmd_string == "stop stop")
  {
    ROS_INFO("Stopping ...");
    // Stop motions within temoto
    callRobotMotionInterface(low_level_cmds::ABORT);

    // Publish to tell non-Temoto actions to abort
    std_msgs::String s;
    s.data = low_level_cmds::ABORT;
    pub_abort_.publish(s);

    return;
  }
  else  // No need to abort
  {
    std_msgs::String s;
    s.data = low_level_cmds::NO_ABORT;
    pub_abort_.publish(s);
  }


  ////////////////////////////////////
  //  Handle all commands except ABORT
  ////////////////////////////////////

  // Normal temoto motions are OK if a preplanned sequence isn't running
  if (executing_preplanned_sequence_==true)
  {
    ROS_INFO_STREAM("Executing a preplanned sequence. Other Temoto actions are blocked.");
    return;
  }


  if ( voice_command.cmd_string == "manipulation" )  // Switch over to manipulation (MoveIt!) mode
  {
    // If not already in manipulation mode
    if ( navT_or_manipF_==true )
    {
      reset_integrated_cmds_ = true;  // Flag that the integrated cmds need to be reset
      ROS_INFO("Going into MANIPULATION mode  ...");
      AMP_HAND_MOTION_ = 1;
      temoto::ChangeTf switch_human2robot_tf;
      switch_human2robot_tf.request.navigate = false; // request a change of control mode
      switch_human2robot_tf.request.first_person_perspective = naturalT_or_invertedF_control_;  // preserve current control perspective
      tf_change_client_.call( switch_human2robot_tf );
      navT_or_manipF_ = false;
      setScale();
    }
    else
      ROS_INFO("Already in manipulation mode.");
    return;
  }
  else if ( voice_command.cmd_string == "navigation" )  // Switch over to navigation mode
  {
    // If not already in nav mode
    if ( navT_or_manipF_==false )
    {
      reset_integrated_cmds_ = true;  // Flag that the integrated cmds need to be reset
      ROS_INFO("Going into NAVIGATION mode  ...");
      AMP_HAND_MOTION_ = 100;
      temoto::ChangeTf switch_human2robot_tf;
      switch_human2robot_tf.request.navigate = true;  // request a change of control mode
      switch_human2robot_tf.request.first_person_perspective = naturalT_or_invertedF_control_;

      tf_change_client_.call( switch_human2robot_tf );
      navT_or_manipF_ = true;
      setScale();
    }
    else
      ROS_INFO("Already in navigation mode.");
    return;
  }


  if(voice_command.cmd_string == "close gripper")  // Close the gripper - a preplanned sequence
  {
    ROS_INFO("Closing the gripper ...");
    Teleoperator::triggerSequence(voice_command);
    return;
  }
  else if(voice_command.cmd_string == "open gripper")  // Open the gripper - a preplanned sequence
  {
    ROS_INFO("Opening the gripper ...");
    Teleoperator::triggerSequence(voice_command);
    return;
  }


  if ( in_jog_mode_ )  // Avoid planning, executing, etc. while in jog mode
    return;


  else  // There's nothing blocking any arbitrary command
  {
    if (voice_command.cmd_string == "jog mode")
    {
      ROS_INFO("Switching to jog mode");
      in_jog_mode_ = true;
      setScale();
      return;
    }
    else if (voice_command.cmd_string == "robot please plan")
    {
      ROS_INFO("Planning ...");
      callRobotMotionInterface(low_level_cmds::PLAN);
      return;
    }
    else if (voice_command.cmd_string == "robot please execute")
    {
      ROS_INFO("Executing last plan ...");
      callRobotMotionInterface(low_level_cmds::EXECUTE);

      // Reset the incremental commands integrations
      incremental_position_cmd_.x = 0.;
      incremental_position_cmd_.y = 0.;
      incremental_position_cmd_.z = 0.;
      return;
    }
    else if (voice_command.cmd_string == "robot plan and go")
    {
      ROS_INFO("Planning and moving ...");
      callRobotMotionInterface(low_level_cmds::GO);
      return;
    }
    else if (voice_command.cmd_string == "robot plan home")
    {
      ROS_INFO("Planning to home ...");
      callRobotMotionInterfaceWithNamedTarget(low_level_cmds::PLAN, "home_pose");
      return;
    }
    else if (voice_command.cmd_string == "robot plan home")
    {
      ROS_INFO("Planning to home ...");
      callRobotMotionInterfaceWithNamedTarget(low_level_cmds::PLAN, "home_pose");
      return;
    }    
    else if (voice_command.cmd_string == "robot please go home")
    {
      ROS_INFO("Planning and moving to home ...");
      callRobotMotionInterfaceWithNamedTarget(low_level_cmds::GO, "home_pose");
      return;
    }
    else if (voice_command.cmd_string == "natural control mode")
    {
      ROS_INFO("Using natural control perspective ...");
      temoto::ChangeTf switch_human2robot_tf;
      switch_human2robot_tf.request.first_person_perspective = true;	// request a change of control perspective
      switch_human2robot_tf.request.navigate = navT_or_manipF_;	// preserve current navigation/manipulation mode
      // if service request successful, change the value of control perspective in this node
      tf_change_client_.call( switch_human2robot_tf );
      naturalT_or_invertedF_control_ = true;
      return;
    }
    else if (voice_command.cmd_string == "inverted control mode")
    {
      ROS_INFO("Using inverted control perspective ...");
      temoto::ChangeTf switch_human2robot_tf;
      switch_human2robot_tf.request.first_person_perspective = false;	// request a change of control perspective
      switch_human2robot_tf.request.navigate = navT_or_manipF_;	// preserve current navigation/manipulation mode

      tf_change_client_.call( switch_human2robot_tf );
      naturalT_or_invertedF_control_ = false;
      return;
    }
    else if (voice_command.cmd_string == "free directions")
    {
      ROS_INFO("Acknowledging 'Free directions' ...");
      position_limited_ = false;
      position_fwd_only_ = false;				// once input position is not limited, there cannot be "forward only" mode
      return;
    }
    else if (voice_command.cmd_string == "limit directions")
    {
      ROS_INFO("Acknowledging 'Limit directions' ...");
      position_limited_ = true;
      return;
    }
    else if (voice_command.cmd_string == "consider rotation")
    {
      ROS_INFO("Considering hand rotation/orientation ...");
      orientation_locked_ = false;
      return;
    }
    else if (voice_command.cmd_string == "ignore rotation")
    {
      ROS_INFO("Ignoring hand rotation/orientation ...");
      orientation_locked_ = true;
      return;
    }
    else if (voice_command.cmd_string == "robot please approach")  // Move the robot base and arm to reach the marker pose
    {
      ROS_INFO("Approaching ...");
      Teleoperator::triggerSequence(voice_command);
      return;
    }
    else if (voice_command.cmd_string == "go to laser scan")  // Move the left arm to a pose for a laser scan
    {
      ROS_INFO("Moving to a pose for laser scanning ...");
      Teleoperator::triggerSequence(voice_command);
      return;
    }
    else if (voice_command.cmd_string == "robot please scan")  // start a lidar scan
    {
      ROS_INFO("Beginning a lidar scan ...");
      Teleoperator::triggerSequence(voice_command);
      return;
    }
    else if (voice_command.cmd_string == "robot push button")  // start a lidar scan
    {
      ROS_INFO("Pushing a button ...");
      Teleoperator::triggerSequence(voice_command);
      return;
    }
  
    else
    {
      ROS_INFO("Unknown voice command.");
    }
  }  // End of handling non-Abort commands

  return;
}

void Teleoperator::triggerSequence(temoto::Command& voice_command)
{
  temoto::PreplannedSequenceGoal goal;
  goal.sequence_name = voice_command.cmd_string;
  //while ( !preplanned_sequence_client_.waitForServer( ros::Duration(1.) ))
  //{
  //  ROS_INFO_STREAM("[start_teleop] Waiting for the preplanned action server.");
  //}
  preplanned_sequence_client_.sendGoal(goal);

  // Flag that a preplanned sequence is running, so pause most other commands (except Abort)
  executing_preplanned_sequence_ = true;
}

/** Puts the latest private variable values into temoto/status message.
 *  This is used in marker publication.
 *  @return temoto::Status message.
 */
temoto::Status Teleoperator::setStatus()
{
  temoto::Status status;
  status.scale_by = pos_scale_;			// Latest pos_scale_ value


  status.commanded_pose = absolute_pose_cmd_;

  status.in_natural_control_mode = naturalT_or_invertedF_control_;
  status.orientation_free = !orientation_locked_;
  status.position_unlimited = !position_limited_;
  status.end_effector_pose = current_pose_;			// latest known end effector pose

  status.position_forward_only = position_fwd_only_;
  status.in_navigation_mode = navT_or_manipF_;

  return status;
} // end setStatus()

void Teleoperator::setScale()
{
  // No scaling seems to be needed with LeapMotion
  if (leap_input_)
  {
    pos_scale_ = 1.;
    rot_scale_ = 1.;
  }

  // Scaling for incremental SpaceNav cmds
  else
    if (navT_or_manipF_)
    {
      if (in_jog_mode_)  // nav, jog mode
      {
        pos_scale_ = 0.3;
        rot_scale_ = 0.2;
      }
      else  // nav, pt-to-pt mode
      {
        pos_scale_ = 0.032;
        rot_scale_ = 0.05;
      }
    }
    else // manipulation
    {
      if (in_jog_mode_)  // manipulate, jog mode
      {
        pos_scale_ = 1.;
        rot_scale_ = 1.;
      }
      else  // manipulate, pt-to-pt mode
      {
        pos_scale_ = 0.012;
        rot_scale_ = 0.015;
      }
    }
}


// MAIN
int main(int argc, char **argv)
{
  ros::init(argc, argv, "start_teleop");

  ros::NodeHandle n;

  ros::Rate node_rate(30.);
  
  // Instance of Teleoperator
  Teleoperator temoto_teleop(n);

  // Make a request for initial human2robot TF set-up
  temoto::ChangeTf initial_human2robot_tf;
  if (temoto_teleop.manipulate_)  // Start in manipulation mode, if it's available
    initial_human2robot_tf.request.navigate = false;
  else
    initial_human2robot_tf.request.navigate = true;
  initial_human2robot_tf.request.first_person_perspective = true;
  ros::service::waitForService("temoto/change_human2robot_tf");
  temoto_teleop.tf_change_client_.call( initial_human2robot_tf );

  // Setup ROS publishers/subscribers
  ros::Subscriber sub_scaling_factor = n.subscribe<griffin_powermate::PowermateEvent>("/griffin_powermate/events", 1, &Teleoperator::processPowermate, &temoto_teleop);

  ros::Subscriber sub_pose_cmd;
  if (temoto_teleop.leap_input_)
  {
    sub_pose_cmd = n.subscribe(temoto_teleop.temoto_pose_cmd_topic_, 1,  &Teleoperator::processLeapCmd, &temoto_teleop);
  }
  else  // incremental pose cmds
    sub_pose_cmd = n.subscribe(temoto_teleop.temoto_pose_cmd_topic_, 1,  &Teleoperator::processJoyCmd, &temoto_teleop);

  ros::Subscriber sub_voice_commands = n.subscribe("temoto/voice_commands", 1, &Teleoperator::processVoiceCommand, &temoto_teleop);

  ros::Subscriber sub_end_effector = n.subscribe("temoto/end_effector_pose", 0, &Teleoperator::updateEndEffectorPose, &temoto_teleop);

  ros::Subscriber sub_executing_preplanned = n.subscribe("temoto/preplanned_sequence/result", 1, &Teleoperator::updatePreplannedFlag, &temoto_teleop);

  ros::Publisher pub_status = n.advertise<temoto::Status>("temoto/status", 3);
  
  while ( ros::ok() )
  {
    // Jog?
    if ( temoto_teleop.in_jog_mode_ && 
	!temoto_teleop.executing_preplanned_sequence_ )  	// Can't while doing something else
    {
      temoto_teleop.callRobotMotionInterface(low_level_cmds::GO);
    }

    // publish current status
    pub_status.publish( temoto_teleop.setStatus() );
    
    ros::spinOnce();
    node_rate.sleep();
  } // end while
  
  return 0;
} // end main
