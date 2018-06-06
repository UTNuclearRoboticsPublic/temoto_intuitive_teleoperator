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
Teleoperator::Teleoperator() :
 preplanned_sequence_client_("temoto/preplanned_sequence", true),  // true--> don't block the thread
 navigateIF_("move_base"),
 tf_listener_(tf_buffer_)
{
  temoto_spacenav_pose_cmd_topic_ = get_ros_params::getStringParam("/temoto/temoto_spacenav_pose_cmd_topic", n_);
  temoto_xbox_pose_cmd_topic_ = get_ros_params::getStringParam("/temoto/temoto_xbox_pose_cmd_topic", n_);

  // By default navigation is turned OFF
  bool navigate = get_ros_params::getBoolParam("temoto/navigate", n_);
  // By default manipulation is turned ON
  manipulate_ = get_ros_params::getBoolParam("temoto/manipulate", n_);

  pub_abort_ = n_.advertise<std_msgs::String>("temoto/abort", 1, true);
  pub_jog_arm_cmds_ = n_.advertise<geometry_msgs::TwistStamped>(get_ros_params::getStringParam("/temoto/incoming_jog_topic", n_), 1);
  pub_jog_base_cmds_ = n_.advertise<geometry_msgs::Twist>("/temoto/base_cmd_vel", 1);
  


  // Get movegroup and frame names of all arms the user might want to control
  // First, how many ee's are there?
  int num_ee = 1;
  if ( !n_.getParam("/temoto_frames/num_ee", num_ee) )
    ROS_ERROR("[start_teleop/Teleoperator] num_ee was not specified in yaml. Aborting.");

  // Get the names associated with multiple end effectors. Shove in vectors.
  for (int i=0; i<num_ee; i++)
  {
    std::string ee_name = get_ros_params::getStringParam("/temoto_frames/ee/ee"+std::to_string(i)+"/end_effector", n_);
    ee_names_.push_back(ee_name);

    // Objects for arm motion interface
    std::string move_group_name = get_ros_params::getStringParam("/temoto_frames/ee/ee"+std::to_string(i)+"/movegroup", n_);
    arm_if_ptrs_.push_back ( new MoveRobotInterface( move_group_name ) );
  }

  // Specify the current ee & move_group
  current_movegroup_ee_index_ = 0;
  jog_twist_cmd_.header.frame_id = ee_names_.at( current_movegroup_ee_index_ );

  // Subscribers
  sub_nav_spd_ = n_.subscribe("/nav_collision_warning/spd_fraction", 1, &Teleoperator::nav_collision_cb, this);
  sub_spacenav_pose_cmd_ = n_.subscribe(temoto_spacenav_pose_cmd_topic_, 1,  &Teleoperator::processSpaceNavCmd, this);
  sub_xbox_pose_cmd_ = n_.subscribe(temoto_xbox_pose_cmd_topic_, 1,  &Teleoperator::processXboxCmd, this);
  sub_voice_commands_ = n_.subscribe("temoto/voice_commands", 1, &Teleoperator::processVoiceCommand, this);
  sub_executing_preplanned_ = n_.subscribe("temoto/preplanned_sequence/result", 1, &Teleoperator::updatePreplannedFlag, this);
  sub_scaling_factor_ = n_.subscribe<griffin_powermate::PowermateEvent>("/griffin_powermate/events", 1, &Teleoperator::processPowermate, this);

  // Set initial scale on incoming commands
  setScale();

  // tf frame for incoming cmds
  incremental_position_cmd_.header.frame_id = "spacenav";
  incremental_orientation_cmd_.header.frame_id = "spacenav";


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

  // Initial pose
  // Make sure absolute_pose_cmd_ is in "base_link" so nav will work properly
  ros::Duration(1).sleep();
  absolute_pose_cmd_ = arm_if_ptrs_.at( current_movegroup_ee_index_ )->movegroup_.getCurrentPose();

  geometry_msgs::TransformStamped prev_frame_to_new = performTransform(absolute_pose_cmd_.header.frame_id, "base_link");
  tf2::doTransform(absolute_pose_cmd_, absolute_pose_cmd_, prev_frame_to_new);

  // Reset the graphic now that we're sure the tf is available.
  reset_ee_graphic_pose_ = true;
}

/** Function that actually makes the service call to appropriate robot motion interface.
 *  Currently, there are MoveRobotInterface and NavigateRobotInterface.
 *  @param action_type determines what is requested from MoveGroup, i.e. PLAN, EXECUTE PLAN, or GO (aka plan and execute). 
 */
void Teleoperator::callRobotMotionInterface(std::string action_type)
{
  // =================================================
  // === Calling NavigateRobotInterface ==============
  // =================================================
  if (navT_or_manipF_) // if in NAVIGATION mode
  {
    // If operator requested ABORT
    if (action_type == low_level_cmds::ABORT)
    {
      ROS_INFO("[start_teleop/callRobotMotionInterface] Requesting robot to stop navigation.");
      // Stop
      geometry_msgs::PoseStamped empty_pose;
      if ( !navigateIF_.navRequest( low_level_cmds::ABORT, empty_pose ) )
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
        jog_twist_cmd_.twist.angular.z = jog_twist_cmd_.twist.linear.y;

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
      tf::quaternionMsgToTF(absolute_pose_cmd_.pose.orientation, quat_current_cmd_frame);
      tf::Matrix3x3(quat_current_cmd_frame).getRPY(lm_roll, lm_pitch, lm_yaw);
    

      double bl_roll, bl_pitch, bl_yaw;
      // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
      tf::Quaternion quat_base_link;
      tf::quaternionMsgToTF(absolute_pose_cmd_.pose.orientation, quat_base_link);

      // the tf::Quaternion has a method to access roll, pitch, and yaw
      tf::Matrix3x3(quat_base_link).getRPY(bl_roll, bl_pitch, bl_yaw);
      
      quat_base_link.setRPY(0., 0., bl_yaw);
      quat_base_link.normalize();
      tf::quaternionTFToMsg(quat_base_link, absolute_pose_cmd_.pose.orientation);
      
      // Move the robot
      geometry_msgs::PoseStamped filler;
      ROS_WARN_STREAM("Requesting navigation.");
      if ( !navigateIF_.navRequest( action_type, absolute_pose_cmd_ ) )
      {
	      ROS_ERROR("[start_teleop/callRobotMotionInterface] Failed to call temoto/navigate_robot/navRequest()");
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
      // Send the motion request
      arm_if_ptrs_.at( current_movegroup_ee_index_ )-> req_action_type_ = action_type;
      arm_if_ptrs_.at( current_movegroup_ee_index_ )-> target_pose_stamped_ = absolute_pose_cmd_;
      arm_if_ptrs_.at( current_movegroup_ee_index_ )-> requestMove();

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
  reset_ee_graphic_pose_ = true;

  return;
} // end callRobotMotionInterface

/** Callback function for relative position commands.
 *  Add the new command to the current RViz marker pose.
 *  @param pose_cmd sensor_msgs::Joy a joystick command containing pose and button info
 */
void Teleoperator::processSpaceNavCmd(sensor_msgs::Joy pose_cmd)
{
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
  if (reset_ee_graphic_pose_)
    resetEEGraphicPose();

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
    incremental_orientation_cmd_.vector.z = rot_scale_*pose_cmd.axes[5];  // about z axis

    tf::Quaternion q_previous_cmd, q_incremental;
    q_incremental = tf::createQuaternionFromRPY(incremental_orientation_cmd_.vector.x, incremental_orientation_cmd_.vector.y, incremental_orientation_cmd_.vector.z);

    // Add the incremental command to the previously commanded orientation
    quaternionMsgToTF(absolute_pose_cmd_.pose.orientation , q_previous_cmd);  // Get the current orientation
    q_previous_cmd *= q_incremental;  // Calculate the new orientation
    q_previous_cmd.normalize();
    quaternionTFToMsg(q_previous_cmd, absolute_pose_cmd_.pose.orientation);  // Stuff it back into the pose cmd

    // POSITION
    // Ignore Z (out of plane)

    // Integrate the incremental cmd. It persists even if the robot moves
    incremental_position_cmd_.vector.x += pos_scale_*pose_cmd.axes[0];  // X is fwd/back in base_link  
    incremental_position_cmd_.vector.y += pos_scale_*pose_cmd.axes[1];  // Y is left/right

    // Incoming position cmds are in the spacenav frame
    // So convert them to base_link for nav
    // We need an intermediate variable here so that incremental_position_cmd_ doesn't get transformed
    geometry_msgs::Vector3Stamped incoming_position_cmd = incremental_position_cmd_;

    geometry_msgs::TransformStamped prev_frame_to_new = performTransform( incoming_position_cmd.header.frame_id, "base_link" );
    tf2::doTransform(incoming_position_cmd, incoming_position_cmd, prev_frame_to_new);

    // Ignore Z
    // Unlike manipulation mode, the center of the robot base is defined to be the origin (0,0,0). So we don't need to add anything else here.
    absolute_pose_cmd_.pose.position.x = incoming_position_cmd.vector.x;
    absolute_pose_cmd_.pose.position.y = incoming_position_cmd.vector.y;
    absolute_pose_cmd_.pose.position.z = 0;
  }


  //////////////////////////////
  // POINT-TO-POINT MANIPULATION
  //////////////////////////////
  if ( !navT_or_manipF_ && !in_jog_mode_ )
  {
    // Integrate for point-to-point motion
    // Member variables (Vector3Stamped) that hold incoming cmds have already been stamped in the spacenav frame

    // ORIENTATION
    // new incremental rpy command
    incremental_orientation_cmd_.vector.x = rot_scale_*pose_cmd.axes[3];  // about x axis
    incremental_orientation_cmd_.vector.y = rot_scale_*pose_cmd.axes[4];  // about y axis
    incremental_orientation_cmd_.vector.z = rot_scale_*pose_cmd.axes[5];  // about z axis

    tf::Quaternion q_previous_cmd, q_incremental;
    q_incremental = tf::createQuaternionFromRPY(incremental_orientation_cmd_.vector.x, incremental_orientation_cmd_.vector.y, incremental_orientation_cmd_.vector.z);

    // Add the incremental command to the previously commanded orientation
    quaternionMsgToTF(absolute_pose_cmd_.pose.orientation , q_previous_cmd);  // Get the current orientation
    q_previous_cmd *= q_incremental;  // Calculate the new orientation
    q_previous_cmd.normalize();
    quaternionTFToMsg(q_previous_cmd, absolute_pose_cmd_.pose.orientation);  // Stuff it back into the pose cmd

    // POSITION
    // Again, in "spacenav" frame
    incremental_position_cmd_.vector.x = pos_scale_*pose_cmd.axes[0];  // X is fwd/back in spacenav
    incremental_position_cmd_.vector.y = pos_scale_*pose_cmd.axes[1];   // Y is left/right
    incremental_position_cmd_.vector.z = pos_scale_*pose_cmd.axes[2];  // Z is up/down

    // Incoming position cmds are in the spacenav frame
    // So convert them the frame of absolute_pose_cmd_
    // We need an intermediate variable here so that incremental_position_cmd_ doesn't get overwritten.
    geometry_msgs::Vector3Stamped incoming_position_cmd = incremental_position_cmd_;

    geometry_msgs::TransformStamped prev_frame_to_new = performTransform( incoming_position_cmd.header.frame_id, absolute_pose_cmd_.header.frame_id );
    tf2::doTransform(incoming_position_cmd, incoming_position_cmd, prev_frame_to_new);

    absolute_pose_cmd_.pose.position.x += incoming_position_cmd.vector.x;
    absolute_pose_cmd_.pose.position.y += incoming_position_cmd.vector.y;
    absolute_pose_cmd_.pose.position.z += incoming_position_cmd.vector.z;  
  }

  return;
} // end processSpaceNavCmd

void Teleoperator::processXboxCmd(sensor_msgs::Joy pose_cmd)
{
  //Mapping xbox controls
  float x_pos = pose_cmd.axes[1]; float y_pos = pose_cmd.axes[0]; //Left Stick
  float z_pos = pose_cmd.buttons[5] - pose_cmd.buttons[4]; //Back Buttons (Left up, Right down)
  float x_ori = -1 * pose_cmd.axes[3]; float y_ori = pose_cmd.axes[4]; //Right Stick
  //z_ori ---> Back Triggers
  //Back Triggers return analog input btw 1 and -1
  float r_trig = pose_cmd.axes[5]; float l_trig = pose_cmd.axes[2];
  r_trig = (r_trig - abs(r_trig))/(-2*r_trig + 0.0000001);
  l_trig = (l_trig - abs(l_trig))/(-2*l_trig + 0.0000001);
  float z_ori = r_trig - l_trig;
  //Buttons
  int a_button = pose_cmd.buttons[0];
  int b_button = pose_cmd.buttons[1];


  // If rotational components >> translational, ignore translation (and vice versa)
  double trans_mag = pow( x_pos*x_pos + y_pos*y_pos + z_pos*z_pos, 2 );
  double rot_mag = pow( x_ori*x_ori + y_ori*y_ori + z_ori*z_ori, 2);
  if ( trans_mag > 2.*rot_mag )
  {
    x_ori = 0.;
    y_ori = 0.;
    z_ori = 0.;
  }
  else if ( rot_mag > 2.*trans_mag )
  {
    x_pos = 0.;
    y_pos = 0.;
    z_pos = 0.;
  }

  // Should we reset the command integrations?
  // Can be used to reset the hand marker, or when switching betw. manip & nav modes
  if (reset_ee_graphic_pose_)
    resetEEGraphicPose();

  ///////////////////////////////////////////////////////////
  // JOGGING
  // Can use the same type of command for arm and base teleop
  ///////////////////////////////////////////////////////////
  if (in_jog_mode_)
  {
    jog_twist_cmd_.header.stamp = ros::Time::now();
    jog_twist_cmd_.twist.linear.x = pos_scale_*x_pos;
    jog_twist_cmd_.twist.linear.y = pos_scale_*y_pos;
    jog_twist_cmd_.twist.linear.z = pos_scale_*z_pos;
    jog_twist_cmd_.twist.angular.x = rot_scale_*x_ori;
    jog_twist_cmd_.twist.angular.y = rot_scale_*y_ori;
    jog_twist_cmd_.twist.angular.z = rot_scale_*z_ori;
  }

  ////////////////////////////
  // POINT-TO-POINT NAVIGATION
  ////////////////////////////
  if ( navT_or_manipF_ && !in_jog_mode_ )
  {
    // ORIENTATION
    // new incremental yaw command
    // For nav mode, we want to stay in the plane ==> ignore roll & pitch
    incremental_orientation_cmd_.vector.z = rot_scale_*z_ori;  // about z axis

    tf::Quaternion q_previous_cmd, q_incremental;
    q_incremental = tf::createQuaternionFromRPY(incremental_orientation_cmd_.vector.x, incremental_orientation_cmd_.vector.y, incremental_orientation_cmd_.vector.z);

    // Add the incremental command to the previously commanded orientation
    quaternionMsgToTF(absolute_pose_cmd_.pose.orientation , q_previous_cmd);  // Get the current orientation
    q_previous_cmd *= q_incremental;  // Calculate the new orientation
    q_previous_cmd.normalize();
    quaternionTFToMsg(q_previous_cmd, absolute_pose_cmd_.pose.orientation);  // Stuff it back into the pose cmd

    // POSITION
    // Ignore Z (out of plane)

    // Integrate the incremental cmd. It persists even if the robot moves
    incremental_position_cmd_.vector.x += pos_scale_*x_pos;  // X is fwd/back in base_link
    incremental_position_cmd_.vector.y += pos_scale_*y_pos;   // Y is left/right

    // Incoming position cmds are in the spacenav frame
    // So convert them to base_link for nav
    // We need an intermediate variable here so that incremental_position_cmd_ doesn't get transformed
    geometry_msgs::Vector3Stamped incoming_position_cmd = incremental_position_cmd_;

    geometry_msgs::TransformStamped prev_frame_to_new = performTransform( incoming_position_cmd.header.frame_id, "base_link" );
    tf2::doTransform(incoming_position_cmd, incoming_position_cmd, prev_frame_to_new);

    // Ignore Z
    // Unlike manipulation mode, the center of the robot base is defined to be the origin (0,0,0). So we don't need to add anything else here.
    absolute_pose_cmd_.pose.position.x = incoming_position_cmd.vector.x;
    absolute_pose_cmd_.pose.position.y = incoming_position_cmd.vector.y;
    absolute_pose_cmd_.pose.position.z = 0;
  }


  //////////////////////////////
  // POINT-TO-POINT MANIPULATION
  //////////////////////////////
  if ( !navT_or_manipF_ && !in_jog_mode_ )
  {
    // Integrate for point-to-point motion
    // Member variables (Vector3Stamped) that hold incoming cmds have already been stamped in the spacenav frame

    // ORIENTATION
    // new incremental rpy command
    incremental_orientation_cmd_.vector.x = rot_scale_*pose_cmd.axes[3];  // about x axis
    incremental_orientation_cmd_.vector.y = rot_scale_*pose_cmd.axes[4];  // about y axis
    incremental_orientation_cmd_.vector.z = rot_scale_*pose_cmd.axes[5];  // about z axis

    tf::Quaternion q_previous_cmd, q_incremental;
    q_incremental = tf::createQuaternionFromRPY(incremental_orientation_cmd_.vector.x, incremental_orientation_cmd_.vector.y, incremental_orientation_cmd_.vector.z);

    // Add the incremental command to the previously commanded orientation
    quaternionMsgToTF(absolute_pose_cmd_.pose.orientation , q_previous_cmd);  // Get the current orientation
    q_previous_cmd *= q_incremental;  // Calculate the new orientation
    q_previous_cmd.normalize();
    quaternionTFToMsg(q_previous_cmd, absolute_pose_cmd_.pose.orientation);  // Stuff it back into the pose cmd

    // POSITION
    // Again, in "spacenav" frame
    incremental_position_cmd_.vector.x = pos_scale_*pose_cmd.axes[0];  // X is fwd/back in spacenav
    incremental_position_cmd_.vector.y = pos_scale_*pose_cmd.axes[1];   // Y is left/right
    incremental_position_cmd_.vector.z = pos_scale_*pose_cmd.axes[2];  // Z is up/down

    // Incoming position cmds are in the spacenav frame
    // So convert them the frame of absolute_pose_cmd_
    // We need an intermediate variable here so that incremental_position_cmd_ doesn't get overwritten.
    geometry_msgs::Vector3Stamped incoming_position_cmd = incremental_position_cmd_;

    geometry_msgs::TransformStamped prev_frame_to_new = performTransform( incoming_position_cmd.header.frame_id, absolute_pose_cmd_.header.frame_id );
    tf2::doTransform(incoming_position_cmd, incoming_position_cmd, prev_frame_to_new);

    absolute_pose_cmd_.pose.position.x += incoming_position_cmd.vector.x;
    absolute_pose_cmd_.pose.position.y += incoming_position_cmd.vector.y;
    absolute_pose_cmd_.pose.position.z += incoming_position_cmd.vector.z;
  }
} // end processXboxCmd


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


/** Callback function for /temoto/voice_commands.
 *  Executes published voice command.
 *  @param voice_command contains the specific command as an unsigned integer.
 */
void Teleoperator::processVoiceCommand(std_msgs::String voice_command)
{
  //////////////////////////////////////////////////
  //  Stop jogging (jogging preempts other commands)
  //////////////////////////////////////////////////

  if (voice_command.data == "stop jogging")
  {
    ROS_INFO("Switching out of jog mode");
    in_jog_mode_ = false;
    reset_ee_graphic_pose_ = true;
    setScale();

    return;
  }

  /////////////////////////
  //  Handle ABORT commands
  /////////////////////////
  if (voice_command.data == "stop stop")
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


  if ( voice_command.data == "manipulation" )  // Switch over to manipulation (MoveIt!) mode
  {
    // If not already in manipulation mode
    if ( navT_or_manipF_==true )
    {
      // Make sure we aren't in jog mode
      ROS_INFO("Switching out of jog mode");
      in_jog_mode_ = false;
      setScale();

      reset_ee_graphic_pose_ = true;  // Flag that the integrated cmds need to be reset
      ROS_INFO("Going into MANIPULATION mode  ...");
      AMP_HAND_MOTION_ = 1;
      navT_or_manipF_ = false;
      setScale();
    }
    else
      ROS_INFO("Already in manipulation mode.");
    return;
  }
  else if ( voice_command.data == "navigation" )  // Switch over to navigation mode
  {
    // If not already in nav mode
    if ( navT_or_manipF_==false )
    {
      // Make sure we aren't in jog mode
      ROS_INFO("Switching out of jog mode");
      in_jog_mode_ = false;
      setScale();

      reset_ee_graphic_pose_ = true;  // Flag that the integrated cmds need to be reset
      ROS_INFO("Going into NAVIGATION mode  ...");
      AMP_HAND_MOTION_ = 100;

      navT_or_manipF_ = true;
      setScale();
    }
    else
      ROS_INFO("Already in navigation mode.");
    return;
  }


  if(voice_command.data == "close gripper")  // Close the gripper - a preplanned sequence
  {
    ROS_INFO("Closing the gripper ...");
    Teleoperator::triggerSequence(voice_command.data);
    return;
  }
  else if(voice_command.data == "open gripper")  // Open the gripper - a preplanned sequence
  {
    ROS_INFO("Opening the gripper ...");
    Teleoperator::triggerSequence(voice_command.data);
    return;
  }


  if ( in_jog_mode_ )  // Avoid planning, executing, etc. while in jog mode
    return;


  else  // There's nothing blocking any arbitrary command
  {
    if (voice_command.data == "jog mode")
    {
      // For now, can only jog with EE0 in manipulation mode.
      // In nav mode or current EE==0  ==> OK to jog.
      if ( navT_or_manipF_ || current_movegroup_ee_index_==0 )
      {
        ROS_INFO("Switching to jog mode");
        in_jog_mode_ = true;
        reset_ee_graphic_pose_ = true;
        setScale();
      }
      else
        ROS_WARN_STREAM("[temoto_teleop::start_teleop] Can only jog with EE0");
      return;
    }
    else if (voice_command.data == "robot please plan")
    {
      ROS_INFO("Planning ...");
      callRobotMotionInterface(low_level_cmds::PLAN);
      return;
    }
    else if (voice_command.data == "robot please execute")
    {
      ROS_INFO("Executing last plan ...");
      callRobotMotionInterface(low_level_cmds::EXECUTE);
      reset_ee_graphic_pose_ = true;

      return;
    }
    else if (voice_command.data == "robot plan and go")
    {
      ROS_INFO("Planning and moving ...");
      reset_ee_graphic_pose_ = true;
      callRobotMotionInterface(low_level_cmds::GO);
      return;
    }
    else if (voice_command.data == "next end effector")
    {
      ROS_INFO("Controlling the next EE from yaml file ...");
      reset_ee_graphic_pose_ = true;  // Flag that the integrated cmds need to be reset
      switchEE();
      return;
    }
  
    else
    {
      ROS_INFO("Unknown voice command.");
    }
  }  // End of handling non-Abort commands

  return;
}

void Teleoperator::triggerSequence(std::string& voice_command)
{
  temoto::PreplannedSequenceGoal goal;
  goal.sequence_name = voice_command;
  preplanned_sequence_client_.sendGoal(goal);

  // Flag that a preplanned sequence is running, so pause most other commands (except Abort)
  executing_preplanned_sequence_ = true;
}

/** Puts the latest private variable values into temoto/status message.
 *  This is used in marker publication.
 *  @return void.
 */
void Teleoperator::setGraphicsFramesStatus(bool adjust_camera)
{
  graphics_and_frames_.latest_status_.in_navigation_mode = navT_or_manipF_;
  graphics_and_frames_.latest_status_.scale_by = pos_scale_;
  graphics_and_frames_.latest_status_.commanded_pose = absolute_pose_cmd_;
  graphics_and_frames_.latest_status_.end_effector_pose = arm_if_ptrs_.at( current_movegroup_ee_index_ )->movegroup_.getCurrentPose();
  graphics_and_frames_.latest_status_.current_movegroup_ee_index = current_movegroup_ee_index_;

  graphics_and_frames_.adjust_camera_ = adjust_camera;

  graphics_and_frames_.crunch();

  return;
} // end setGraphicsFramesStatus()

void Teleoperator::setScale()
{
  if (navT_or_manipF_) // navigation
  {
    if (in_jog_mode_)  // nav, jog mode
    {
      pos_scale_ = get_ros_params::getDoubleParam("temoto/motion_scales/jog/nav_pos_scale", n_);
      rot_scale_ = get_ros_params::getDoubleParam("temoto/motion_scales/jog/nav_rot_scale", n_);
    }
    else  // nav, pt-to-pt mode
    {
      pos_scale_ = get_ros_params::getDoubleParam("temoto/motion_scales/pt_to_pt/nav_pos_scale", n_);
      rot_scale_ = get_ros_params::getDoubleParam("temoto/motion_scales/pt_to_pt/nav_rot_scale", n_);
    }
  }
  else // manipulation
  {
    if (in_jog_mode_)  // manipulate, jog mode
    {
      pos_scale_ = get_ros_params::getDoubleParam("temoto/motion_scales/jog/manip_pos_scale", n_);
      rot_scale_ = get_ros_params::getDoubleParam("temoto/motion_scales/jog/manip_rot_scale", n_);
    }
    else  // manipulate, pt-to-pt mode
    {
      pos_scale_ = get_ros_params::getDoubleParam("temoto/motion_scales/pt_to_pt/manip_pos_scale", n_);
      rot_scale_ = get_ros_params::getDoubleParam("temoto/motion_scales/pt_to_pt/manip_rot_scale", n_);
    }
  }
}

// Switch to the next available EE
void Teleoperator::switchEE()
{
  if (current_movegroup_ee_index_ < ee_names_.size()-1 )
    current_movegroup_ee_index_++;
  else
    current_movegroup_ee_index_ = 0;

  jog_twist_cmd_.header.frame_id = ee_names_.at(current_movegroup_ee_index_);

  // Set camera at new EE
  bool adjust_camera = true;
  setGraphicsFramesStatus( adjust_camera );

  // Reset the hand marker to be at the EE
  reset_ee_graphic_pose_ = true;
}

// Return a transform between 2 frames
geometry_msgs::TransformStamped Teleoperator::performTransform(std::string source_frame, std::string target_frame)
{
  geometry_msgs::TransformStamped prev_frame_to_new;

  // If the strings were valid
  if ( source_frame.length()>0 && target_frame.length()>0 )
  {
    // Check for and remove leading '/'
    if ( source_frame.at(0)=='/' )
      source_frame.erase(0,1);
    if ( target_frame.at(0)=='/' )
      target_frame.erase(0,1);

    try{
      prev_frame_to_new = tf_buffer_.lookupTransform( source_frame, target_frame, ros::Time(0), ros::Duration(2.0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  return prev_frame_to_new;
}

// Reset the end-effector graphic to current robot pose
void Teleoperator::resetEEGraphicPose()
{
  if ( navT_or_manipF_ )  // Navigation --> Center on base_link
  {
    absolute_pose_cmd_.pose.position.x = 0.;
    absolute_pose_cmd_.pose.position.y = 0.;
    absolute_pose_cmd_.pose.position.z = 0.;

    absolute_pose_cmd_.pose.orientation.x = 0.;
    absolute_pose_cmd_.pose.orientation.y = 0.;
    absolute_pose_cmd_.pose.orientation.z = 0.;
    absolute_pose_cmd_.pose.orientation.w = 1.;

    incremental_position_cmd_.vector.x = 0.;
    incremental_position_cmd_.vector.y = 0.;
    incremental_position_cmd_.vector.z = 0.;

    incremental_orientation_cmd_.vector.x = 0.;
    incremental_orientation_cmd_.vector.y = 0.;
    incremental_orientation_cmd_.vector.z = 0.;
  }
  else  // Manipulation --> Center on EE
    absolute_pose_cmd_ = arm_if_ptrs_.at( current_movegroup_ee_index_ )->movegroup_.getCurrentPose();

  // Reset the flag
  reset_ee_graphic_pose_ = false;
}


// MAIN
int main(int argc, char **argv)
{
  ros::init(argc, argv, "start_teleop");

  ros::NodeHandle n;

  ros::Rate node_rate(90.);

  // Using an async spinner. It is needed for moveit's MoveGroup::plan()
  ros::AsyncSpinner spinner(4);
  spinner.start();

  Teleoperator temoto_teleop;

  while ( ros::ok() )
  {
    // Jog?
    // Can't jog while doing something else
    if ( temoto_teleop.in_jog_mode_ && 
	!temoto_teleop.executing_preplanned_sequence_ )
      temoto_teleop.callRobotMotionInterface(low_level_cmds::GO);

    temoto_teleop.setGraphicsFramesStatus(false); // Update poses, scale, and nav-or-manip mode for the frames calculation
    temoto_teleop.graphics_and_frames_.crunch();

    node_rate.sleep();
  } // end main loop

  
  return 0;
} // end main