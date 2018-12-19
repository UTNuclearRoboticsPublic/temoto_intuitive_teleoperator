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

/** @file start_teleop.cpp
 *
 *  @brief Central node for TEMOTO teleoperator. Subscribes to relevant
 * messages, calls move and navigation interfaces, and publishes system status.
 *
 *  @author karl.kruusamae(at)utexas.edu, andyz(at)utexas.edu
 */

#include "temoto/start_teleop.h"

/** Constructor for Teleoperator.
 */
Teleoperator::Teleoperator() : nav_interface_("move_base"), tf_listener_(tf_buffer_)
{
  temoto_spacenav_pose_cmd_topic_ = get_ros_params::getStringParam("/temoto/temoto_spacenav_pose_cmd_topic", n_);
  temoto_xbox_pose_cmd_topic_ = get_ros_params::getStringParam("/temoto/temoto_xbox_pose_cmd_topic", n_);

  // By default navigation is turned OFF
  enable_navigation_ = get_ros_params::getBoolParam("temoto/enable_navigation", n_);
  // By default manipulation is turned ON
  enable_manipulation_ = get_ros_params::getBoolParam("temoto/enable_manipulation", n_);

  base_frame_ = get_ros_params::getStringParam("temoto/base_frame", n_);

  pub_abort_ = n_.advertise<std_msgs::String>("temoto/abort", 1, true);
  pub_jog_base_cmds_ = n_.advertise<geometry_msgs::Twist>("/temoto/base_cmd_vel", 1);

  // Get movegroup and frame names of all arms the user might want to control
  // First, how many ee's are there?
  int num_ee = 1;
  if (!n_.getParam("/temoto/num_ee", num_ee))
    ROS_ERROR("[start_teleop/Teleoperator] num_ee was not specified in yaml. "
              "Aborting.");

  // Get the parameters associated with (possibly) multiple end effectors. Shove in vectors.
  for (int i = 0; i < num_ee; ++i)
  {
    end_effector_parameters_.ee_names.push_back(
        get_ros_params::getStringParam("/temoto/ee/ee" + std::to_string(i) + "/end_effector", n_));

    // Objects for arm motion interface
    std::string move_group_name =
        get_ros_params::getStringParam("/temoto/ee/ee" + std::to_string(i) + "/movegroup", n_);
    end_effector_parameters_.arm_interface_ptrs.push_back(new MoveRobotInterface(move_group_name));

    // Cartesian jogging commands
    std::string jog_topic = get_ros_params::getStringParam("/temoto/ee/ee" + std::to_string(i) + "/jog_topic", n_);
    ros::Publisher jog_pub = n_.advertise<geometry_msgs::TwistStamped>(jog_topic, 1);
    end_effector_parameters_.jog_publishers.push_back(std::make_shared<ros::Publisher>(jog_pub));

    // Joint jogging commands
    std::string joint_jog_topic =
        get_ros_params::getStringParam("/temoto/ee/ee" + std::to_string(i) + "/joint_jog_topic", n_);
    ros::Publisher joint_jog_pub = n_.advertise<jog_msgs::JogJoint>(joint_jog_topic, 1);
    end_effector_parameters_.joint_jog_publishers.push_back(std::make_shared<ros::Publisher>(joint_jog_pub));

    // Names of wrist joints, for jogging
    end_effector_parameters_.wrist_joint_names.push_back(
        get_ros_params::getStringParam("/temoto/ee/ee" + std::to_string(i) + "/wrist_joint_name", n_));

    // Enable compliance for this end-effector?
    end_effector_parameters_.allow_compliant_jog.push_back(
        get_ros_params::getBoolParam("/temoto/ee/ee" + std::to_string(i) + "/allow_compliant_jog", n_));

    if (end_effector_parameters_.allow_compliant_jog.at(i))
      compliance_object_.addCompliantEndEffector(i);

    // Names of gripper topics
    end_effector_parameters_.gripper_topics.push_back(
        get_ros_params::getStringParam("/temoto/ee/ee" + std::to_string(i) + "/gripper_topic", n_));
  }

  // An object to handle gripper commands
  gripper_interface_ =
      std::unique_ptr<grippers::Grippers>(new grippers::Grippers(end_effector_parameters_.gripper_topics));

  // Specify the current ee & move_group
  current_movegroup_ee_index_ = 0;

  // Set up jogging msgs for the base
  nav_twist_cmd_.header.frame_id = base_frame_;

  // Set up jogging msgs for the current EE
  jog_twist_cmd_.header.frame_id = end_effector_parameters_.ee_names.at(current_movegroup_ee_index_);
  joint_jog_cmd_.joint_names.push_back(end_effector_parameters_.wrist_joint_names.at(current_movegroup_ee_index_));
  joint_jog_cmd_.deltas.push_back(0);

  // Subscribers
  sub_nav_spd_ = n_.subscribe("/nav_collision_warning/spd_fraction", 1, &Teleoperator::navCollisionCallback, this);
  sub_spacenav_pose_cmd_ = n_.subscribe(temoto_spacenav_pose_cmd_topic_, 1, &Teleoperator::spaceNavCallback, this);
  sub_xbox_pose_cmd_ = n_.subscribe(temoto_xbox_pose_cmd_topic_, 1, &Teleoperator::xboxCallback, this);
  sub_voice_commands_ = n_.subscribe("temoto/voice_commands", 1, &Teleoperator::processStringCommand, this);
  sub_scaling_factor_ = n_.subscribe<griffin_powermate::PowermateEvent>("/griffin_powermate/events", 1,
                                                                        &Teleoperator::processPowermate, this);
  sub_temoto_sleep_ = n_.subscribe<std_msgs::Bool>("temoto/temoto_sleep", 1, &Teleoperator::sleepCallback, this);

  // Set initial scale on incoming commands
  setScale();

  // tf frame for incoming cmds
  incremental_position_cmd_.header.frame_id = "temoto_command_frame";
  incremental_orientation_cmd_.header.frame_id = "temoto_command_frame";

  // Setting up control_state, i.e., whether teleoperator is controlling
  // navigation, manipulation, or both.
  if (enable_manipulation_ && enable_navigation_)
    current_nav_or_manip_mode_ = MANIPULATION;  // if navigation AND manipulation are enabled,
                                                // start out in manipulation mode.
  else if (enable_navigation_ && !enable_manipulation_)
    current_nav_or_manip_mode_ = NAVIGATION;
  else if (enable_manipulation_ && !enable_navigation_)
    current_nav_or_manip_mode_ = MANIPULATION;

  // Initial pose
  // Make sure absolute_pose_cmd_ is in base_frame_ so nav will work properly
  ros::Duration(1).sleep();
  absolute_pose_cmd_ =
      end_effector_parameters_.arm_interface_ptrs.at(current_movegroup_ee_index_)->movegroup_.getCurrentPose();

  // Wait for this pose to be available
  geometry_msgs::TransformStamped prev_frame_to_new;
  while (!calculateTransform(absolute_pose_cmd_.header.frame_id, base_frame_, prev_frame_to_new))
    ROS_WARN_STREAM("Waiting for initial transform from command frame to base_frame_");
  tf2::doTransform(absolute_pose_cmd_, absolute_pose_cmd_, prev_frame_to_new);

  // Reset the graphic now that we're sure the tf is available.
  graphics_and_frames_.latest_status_.moveit_planning_frame =
      end_effector_parameters_.arm_interface_ptrs.at(current_movegroup_ee_index_)->movegroup_.getPlanningFrame();
  resetEEGraphicPose();
  setGraphicsFramesStatus(true);
}

/** Function that actually makes the service call to appropriate robot motion
 * interface.
 *  Currently, there are MoveRobotInterface and NavigateRobotInterface.
 *  @param action_type determines what is requested from MoveGroup, i.e. PLAN,
 * EXECUTE PLAN, or GO (aka plan and execute).
 */
void Teleoperator::callRobotMotionInterface(std::string action_type)
{
  // =================================================
  // === Calling NavigateRobotInterface ==============
  // =================================================
  if (current_nav_or_manip_mode_ == NAVIGATION)
  {
    // If operator requested ABORT
    if (action_type == common_commands::ABORT)
    {
      ROS_INFO("[start_teleop/callRobotMotionInterface] Requesting robot to "
               "stop navigation.");
      // Stop
      geometry_msgs::PoseStamped empty_pose;
      if (!nav_interface_.navRequest(common_commands::ABORT, empty_pose))
      {
        ROS_ERROR("[start_teleop/callRobotMotionInterface] Failed to call "
                  "temoto/navigate_robot_srv");
      }
      return;
    }  // end: if (action_type == "abort")
    // If operator requested the robot to move to a goal pose
    else if (action_type == common_commands::GO)
    {
      // Jogging
      if (in_jog_mode_)
      {
        nav_twist_cmd_.twist.angular.z = nav_twist_cmd_.twist.linear.y;

        // Scale velocity if close to obstacle
        nav_twist_cmd_.twist.linear.x *= nav_speed_fraction_;
        nav_twist_cmd_.twist.linear.y *= nav_speed_fraction_;
        nav_twist_cmd_.twist.linear.z *= nav_speed_fraction_;

        nav_twist_cmd_.twist.angular.x *= nav_speed_fraction_;
        nav_twist_cmd_.twist.angular.y *= nav_speed_fraction_;
        nav_twist_cmd_.twist.angular.z *= nav_speed_fraction_;

        pub_jog_base_cmds_.publish(nav_twist_cmd_.twist);
        return;
      }

      double bl_roll, bl_pitch, bl_yaw;
      // the incoming geometry_msgs::Quaternion is transformed to a
      // tf::Quaterion
      tf::Quaternion quat_base_link;
      tf::quaternionMsgToTF(absolute_pose_cmd_.pose.orientation, quat_base_link);
      tf::Matrix3x3(quat_base_link).getRPY(bl_roll, bl_pitch, bl_yaw);

      quat_base_link.setRPY(0., 0., bl_yaw);
      quat_base_link.normalize();
      tf::quaternionTFToMsg(quat_base_link, absolute_pose_cmd_.pose.orientation);

      // Move the robot
      ROS_WARN_STREAM("Requesting navigation.");
      if (!nav_interface_.navRequest(action_type, absolute_pose_cmd_))
      {
        ROS_ERROR("[start_teleop/callRobotMotionInterface] Failed to call "
                  "temoto/navigate_robot/navRequest()");
      }
      return;
    }  // else if (action_type == "go")
  }    // if (current_nav_or_manip_mode_==NAVIGATION)
  // =================================================
  // === Calling MoveRobotInterface ==================
  // =================================================
  else if (current_nav_or_manip_mode_ == MANIPULATION)
  {
    // Jogging
    if (in_jog_mode_)
    {
      // If compliance is allowed (yaml file) and user gave a command to enable it
      if (enable_compliance_)
        compliance_object_.cartesianCompliantAdjustment(jog_twist_cmd_, current_movegroup_ee_index_);

      end_effector_parameters_.jog_publishers.at(current_movegroup_ee_index_)->publish(jog_twist_cmd_);

      end_effector_parameters_.joint_jog_publishers.at(current_movegroup_ee_index_)->publish(joint_jog_cmd_);

      return;
    }
    // Point-to-point motion
    else
    {
      // Send the motion request
      end_effector_parameters_.arm_interface_ptrs.at(current_movegroup_ee_index_)->req_action_type_ = action_type;
      end_effector_parameters_.arm_interface_ptrs.at(current_movegroup_ee_index_)->target_pose_stamped_ =
          absolute_pose_cmd_;
      end_effector_parameters_.arm_interface_ptrs.at(current_movegroup_ee_index_)->requestMove();

      return;
    }
  }
  // =================================================
  // === Unexpected case == NEITHER NAV NOR MANIP ====
  // =================================================
  else
  {
    ROS_INFO("[start_teleop/callRobotMotionInterface] Request unavailable for "
             "current list of robot interfaces.");
  }

  // Reset the hand marker to be at the EE
  resetEEGraphicPose();

  return;
}  // end callRobotMotionInterface

/** Callback function for relative position commands.
 *  Add the new command to the current RViz marker pose.
 *  @param pose_cmd sensor_msgs::Joy a joystick command containing pose and
 * button info
 */
void Teleoperator::spaceNavCallback(sensor_msgs::Joy pose_cmd)
{
  // If user put Temoto in sleep mode, do nothing.
  if (!temoto_sleep_)
  {
    // Wrist joint jogging with these two buttons (SpaceNav Pro)
    joint_jog_cmd_.deltas[0] = (pose_cmd.buttons[2] - pose_cmd.buttons[8]);
    joint_jog_cmd_.header.stamp = ros::Time::now();

    // Search for buttons that are mapped to verbal commands.
    // These are defined in a std::map in start_teleop.h
    for (int i = 0; i < pose_cmd.buttons.size(); ++i)
    {
      if (pose_cmd.buttons[i])
      {
        std_msgs::String text_command;

        // If that command exists in the std::map
        if (spacenav_buttons_.find(i) != spacenav_buttons_.end())
        {
          // Find the string that maps to this button index
          text_command.data = spacenav_buttons_.find(i)->second;
          processStringCommand(text_command);
        }

        // Debounce
        ros::Duration(0.1).sleep();
        break;
      }
    }

    processIncrementalPoseCmd(pose_cmd.axes[0], pose_cmd.axes[1], pose_cmd.axes[2], pose_cmd.axes[3], pose_cmd.axes[4],
                              pose_cmd.axes[5]);
  }

  return;
}  // end spaceNavCallback

void Teleoperator::xboxCallback(sensor_msgs::Joy pose_cmd)
{
  // If user put Temoto in sleep mode, do nothing.
  if (!temoto_sleep_)
  {
    // Mapping xbox controls
    double x_pos = pose_cmd.axes[1];
    double y_pos = pose_cmd.axes[0];                           // Left Stick
    double z_pos = pose_cmd.buttons[5] - pose_cmd.buttons[4];  // Back Buttons (Left up, Right down)
    double x_ori = -pose_cmd.axes[3];
    double y_ori = pose_cmd.axes[4];  // Right Stick
    // z_ori ---> Back Triggers
    // Back Triggers return analog input btw 1 and -1
    double r_trig = pose_cmd.axes[5];
    double l_trig = pose_cmd.axes[2];
    r_trig = (r_trig - abs(r_trig)) / (-2 * r_trig + 0.0000001);
    l_trig = (l_trig - abs(l_trig)) / (-2 * l_trig + 0.0000001);
    double z_ori = r_trig - l_trig;
    // Buttons
    int a_button = pose_cmd.buttons[0];
    int b_button = pose_cmd.buttons[1];

    processIncrementalPoseCmd(x_pos, y_pos, z_pos, x_ori, y_ori, z_ori);
  }

  return;
}  // end xboxCallback

// Do most of the calculations for an incremental-type pose command (like
// SpaceNav or XBox)
void Teleoperator::processIncrementalPoseCmd(double x_pos, double y_pos, double z_pos, double x_ori, double y_ori,
                                             double z_ori)
{
  // If rotational components >> translational, ignore translation (and vice
  // versa)
  double trans_mag = pow(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos, 0.5);
  double rot_mag = pow(x_ori * x_ori + y_ori * y_ori + z_ori * z_ori, 0.5);
  if (trans_mag > 2. * rot_mag)
  {
    x_ori = 0.;
    y_ori = 0.;
    z_ori = 0.;
  }
  else if (rot_mag > 2. * trans_mag)
  {
    x_pos = 0.;
    y_pos = 0.;
    z_pos = 0.;
  }

  ///////////////////////////////////////////////////////////
  // JOGGING
  ///////////////////////////////////////////////////////////
  if (in_jog_mode_)
  {
    if (current_nav_or_manip_mode_ == NAVIGATION)
    {
      nav_twist_cmd_.header.stamp = ros::Time::now();
      nav_twist_cmd_.twist.linear.x = pos_scale_ * x_pos;
      nav_twist_cmd_.twist.linear.y = pos_scale_ * y_pos;
      nav_twist_cmd_.twist.linear.z = pos_scale_ * z_pos;
      nav_twist_cmd_.twist.angular.x = rot_scale_ * x_ori;
      nav_twist_cmd_.twist.angular.y = rot_scale_ * y_ori;
      nav_twist_cmd_.twist.angular.z = rot_scale_ * z_ori;
    }

    if (current_nav_or_manip_mode_ == MANIPULATION)
    {
      jog_twist_cmd_.header.stamp = ros::Time::now();
      jog_twist_cmd_.twist.linear.x = pos_scale_ * x_pos;
      jog_twist_cmd_.twist.linear.y = pos_scale_ * y_pos;
      jog_twist_cmd_.twist.linear.z = pos_scale_ * z_pos;
      jog_twist_cmd_.twist.angular.x = rot_scale_ * x_ori;
      jog_twist_cmd_.twist.angular.y = rot_scale_ * y_ori;
      jog_twist_cmd_.twist.angular.z = rot_scale_ * z_ori;
    }
  }

  ////////////////////////////
  // POINT-TO-POINT NAVIGATION
  ////////////////////////////
  if ((current_nav_or_manip_mode_ == NAVIGATION) && !in_jog_mode_)
  {
    // ORIENTATION
    // new incremental yaw command
    // For nav mode, we want to stay in the plane ==> ignore roll & pitch
    incremental_orientation_cmd_.vector.z = rot_scale_ * z_ori;  // about z axis

    tf::Quaternion q_previous_cmd, q_incremental;
    q_incremental =
        tf::createQuaternionFromRPY(incremental_orientation_cmd_.vector.x, incremental_orientation_cmd_.vector.y,
                                    incremental_orientation_cmd_.vector.z);

    // Add the incremental command to the previously commanded orientation
    quaternionMsgToTF(absolute_pose_cmd_.pose.orientation, q_previous_cmd);  // Get the current orientation
    q_previous_cmd *= q_incremental;                                         // Calculate the new orientation
    q_previous_cmd.normalize();
    quaternionTFToMsg(q_previous_cmd, absolute_pose_cmd_.pose.orientation);  // Stuff it back into the pose cmd

    // POSITION
    // Ignore Z (out of plane)

    // Integrate the incremental cmd. It persists even if the robot moves
    incremental_position_cmd_.vector.x += pos_scale_ * x_pos;  // X is fwd/back in base_link
    incremental_position_cmd_.vector.y += pos_scale_ * y_pos;  // Y is left/right

    // Incoming position cmds are in the temoto_command_frame
    // So convert them to base_link for nav
    // We need an intermediate variable here so that incremental_position_cmd_
    // doesn't get transformed
    geometry_msgs::Vector3Stamped incoming_position_cmd = incremental_position_cmd_;

    geometry_msgs::TransformStamped prev_frame_to_new;
    // Make sure the transform is available, otherwise skip updating the pose
    if (calculateTransform(incoming_position_cmd.header.frame_id, base_frame_, prev_frame_to_new))
    {
      tf2::doTransform(incoming_position_cmd, incoming_position_cmd, prev_frame_to_new);

      // Ignore Z
      // Unlike manipulation mode, the center of the robot base is defined to be
      // the origin (0,0,0). So we don't need to add anything else here.
      absolute_pose_cmd_.pose.position.x = incoming_position_cmd.vector.x;
      absolute_pose_cmd_.pose.position.y = incoming_position_cmd.vector.y;
      absolute_pose_cmd_.pose.position.z = 0;
    }
    else
      ROS_WARN_STREAM("The transform to base frame is not available. Skipping.");
  }

  //////////////////////////////
  // POINT-TO-POINT MANIPULATION
  //////////////////////////////
  if ((current_nav_or_manip_mode_ == MANIPULATION) && !in_jog_mode_)
  {
    // Integrate for point-to-point motion
    // Member variables (Vector3Stamped) that hold incoming cmds have already
    // been stamped in the temoto_command_frame

    // ORIENTATION
    // new incremental rpy command
    incremental_orientation_cmd_.vector.x = rot_scale_ * x_ori;  // about x axis
    incremental_orientation_cmd_.vector.y = rot_scale_ * y_ori;  // about y axis
    incremental_orientation_cmd_.vector.z = rot_scale_ * z_ori;  // about z axis

    tf::Quaternion q_previous_cmd, q_incremental;
    q_incremental =
        tf::createQuaternionFromRPY(incremental_orientation_cmd_.vector.x, incremental_orientation_cmd_.vector.y,
                                    incremental_orientation_cmd_.vector.z);

    // Add the incremental command to the previously commanded orientation
    quaternionMsgToTF(absolute_pose_cmd_.pose.orientation, q_previous_cmd);  // Get the current orientation
    q_previous_cmd *= q_incremental;                                         // Calculate the new orientation
    q_previous_cmd.normalize();
    quaternionTFToMsg(q_previous_cmd, absolute_pose_cmd_.pose.orientation);  // Stuff it back into the pose cmd

    // POSITION
    // Again, in "temoto_command_frame" frame
    incremental_position_cmd_.vector.x = pos_scale_ * x_pos;  // X is fwd/back in temoto_command_frame
    incremental_position_cmd_.vector.y = pos_scale_ * y_pos;  // Y is left/right
    incremental_position_cmd_.vector.z = pos_scale_ * z_pos;  // Z is up/down

    // Incoming position cmds are in the temoto_command_frame
    // So convert them the frame of absolute_pose_cmd_
    // We need an intermediate variable here so that incremental_position_cmd_
    // doesn't get overwritten.
    geometry_msgs::Vector3Stamped incoming_position_cmd = incremental_position_cmd_;

    geometry_msgs::TransformStamped prev_frame_to_new;
    // Make sure the transform is available, otherwise skip updating the pose
    if (calculateTransform(incoming_position_cmd.header.frame_id, absolute_pose_cmd_.header.frame_id,
                           prev_frame_to_new))
    {
      tf2::doTransform(incoming_position_cmd, incoming_position_cmd, prev_frame_to_new);

      absolute_pose_cmd_.pose.position.x += incoming_position_cmd.vector.x;
      absolute_pose_cmd_.pose.position.y += incoming_position_cmd.vector.y;
      absolute_pose_cmd_.pose.position.z += incoming_position_cmd.vector.z;
    }
  }
}

/** Callback function for Griffin Powermate events subscriber.
 *  It either reacts to push button being pressed or it updates the scaling
 * factor.
 *  @param powermate temoto::Dial message published by griffin_powermate node.
 */
void Teleoperator::processPowermate(griffin_powermate::PowermateEvent powermate)
{
  if (powermate.push_state_changed)  // if push event (i.e. press or depress) has
                                     // occured_occured
  {
    if (powermate.is_pressed)  // if the push button has been pressed
    {
      callRobotMotionInterface(common_commands::EXECUTE);  // makes the service
                                                           // request to move the
                                                           // robot, according to
                                                           // prior plan
    }
    return;
  }
  else  // if push event did not occur, it had to be turning
  {
    // Calculate step size depending on the current pos_scale_ value. Negating
    // direction means that clock-wise is zoom-in/scale-down and ccw is
    // zoom-out/scale-up
    // The smaller the pos_scale_, the smaller the step. log10 gives the order
    // of magnitude
    double step = (-powermate.direction) * pow(10, floor(log10(pos_scale_)) - 1.);
    if (step < -0.09)
      step = -0.01;                  // special case for when scale is 1; to ensure that step is
                                     // never larger than 0.01
    pos_scale_ = pos_scale_ + step;  // increase/decrease scale

    step = (-powermate.direction) * pow(10, floor(log10(rot_scale_)) - 1.);
    rot_scale_ = rot_scale_ + step;

    // cap the scales
    if (pos_scale_ > pos_scale_max_)
      pos_scale_ = pos_scale_max_;
    if (rot_scale_ > rot_scale_max_)
      rot_scale_ = rot_scale_max_;

    return;
  }  // else
}  // end processPowermate()

/** Callback function for /nav_collision_warning/spd_fraction
 *  Scales teleoperated velocity cmds when an obstacle is close.
 *  @param msg from the nav_collision_warning node
 */
void Teleoperator::navCollisionCallback(const std_msgs::Float64::ConstPtr& msg)
{
  nav_speed_fraction_ = msg->data;
}

/** Callback function for /temoto/sleep
 *  If user publishes true, temoto will spin without sending commands
 *  @param msg from the nav_collision_warning node
 */
void Teleoperator::sleepCallback(const std_msgs::Bool::ConstPtr& msg)
{
  temoto_sleep_ = msg->data;
}

/** Callback function for /temoto/voice_commands.
 *  Executes published voice command.
 *  @param voice_command contains the specific command as an unsigned integer.
 */
void Teleoperator::processStringCommand(std_msgs::String voice_command)
{
  // Do nothing if user put Temoto in sleep mode
  if (!temoto_sleep_)
  {
    //////////////////////////////////////////////////
    //  Stop jogging (jogging preempts other commands)
    //////////////////////////////////////////////////

    if (voice_command.data == "point to point mode")
    {
      ROS_INFO("Switching to point-to-point mode");
      in_jog_mode_ = false;
      resetEEGraphicPose();
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
      callRobotMotionInterface(common_commands::ABORT);

      // Publish to tell non-Temoto actions to abort
      std_msgs::String s;
      s.data = common_commands::ABORT;
      pub_abort_.publish(s);

      return;
    }

    ////////////////////////////////////
    //  Handle all commands except ABORT
    ////////////////////////////////////

    if (voice_command.data == "manipulation")  // Switch over to manipulation (MoveIt!) mode
    {
      if (!enable_manipulation_)
      {
        ROS_INFO_STREAM("Manipulation was not enabled in the yaml file.");
        return;
      }

      // If not already in manipulation mode
      if (current_nav_or_manip_mode_ == NAVIGATION)
      {
        // Make sure we aren't in jog mode. Don't want to start jogging an arm
        // suddenly
        ROS_INFO("Switching out of jog mode");
        in_jog_mode_ = false;
        setScale();

        ROS_INFO("Going into MANIPULATION mode  ...");
        current_nav_or_manip_mode_ = MANIPULATION;
        resetEEGraphicPose();
        setScale();
      }
      else
        ROS_INFO("Already in manipulation mode.");
      return;
    }
    else if (voice_command.data == "navigation")  // Switch to navigation mode
    {
      if (!enable_navigation_)
      {
        ROS_INFO_STREAM("Navigation was not enabled in the yaml file.");
        return;
      }

      // If not already in nav mode
      if (current_nav_or_manip_mode_ == MANIPULATION)
      {
        // Make sure we aren't in jog mode, for safety
        ROS_INFO("Switching out of jog mode");
        resetEEGraphicPose();
        in_jog_mode_ = false;
        ROS_INFO("Going into NAVIGATION mode  ...");
        current_nav_or_manip_mode_ = NAVIGATION;

        bool adjust_camera = true;
        setGraphicsFramesStatus(adjust_camera);
        resetEEGraphicPose();
        setScale();
      }
      else
        ROS_INFO("Already in navigation mode.");
      return;
    }
    else if (voice_command.data == "next end effector")
    {
      ROS_INFO("Controlling the next EE from yaml file ...");

      // No jogging, initially, for safety
      in_jog_mode_ = false;
      switchEE();

      return;
    }
    else if (voice_command.data == "enable compliance")
    {
      if (!in_jog_mode_)
        ROS_INFO_STREAM("Compliance is only available in jog mode.");
      if (!end_effector_parameters_.allow_compliant_jog.at(current_movegroup_ee_index_))
        ROS_INFO_STREAM("Compliance was not enabled in the YAML file.");
      else
      {
        ROS_INFO_STREAM("Enabling compliance");
        enable_compliance_ = true;
      }

      return;
    }
    else if (voice_command.data == "disable compliance")
    {
      if (!in_jog_mode_)
        ROS_INFO_STREAM("Compliance is only available in jog mode.");
      if (!end_effector_parameters_.allow_compliant_jog.at(current_movegroup_ee_index_))
        ROS_INFO_STREAM("Compliance was not enabled in the YAML file.");
      else
      {
        ROS_INFO_STREAM("Disabling compliance");
        enable_compliance_ = false;
      }

      return;
    }
    else if (voice_command.data == "close gripper")
    {
      ROS_INFO("Closing the gripper ...");

      if (end_effector_parameters_.gripper_topics.at(current_movegroup_ee_index_) != "")
        gripper_interface_->close(end_effector_parameters_.gripper_topics.at(current_movegroup_ee_index_));
      else
        ROS_WARN_STREAM("A gripper topic is not defined for this end-effector. Check yaml file.");

      return;
    }
    else if (voice_command.data == "open gripper")
    {
      ROS_INFO("Opening the gripper ...");

      if (end_effector_parameters_.gripper_topics.at(current_movegroup_ee_index_) != "")
        gripper_interface_->open(end_effector_parameters_.gripper_topics.at(current_movegroup_ee_index_));
      else
        ROS_WARN_STREAM("A gripper topic is not defined for this end-effector. Check yaml file.");

      return;
    }

    // Avoid planning, executing, etc. while in jog mode
    if (in_jog_mode_)
      return;

    // There's nothing blocking any arbitrary command
    else
    {
      if (voice_command.data == "jog mode")
      {
        ROS_INFO("Switching to jog mode");
        in_jog_mode_ = true;
        resetEEGraphicPose();
        setScale();

        return;
      }
      else if (voice_command.data == "robot please plan")
      {
        ROS_INFO("Planning ...");
        callRobotMotionInterface(common_commands::PLAN);
        return;
      }
      else if (voice_command.data == "robot please execute")
      {
        ROS_INFO("Executing last plan ...");
        callRobotMotionInterface(common_commands::EXECUTE);
        resetEEGraphicPose();
        return;
      }
      else if (voice_command.data == "base move")
      {
        ROS_INFO("Planning and moving ...");
        resetEEGraphicPose();
        callRobotMotionInterface(common_commands::GO);
        return;
      }

      else
      {
        ROS_INFO_STREAM("Unknown voice command: " << voice_command.data);
      }
    }  // End of handling non-Abort commands
  }    // End of !temoto_sleep_

  return;
}

/** Puts the latest private variable values into temoto/status message.
 *  This is used in marker publication.
 *  @return void.
 */
void Teleoperator::setGraphicsFramesStatus(bool adjust_camera)
{
  graphics_and_frames_.latest_status_.in_navigation_mode = (current_nav_or_manip_mode_ == NAVIGATION);
  graphics_and_frames_.latest_status_.scale_by = pos_scale_;
  graphics_and_frames_.latest_status_.commanded_pose = absolute_pose_cmd_;
  graphics_and_frames_.latest_status_.end_effector_pose =
      end_effector_parameters_.arm_interface_ptrs.at(current_movegroup_ee_index_)->movegroup_.getCurrentPose();
  graphics_and_frames_.latest_status_.current_movegroup_ee_index = current_movegroup_ee_index_;

  graphics_and_frames_.adjust_camera_ = adjust_camera;

  // Sometimes, false may be returned if a pose is not initialized
  while (!graphics_and_frames_.crunch())
    ros::Duration(0.01).sleep();

  return;
}  // end setGraphicsFramesStatus()

void Teleoperator::setScale()
{
  if (current_nav_or_manip_mode_ == NAVIGATION)
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
  else  // manipulation
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

  // Set the max scaling factors as multiples of the defaults
  pos_scale_max_ = 5 * pos_scale_;
  rot_scale_max_ = 5 * rot_scale_;
}

// Switch to the next available EE
void Teleoperator::switchEE()
{
  if (current_movegroup_ee_index_ < end_effector_parameters_.ee_names.size() - 1)
    ++current_movegroup_ee_index_;
  else
    current_movegroup_ee_index_ = 0;

  // Set up jog msgs for the current EE
  jog_twist_cmd_.header.frame_id = end_effector_parameters_.ee_names.at(current_movegroup_ee_index_);
  joint_jog_cmd_.joint_names[0] = end_effector_parameters_.wrist_joint_names.at(current_movegroup_ee_index_);

  graphics_and_frames_.latest_status_.moveit_planning_frame =
      end_effector_parameters_.arm_interface_ptrs.at(current_movegroup_ee_index_)->movegroup_.getPlanningFrame();

  // Set camera at new EE
  bool adjust_camera = true;
  setGraphicsFramesStatus(adjust_camera);

  resetEEGraphicPose();

  // Reset the scale when switching EE's
  setScale();
}

// Calculate a transform between 2 frames
bool Teleoperator::calculateTransform(std::string source_frame, std::string target_frame,
                                      geometry_msgs::TransformStamped& transform)
{
  bool success = false;

  // If the strings were valid
  if (source_frame.length() > 0 && target_frame.length() > 0)
  {
    // Check for and remove leading '/'
    if (source_frame.at(0) == '/')
      source_frame.erase(0, 1);
    if (target_frame.at(0) == '/')
      target_frame.erase(0, 1);

    try
    {
      transform = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(0.1));
      success = true;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  return success;
}

// Reset the end-effector graphic to current robot pose
void Teleoperator::resetEEGraphicPose()
{
  if (current_nav_or_manip_mode_ == NAVIGATION)  // Navigation --> Center on base_frame
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
    absolute_pose_cmd_ =
        end_effector_parameters_.arm_interface_ptrs.at(current_movegroup_ee_index_)->movegroup_.getCurrentPose();
}

// MAIN
int main(int argc, char** argv)
{
  ros::init(argc, argv, "start_teleop");

  ros::NodeHandle n;

  ros::Rate node_rate(90.);

  // An async spinner is needed for moveit's MoveGroup::plan()
  ros::AsyncSpinner spinner(4);
  spinner.start();

  Teleoperator temoto_teleop;

  while (ros::ok())
  {
    // Don't do anything if the user put Temoto in sleep mode
    if (!temoto_teleop.temoto_sleep_)
    {
      // Jog?
      if (temoto_teleop.in_jog_mode_)
        temoto_teleop.callRobotMotionInterface(common_commands::GO);
      else
      {
        // Update poses, scale, and nav-or-manip mode for the frames calculation
        temoto_teleop.setGraphicsFramesStatus(false);
        temoto_teleop.graphics_and_frames_.crunch();
      }
    }

    node_rate.sleep();
  }  // end main loop

  return 0;
}  // end main