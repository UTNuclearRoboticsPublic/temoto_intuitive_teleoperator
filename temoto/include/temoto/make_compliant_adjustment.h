// Copyright (c) 2018, The University of Texas at Austin
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

/** @file make_compliant_adjustment.h
 *
 *  @brief Take a TwistStamped jogging message. Make compliant adjustments.
 *
 *  @author andyz(at)utexas.edu
 */

#ifndef MAKE_COMPLIANT_ADJUSTMENT_H
#define MAKE_COMPLIANT_ADJUSTMENT_H

#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "jog_arm/compliant_control.h"
#include "ros/ros.h"
#include "temoto/get_ros_params.h"
#include <tf2_ros/transform_listener.h>


// This class holds all compliance data for one arm
class SingleArmComplianceData
{
public:
  SingleArmComplianceData(int ee_index, geometry_msgs::WrenchStamped &bias) :
    compliant_control_instance_(
      stiffness_,
      deadband_,
      end_condition_wrench_,
      filter_param_,
      bias,
      highest_allowable_force_,
      highest_allowable_torque_)
  {
  	force_torque_frame_ = get_ros_params::getStringParam("temoto/ee/ee" + std::to_string(ee_index) + "/force_torque_frame", n_);
    jog_frame_ = get_ros_params::getStringParam("temoto/ee/ee" + std::to_string(ee_index) + "/end_effector", n_);
  };

  ros::NodeHandle n_;

  // Publish velocity cmd(s) to the jog_arm node(s)
  ros::Publisher velocity_pub_;

  // Status, e.g. safety threshold exceeded
  compliant_control::ExitCondition compliance_status_;

  // TF frame of force/torque data
  std::string force_torque_frame_ = "";

  // Frame of jogging
  std::string jog_frame_ = "";

  // Key equation: compliance_velocity[i] = wrench[i]/stiffness[i]
  std::vector<double> stiffness_{4000, 4000, 4000, 4000, 4000, 4000};

  // Related to the cutoff frequency of the low-pass filter.
  double filter_param_ = 10.;

  // Deadband for force/torque measurements
  std::vector<double> deadband_{10, 10, 10, 10, 10, 10};

  // Stop when force exceeds X N or torque exceeds X Nm in any dimension.
  // The robot controller's built-in safety limits are ~90 N, ? Nm
  std::vector<double> end_condition_wrench_{80, 80, 80, 60, 60, 60};

  // Highest allowable force/torque across all dimensions.
  double highest_allowable_force_ = 88, highest_allowable_torque_ = 50;

  // Current force/torque data
  geometry_msgs::WrenchStamped force_torque_data_;

  // Outgoing velocity msg
  std::vector<double> velocity_out_{0, 0, 0, 0, 0, 0};

  // An object to do compliance calculations
  compliant_control::CompliantControl compliant_control_instance_;
};


class CompliantAdjustment
{
public:
  CompliantAdjustment() : tf2_listener_(tf_buffer_)
  {
  }

  bool addCompliantEndEffector(int ee_index)
  {
    std::string force_torque_topic = get_ros_params::getStringParam("temoto/ee/ee" + std::to_string(ee_index) + "/force_torque_topic", n_);

    // Allocate space for the compliance data
    geometry_msgs::WrenchStamped empty_bias;
    compliance_data_vectors_.push_back( SingleArmComplianceData(ee_index, empty_bias) );

    // Listen to wrench data from a force/torque sensor.
    // Unfortunately this is hard-coded to 2 callback functions because programmatically generating multiple callbacks ain't easy.
    if (ee_index == 0)
    {
      ros::Subscriber sub0 = n_.subscribe(
        force_torque_topic,
        1,
        &CompliantAdjustment::ftCB0,
        this);

      force_torque_subs_.push_back(sub0);
    }

    if (ee_index == 1)
    {
      ros::Subscriber sub1 = n_.subscribe(
        force_torque_topic,
        1,
        &CompliantAdjustment::ftCB1,
        this);

      force_torque_subs_.push_back(sub1);
    }

    // Get initial bias reading from the force/torque sensor
    while ( ros::ok() && compliance_data_vectors_[ee_index].force_torque_data_.header.frame_id == "" )
    {
      ros::Duration(0.1).sleep();
      ROS_INFO_STREAM_THROTTLE(2, "[make_compliant_adjustment] Waiting for first force/torque data for arm " << ee_index);
    }
    // Re-create the compliance object with updated bias. Could do this more efficiently.
    compliance_data_vectors_.at(ee_index) = ( SingleArmComplianceData(ee_index, compliance_data_vectors_[ee_index].force_torque_data_) );
    ROS_INFO_STREAM("[make_compliant_adjustment] Received first force/torque data for arm " << ee_index);
  }

  bool cartesianCompliantAdjustment(geometry_msgs::TwistStamped &jog_command, int arm_index_);

private:
  ros::NodeHandle n_;

  int num_arms_;

  // Hold compliance data for each arm
  std::vector<SingleArmComplianceData> compliance_data_vectors_;

  // Subscribe to force/torque topics
  // Unfortunately this is hard-coded to 2 callback functions because programmatically generating multiple callbacks ain't easy.
  std::vector<ros::Subscriber> force_torque_subs_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  // CBs for force/torque data
  void ftCB0(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void ftCB1(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  // Datatype conversions
  geometry_msgs::Twist convertVectorToTwist(const std::vector<double> &v);
  std::vector<double> convertTwistToVector(const geometry_msgs::Twist &twist);

  bool transformTwist(geometry_msgs::TwistStamped& twist, std::string desired_frame);
};

#endif