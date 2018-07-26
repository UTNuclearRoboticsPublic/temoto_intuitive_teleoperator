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
#include "jog_arm/compliant_control.h"
#include "ros/ros.h"
#include "temoto/get_ros_params.h"


// This class holds all compliance data for one arm
class SingleArmComplianceData
{
public:
  SingleArmComplianceData(int ee_index) :
    compliance_object_(stiffness_,
    deadband_, end_condition_wrench_,
    filter_param_,
    bias_,
    highest_allowable_force_,
    highest_allowable_torque_ )
  {
  	force_torque_frame_ = get_ros_params::getStringParam("temoto/ee" + std::to_string(ee_index) + "/force_torque_frame", n_);
  };

  ros::NodeHandle n_;

  // An object to do compliance calculations
  compliant_control::CompliantControl compliance_object_;

  // Subscribe to force/torque topic
  ros::Subscriber force_torque_sub_;

  // Publish velocity cmd(s) to the jog_arm node(s)
  ros::Publisher velocity_pub_;

  // Compliance for each arm
  std::vector<compliant_control::ExitCondition> compliance_status_;

  // TF frame of force/torque data
  std::string force_torque_frame_ = "";

  // Key equation: compliance_velocity[i] = wrench[i]/stiffness[i]
  std::vector<double> stiffness_{25000, 25000, 25000, 2000, 2000, 2000};

  // Related to the cutoff frequency of the low-pass filter.
  double filter_param_ = 10.;

  // Deadband for force/torque measurements
  std::vector<double> deadband_{10, 10, 10, 10, 10, 10};

  // Stop when any force exceeds X N, or torque exceeds X Nm
  // The robot controller's built-in safety limits are ~90 N, ? Nm
  std::vector<double> end_condition_wrench_{70, 70, 70, 60, 60, 60};

  // Current force/torque data
  geometry_msgs::WrenchStamped force_torque_data_;

  // Outgoing velocity msg
  std::vector<double> velocity_out_{0, 0, 0, 0, 0, 0};

  geometry_msgs::WrenchStamped bias_;

  double highest_allowable_force_, highest_allowable_torque_;
};


class CompliantAdjustment
{
public:
  CompliantAdjustment()
  {
  	// For each end-effector where compliance is enabled:
    // Read the compliance parameters.
    num_arms_ = get_ros_params::getIntParam("teach_motions/num_arms", n_);

    for (int ee_index=0; ee_index<num_arms_; ++ee_index)
    {
      compliance_data_vectors_.push_back( SingleArmComplianceData(ee_index) );
    	std::string force_torque_topic = get_ros_params::getStringParam("temoto/ee" + std::to_string(ee_index) + "/force_torque_topic", n_);

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
    }

    // Get a bias reading from each sensor
  };

  bool cartesianCompliantAdjustment(geometry_msgs::TwistStamped &jog_command);

private:
  ros::NodeHandle n_;

  int num_arms_;

  // Hold compliance data for each arm
  std::vector<SingleArmComplianceData> compliance_data_vectors_;

  // Subscribe to force/torque topics
  // Unfortunately this is hard-coded to 2 callback functions because programmatically generating multiple callbacks ain't easy.
  std::vector<ros::Subscriber> force_torque_subs_;

  // CBs for force/torque data
  void ftCB0(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void ftCB1(const geometry_msgs::WrenchStamped::ConstPtr& msg);
};

#endif