#include "temoto/make_compliant_adjustment.h"

// Add small deltas to a Cartesian jogging command.
bool CompliantAdjustment::cartesianCompliantAdjustment(geometry_msgs::TwistStamped &jog_command)
{

  return true;
}

// CB for force/torque data of arm0
void CompliantAdjustment::ftCB0(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  compliance_data_vectors_[0].force_torque_data_ = *msg;
  compliance_data_vectors_[0].force_torque_data_.header.frame_id = compliance_data_vectors_.at(0).force_torque_frame_;
}

// CB for force/torque data of arm1
void CompliantAdjustment::ftCB1(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  compliance_data_vectors_[1].force_torque_data_ = *msg;
  compliance_data_vectors_[1].force_torque_data_.header.frame_id = compliance_data_vectors_.at(1).force_torque_frame_;
}