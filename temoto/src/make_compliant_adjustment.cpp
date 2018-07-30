#include "temoto/make_compliant_adjustment.h"

// Add small deltas to a Cartesian jogging command.
bool CompliantAdjustment::cartesianCompliantAdjustment(geometry_msgs::TwistStamped &jog_command, int arm_index)
{
  std::vector<double> velocity = convertTwistToVector( jog_command.twist );

  // Add the compliance velocity to nominal velocity, and check for force/torque limits
  compliance_data_vectors_[arm_index].compliance_status_ =
    compliance_data_vectors_[arm_index].compliant_control_instance_.getVelocity(velocity, compliance_data_vectors_[arm_index].force_torque_data_, velocity);

  // Put back into TwistStamped form (which the arm jogger expects)
  jog_command.twist = convertVectorToTwist( velocity );

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

// Datatype conversion
geometry_msgs::Twist CompliantAdjustment::convertVectorToTwist(const std::vector<double> &v)
{
  geometry_msgs::Twist twist;
  twist.linear.x = v.at(0);
  twist.linear.y = v.at(1);
  twist.linear.z = v.at(2);
  twist.angular.x = v.at(3);
  twist.angular.y = v.at(4);
  twist.angular.z = v.at(5);

  return twist;
}

// Datatype conversion
std::vector<double> CompliantAdjustment::convertTwistToVector(const geometry_msgs::Twist &twist)
{
  std::vector<double> v(6);
  v[0] = twist.linear.x;
  v[1] = twist.linear.y;
  v[2] = twist.linear.z;
  v[3] = twist.angular.x;
  v[4] = twist.angular.y;
  v[5] = twist.angular.z;

  return v; 
}