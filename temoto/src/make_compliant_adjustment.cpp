#include "temoto/make_compliant_adjustment.h"

// Add small deltas to a Cartesian jogging command.
bool CompliantAdjustment::cartesianCompliantAdjustment(geometry_msgs::TwistStamped &jog_command, int arm_index)
{

	// Transform from the jogging frame to force/torque frame
	jog_command.header.frame_id = compliance_data_vectors_[arm_index].jog_frame_;
  transformTwist(jog_command, compliance_data_vectors_[arm_index].force_torque_frame_);

  std::vector<double> velocity = convertTwistToVector( jog_command.twist );

  //ROS_INFO_STREAM_THROTTLE(0.1, velocity[0] << "  " << velocity[1] << "  " << velocity[2] << "  " << velocity[3] << "  " << velocity[4] << "  " << velocity[5]);

  // Add the compliance velocity to nominal velocity, and check for force/torque limits
  compliance_data_vectors_[arm_index].compliance_status_ =
    compliance_data_vectors_[arm_index].compliant_control_instance_.getVelocity(velocity, compliance_data_vectors_[arm_index].force_torque_data_, velocity);

  //ROS_INFO_STREAM_THROTTLE(5, velocity[0] << "  " << velocity[1] << "  " << velocity[2] << "  " << velocity[3] << "  " << velocity[4] << "  " << velocity[5]);

  //ROS_ERROR_STREAM( "ft_[0]: " << compliance_data_vectors_[arm_index].force_torque_data_.wrench.force.x );
  //ROS_ERROR_STREAM( "ft_[1]: " << compliance_data_vectors_[arm_index].force_torque_data_.wrench.force.y );
  //ROS_ERROR_STREAM( "ft_[2]: " << compliance_data_vectors_[arm_index].force_torque_data_.wrench.force.z );

  // Put back into TwistStamped form (which the arm jogger expects)
  jog_command.twist = convertVectorToTwist( velocity );
  jog_command.header.frame_id = compliance_data_vectors_[arm_index].force_torque_frame_;

  // Transform from the force/torque frame back to jogging frame
  transformTwist(jog_command, compliance_data_vectors_[arm_index].jog_frame_);

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

// Transform a twist into given frame
bool CompliantAdjustment::transformTwist(geometry_msgs::TwistStamped& twist, std::string desired_frame)
{
  if ( twist.header.frame_id == desired_frame )
    return true;

  geometry_msgs::TransformStamped current_frame_to_target;
  try {
    current_frame_to_target =
        tf_buffer_.lookupTransform(twist.header.frame_id, desired_frame, ros::Time(0), ros::Duration(1.0));

  } catch (tf2::TransformException ex) {
    ROS_ERROR("[make_compliant_adjustment] Failed to transform twist to EE frame.");
    return false;
  }

  // There is no method to transform a Twist, so break it into vectors and transform one at a time
  geometry_msgs::Vector3Stamped linear;
  linear.vector = twist.twist.linear;
  linear.header.frame_id = twist.header.frame_id;
  tf2::doTransform(linear, linear, current_frame_to_target);

  geometry_msgs::Vector3Stamped angular;
  angular.vector = twist.twist.angular;
  angular.header.frame_id = twist.header.frame_id;
  tf2::doTransform(linear, linear, current_frame_to_target);

  // Put these components back into a WrenchStamped
  twist.header.frame_id = desired_frame;
  twist.twist.linear = linear.vector;
  twist.twist.angular = angular.vector;

  return true;
}

// Transform a wrench into given frame
bool CompliantAdjustment::transformWrench(geometry_msgs::WrenchStamped& wrench, std::string desired_frame)
{
  if ( wrench.header.frame_id == desired_frame )
    return true;

  geometry_msgs::TransformStamped current_frame_to_target;
  try {
    current_frame_to_target =
        tf_buffer_.lookupTransform(wrench.header.frame_id, desired_frame, ros::Time(0), ros::Duration(1.0));

  } catch (tf2::TransformException ex) {
    ROS_ERROR("[make_compliant_adjustment] Failed to transform wrench to EE frame.");
    return false;
  }

  // There is no method to transform a Wrench, so break it into vectors and transform one at a time
  geometry_msgs::Vector3Stamped force_vector;
  force_vector.vector = wrench.wrench.force;
  force_vector.header.frame_id = wrench.header.frame_id;
  tf2::doTransform(force_vector, force_vector, current_frame_to_target);

  geometry_msgs::Vector3Stamped torque_vector;
  torque_vector.vector = wrench.wrench.torque;
  torque_vector.header.frame_id = wrench.header.frame_id;
  tf2::doTransform(torque_vector, torque_vector, current_frame_to_target);

  // Put these components back into a WrenchStamped
  wrench.header.frame_id = desired_frame;
  wrench.wrench.force = force_vector.vector;
  wrench.wrench.torque = torque_vector.vector;

  return true;
}