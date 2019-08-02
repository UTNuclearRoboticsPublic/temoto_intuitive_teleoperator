#ifndef GRIPPER_ROBOTIQ_H
#define GRIPPER_ROBOTIQ_H

#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h"  // RCGripper msg type
#include "ros/ros.h"
#include "temoto/gripper_base_class.h"

namespace grippers
{
class GripperRobotiq : public GripperBaseClass
{
public:
  GripperRobotiq(std::string gripper_topic);

  virtual void close();

  virtual void open();

private:
  ros::Publisher gripper_publisher_;

  ros::NodeHandle nh_;
};
}

#endif
