#ifndef GRIPPER_ROBOTIQ_H
#define GRIPPER_ROBOTIQ_H

#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h"  // RCGripper msg type
#include "ros/ros.h"
#include "temoto/gripper_base_class.h"

class GripperRobotiq : GripperBaseClass
{
public:
  GripperRobotiq();

  /* use virtual otherwise linker will try to perform static linkage */
  virtual void close();
  virtual void open();

private:
  //std::vector<std::shared_ptr<ros::Publisher>> gripper_publishers_;
};

#endif
