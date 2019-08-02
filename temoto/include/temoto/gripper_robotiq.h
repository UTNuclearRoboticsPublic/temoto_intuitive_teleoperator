#ifndef GRIPPER_ROBOTIQ_H
#define GRIPPER_ROBOTIQ_H

#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h"  // RCGripper msg type
#include "ros/ros.h"
#include "temoto/gripper_base_class.h"

class GripperRobotiq : public GripperBaseClass
{
public:
  virtual void close()
  {
    ROS_INFO_STREAM("Closing");
  }

  virtual void open()
  {
    ROS_INFO_STREAM("Opening");
  }

private:
  //std::vector<std::shared_ptr<ros::Publisher>> gripper_publishers_;
};

#endif