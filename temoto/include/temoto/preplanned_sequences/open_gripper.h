#ifndef OPEN_GRIPPER_H
#define OPEN_GRIPPER_H

#include <robotiq_c_model_control/CModel_robot_output.h> // Include the RCGripper msg type
#include <ros/ros.h>

namespace open_gripper
{
  int open_gripper(ros::Publisher& gripper_publisher);
}

#endif
