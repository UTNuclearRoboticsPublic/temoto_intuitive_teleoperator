#ifndef CLOSE_GRIPPER_H
#define CLOSE_GRIPPER_H

#include <robotiq_c_model_control/CModel_robot_output.h> // Include the RCGripper msg type
#include <ros/ros.h>

namespace close_gripper
{
  int close_gripper(ros::Publisher& gripper_publisher);
}

#endif