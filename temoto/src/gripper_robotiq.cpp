
#include "temoto/gripper_robotiq.h"

namespace grippers
{
GripperRobotiq::GripperRobotiq(std::string gripper_topic)
{
  gripper_publisher_ = nh_.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>(gripper_topic, 1);
}

void GripperRobotiq::close()
{
  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output gripper_msg;
  gripper_msg.rPR = 255;
  gripper_msg.rACT = 1;
  gripper_msg.rGTO = 1;
  gripper_msg.rATR = 0;
  gripper_msg.rSP = 255;
  gripper_msg.rFR = 150;

  gripper_publisher_.publish(gripper_msg);
}

void GripperRobotiq::open()
{
  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output gripper_msg;
  gripper_msg.rPR = 0;
  gripper_msg.rACT = 1;
  gripper_msg.rGTO = 1;
  gripper_msg.rATR = 0;
  gripper_msg.rSP = 255;
  gripper_msg.rFR = 150;

  gripper_publisher_.publish(gripper_msg);
}
}
