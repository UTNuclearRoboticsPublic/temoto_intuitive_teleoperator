
#include "temoto/preplanned_sequences/open_gripper.h"

int open_gripper::open_gripper(ros::Publisher& gripper_publisher)
{
  robotiq_c_model_control::CModel_robot_output gripper_msg;
  gripper_msg.rPR = 0;
  gripper_msg.rACT = 1;
  gripper_msg.rGTO = 1;
  gripper_msg.rATR = 0;
  gripper_msg.rSP = 255;
  gripper_msg.rFR = 150;

  // Spam it a few times to make sure it's heard
  gripper_publisher.publish(gripper_msg);
  ros::Duration(0.1).sleep();
  gripper_publisher.publish(gripper_msg);
  ros::Duration(0.1).sleep();

  return 0;
}
