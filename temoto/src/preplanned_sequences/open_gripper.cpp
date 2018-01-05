
#include "temoto/preplanned_sequences/open_gripper.h"

int open_hand::open_hand()
{
  ros::NodeHandle nh;

  ros::Publisher gripper_publisher = nh.advertise<robotiq_c_model_control::CModel_robot_output>("/CModelRobotOutput", 5);

  // Wait for the publisher to become ready
  ros::Duration(2).sleep();

  robotiq_c_model_control::CModel_robot_output gripper_msg;
  gripper_msg.rPR = 0;
  // Spam it a few times to make sure it's heard
  gripper_publisher.publish(gripper_msg);
  ros::Duration(0.1).sleep();
  gripper_publisher.publish(gripper_msg);
  ros::Duration(0.1).sleep();
  gripper_publisher.publish(gripper_msg);
  ros::Duration(0.1).sleep();

  return 0;
}
