
#include "temoto/preplanned_sequences/close_hand.h"

int close_hand::close_hand()
{
  ros::NodeHandle nh;

  ros::Publisher gripper_publisher = nh.advertise<robotiq_c_model_control::CModel_robot_output>("/CModelRobotOutput", 5);

  // Wait for the publisher to become ready
  ros::Duration(1).sleep();

  robotiq_c_model_control::CModel_robot_output gripper_msg;
  gripper_msg.rPR = 255;
  gripper_publisher.publish(gripper_msg);

  ros::Duration(4).sleep();

  return 0;
}
