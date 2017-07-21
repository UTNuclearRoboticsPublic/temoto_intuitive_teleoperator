
#include "temoto/preplanned_sequences/open_hand.h"

int open_hand::open_hand()
{
  ros::NodeHandle nh;

  ros::Publisher gripper_publisher = nh.advertise<grasp_interface::RCGripperCommand>("grip_command", 5);

  // Wait for the publisher to become ready
  ros::Duration(1).sleep();

  grasp_interface::RCGripperCommand gripper_msg;
  gripper_msg.force = 255; // Max force
  gripper_msg.speed = 255;
  gripper_msg.position = 255; // open
  gripper_publisher.publish(gripper_msg);

  ros::Duration(4).sleep();

  return 0;
}
