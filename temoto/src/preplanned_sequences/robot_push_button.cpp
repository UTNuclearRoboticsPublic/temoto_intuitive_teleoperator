#include "temoto/preplanned_sequences/robot_push_button.h"

// Send a service request to start pushing a button

int robot_push_button::robot_push_button(ros::NodeHandle &n)
{

  ros::ServiceClient client = n.serviceClient<std_srvs::EmptyRequest>("/arm_services/push_button");

  std_srvs::Empty srv;

  if (!client.call(srv))
  {
    ROS_ERROR("[robot_push_button] Failed to call service");
    return 1;
  }

  return 0;
}
