
#include "temoto/preplanned_sequences/go_to_laser_scan.h"


// Send the left arm to the laser scan pose
int go_to_laser_scan::go_to_laser_scan(ros::NodeHandle& n)
{
  ROS_INFO_STREAM("Moving to a laser scan pose.");

  // Call /arm_services/move_to_left_laser_scan
  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/arm_services/move_to_left_laser_scan");

  std_srvs::Empty srv;

  if (!client.call(srv))
  {
    ROS_WARN("[arm_services] Could not call /arm_services/move_to_left_laser_scan");
  }

  return 0;
}
