#ifndef GO_TO_LASER_SCAN_H
#define GO_TO_LASER_SCAN_H

#include "ros/ros.h"
#include <std_srvs/Empty.h>

namespace go_to_laser_scan
{
  int go_to_laser_scan(ros::NodeHandle& n);
}

#endif
