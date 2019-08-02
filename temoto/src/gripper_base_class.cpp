
#include "temoto/gripper_base_class.h"

using namespace std;

void GripperBaseClass::close()
{
  ROS_WARN_STREAM("Invalid gripper type");
}

void GripperBaseClass::open()
{
  ROS_WARN_STREAM("Invalid gripper type");
}
