
#include "temoto/gripper_base_class.h"

namespace grippers
{
void GripperBaseClass::close()
{
  ROS_WARN_STREAM("Invalid gripper type");
}

void GripperBaseClass::open()
{
  ROS_WARN_STREAM("Invalid gripper type");
}
}
