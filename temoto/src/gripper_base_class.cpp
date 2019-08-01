
#include "temoto/gripper_base_class.h"

using namespace std;

extern "C" GripperBaseClass* create_object()
{
  return new GripperBaseClass;
}

extern "C" void destroy_object( GripperBaseClass* object )
{
  delete object;
}

GripperBaseClass::GripperBaseClass()
{
}

void GripperBaseClass::close()
{
  ROS_ERROR_STREAM("Closing");
}

void GripperBaseClass::open()
{
  ROS_ERROR_STREAM("Opening");
}
