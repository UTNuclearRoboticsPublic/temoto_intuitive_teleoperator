#ifndef GRIPPER_BASE_CLASS_H
#define GRIPPER_BASE_CLASS_H

#include "ros/ros.h"

namespace grippers
{
class GripperBaseClass
{
public:
  virtual void close();
  virtual void open();

private:
};
}

#endif
