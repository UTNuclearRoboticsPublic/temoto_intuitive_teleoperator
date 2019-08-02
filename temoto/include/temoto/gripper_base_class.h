#ifndef GRIPPER_BASE_CLASS_H
#define GRIPPER_BASE_CLASS_H

#include "ros/ros.h"

class GripperBaseClass
{
public:
  virtual void close();
  virtual void open();

private:
};

#endif
