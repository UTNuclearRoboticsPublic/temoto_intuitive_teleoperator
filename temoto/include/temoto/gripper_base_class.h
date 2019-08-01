#ifndef GRIPPER_BASE_CLASS_H
#define GRIPPER_BASE_CLASS_H

#include "ros/ros.h"

class GripperBaseClass
{
public:
  GripperBaseClass();

  /* use virtual otherwise linker will try to perform static linkage */
  virtual void close();
  virtual void open();

private:
};

#endif
