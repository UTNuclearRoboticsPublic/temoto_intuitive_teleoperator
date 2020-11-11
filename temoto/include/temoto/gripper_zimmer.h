#pragma once

#include "ros/ros.h"
#include "temoto/gripper_base_class.h"
#include "control_msgs/GripperCommandAction.h"
#include <actionlib/client/simple_action_client.h>

namespace grippers
{
namespace zimmer
{
  // Hardcode these, values from the SRDF
  constexpr double ZIMMER_OPEN_POSITION = 0.0008;
  constexpr double ZIMMER_CLOSE_POSITION = 0.0399;
}

class GripperZimmer : public GripperBaseClass
{
public:
  GripperZimmer(std::string gripper_topic);

  virtual void close();

  virtual void open();

private:
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> action_client_;
};
}
