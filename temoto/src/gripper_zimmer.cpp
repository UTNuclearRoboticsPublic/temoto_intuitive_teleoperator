#include "temoto/gripper_zimmer.h"

namespace grippers
{
GripperZimmer::GripperZimmer(std::string gripper_topic):
action_client_(gripper_topic, true)
{
  // TODO think about this timeout more
  action_client_.waitForServer();
}

void GripperZimmer::close()
{
  action_client_.cancelAllGoals();
  control_msgs::GripperCommandGoal goal;
  goal.command.position = zimmer::ZIMMER_CLOSE_POSITION;
  action_client_.sendGoal(goal);
  // bool finished_before_timeout = action_client_.waitForResult(ros::Duration(30.0));
}

void GripperZimmer::open()
{
  action_client_.cancelAllGoals();
  control_msgs::GripperCommandGoal goal;
  goal.command.position = zimmer::ZIMMER_OPEN_POSITION;
  action_client_.sendGoal(goal);
  // bool finished_before_timeout = action_client_.waitForResult(ros::Duration(30.0));
}
}
