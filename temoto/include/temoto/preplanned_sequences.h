// ROS includes
#include <actionlib/server/simple_action_server.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "ros/ros.h"

// temoto includes
#include "temoto/low_level_cmds.h"
#include "temoto/PreplannedSequenceAction.h"  // Define an action. This is how the sequence gets triggered

// Other includes

class preplanned_sequence {
  public:
    preplanned_sequence();

  private:
    ros::NodeHandle n_;

    // Set a flag that the "abort" command was heard
    ros::Subscriber abort_sub_;
    void abort_cb_(const std_msgs::String::ConstPtr& msg);

    // Listen for a new "preplanned sequence goal", which is a string
    // TO DO: this is not being initialized correctly
    //actionlib::SimpleActionServer<temoto::PreplannedSequenceAction> sequence_server_;

/*
    // Callback. Gets called by an incoming action server goal
    void initiate_sequence_(const temoto::PreplannedSequenceActionGoalConstPtr& goal, actionlib::SimpleActionServer<temoto::PreplannedSequenceAction>* as)
*/
  
   bool abort_ = false;
};
