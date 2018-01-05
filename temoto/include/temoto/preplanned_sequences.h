#ifndef PREPLANNED_SEQUENCES_H
#define PREPLANNED_SEQUENCES_H

// ROS includes
#include <actionlib/server/simple_action_server.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "ros/ros.h"

// temoto includes
#include "temoto/preplanned_sequences/close_gripper.h"
#include "temoto/preplanned_sequences/go_to_laser_scan.h"
#include "temoto/preplanned_sequences/open_gripper.h"
#include "temoto/preplanned_sequences/robot_please_approach.h"
#include "temoto/preplanned_sequences/robot_please_scan.h"
#include "temoto/preplanned_sequences/robot_push_button.h"
#include "temoto/low_level_cmds.h"
#include "temoto/PreplannedSequenceAction.h"  // ROS Action. This is how the preplanned sequence gets triggered

// Other includes


class preplanned_sequence {
  protected:
    ros::NodeHandle n_;

    // Listen for a new "preplanned sequence goal", which is a string
    actionlib::SimpleActionServer<temoto::PreplannedSequenceAction> sequence_server_;

    // create messages that are used to published feedback/result
    temoto::PreplannedSequenceResult result_;

  public:
    // Constructor
    preplanned_sequence()
     : sequence_server_(n_, "temoto/preplanned_sequence", boost::bind(&preplanned_sequence::execute_CB_, this, _1), false)
    {
      // Listen for abort commands
      abort_sub_ = n_.subscribe("temoto/abort", 1, &preplanned_sequence::abort_cb_, this);

      sequence_server_.start();

    }

    // Action server CB: launch the preplanned sequence, as specified by the incoming command.
    // Action server allows interruption.
    void execute_CB_(const temoto::PreplannedSequenceGoalConstPtr& goal);

  private:
    // Set a flag that the "abort" command was heard
    ros::Subscriber abort_sub_;
    void abort_cb_(const std_msgs::String::ConstPtr& msg);
    bool abort_ = false;
};

#endif
