// Copyright (c) 2017, The University of Texas at Austin
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
// 
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
// 
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/** @file preplanned_sequences.cpp
 * 
 *  @brief ROS server that executes preplanned sequences while the rest of Temoto pauses.
 * 
 *  @author andyz(at)utexas.edu
 */

#include "temoto/preplanned_sequences.h"

// Action server CB: launch a preplanned sequence, as selected by the incoming command.
// It's an action server to be interruptible.
/*
void initiate_sequence_(const temoto::PreplannedSequenceActionGoalConstPtr& goal, actionlib::SimpleActionServer<temoto::PreplannedSequenceAction>* as)
{
  //ROS_INFO_STREAM("[preplanned_sequences] Starting " << goal.sequence_name.data );

  // TO DO: Actually run whatever sequence here...

  as->setSucceeded();
}
*/

// CB: Listen for an ABORT, and set a flag when it's heard
void preplanned_sequence::abort_cb_(const std_msgs::String::ConstPtr& msg)
{
  if ( msg-> data.c_str() == low_level_cmds::ABORT)
    ROS_INFO_STREAM("[preplanned_sequences] ABORTING");
}


// Constructor
preplanned_sequence::preplanned_sequence()
  /*: sequence_server_(n_, "temoto/preplanned_sequence", boost::bind(&initiate_sequence_, _1, &sequence_server_), false)*/
{
  // Listen for abort commands
  abort_sub_ = n_.subscribe("temoto/abort", 1, &preplanned_sequence::abort_cb_, this);
}


/** MAIN */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigate_robot");
  preplanned_sequence sequence;  // An object to process incoming commands
  ros::AsyncSpinner spinner(2);  // 2 threads: execute the given task, and listen for ABORT
  spinner.start();

/*
  ros::NodeHandle nh;
  actionlib::SimpleActionServer<temoto::PreplannedSequenceAction> sequence_server_(nh, "temoto/preplanned_sequence", boost::bind(&initiate_sequence_, _1, &sequence_server_), false);
*/

  ros::waitForShutdown();
  return 0;
}
