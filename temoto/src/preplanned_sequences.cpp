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
 *  @brief While executing, it also listens for an "Abort" command.
 * 
 *  @author andyz(at)utexas.edu
 */

#include "temoto/preplanned_sequences.h"

// Action server CB: launch the preplanned sequence, as specified by the incoming command.
// Action server allows interruption.
void preplanned_sequence::execute_CB_(const temoto::PreplannedSequenceGoalConstPtr& goal)
{
  ROS_INFO_STREAM("[preplanned_sequences] Starting \"" << goal->sequence_name << "\"" );
  result_.success = false;

  if (goal->sequence_name == "close gripper")
    close_gripper::close_gripper(gripper_publisher_);
  else if (goal->sequence_name == "open gripper")
    open_gripper::open_gripper(gripper_publisher_);

  // Action server: signal that the sequence is done
  result_.success = true;
  sequence_server_.setSucceeded(result_);
}


// Topic CB: Listen for an ABORT, and set a flag when it's heard.
// This provides a method of halting preplanned sequences.
void preplanned_sequence::abort_cb_(const std_msgs::String::ConstPtr& msg)
{
  if ( msg-> data.c_str() == low_level_cmds::ABORT)
    ROS_INFO_STREAM("[preplanned_sequences] ABORTING");
}
