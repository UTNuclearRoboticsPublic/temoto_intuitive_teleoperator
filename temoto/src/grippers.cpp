// Copyright (c) 2018, The University of Texas at Austin
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
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/** @file grippers.cpp
 *
 *  @brief Send commands to a vector of grippers.
 *
 *  @author andyz(at)utexas.edu
 */

#include "temoto/grippers.h"

#include "dlfcn.h"

namespace grippers
{
Grippers::Grippers(std::string gripper_type, std::string gripper_topic)
{
  if (gripper_type == "")
  {
    // No gripper type was defined
    gripper_object_ = std::unique_ptr<GripperBaseClass>(new GripperBaseClass());
  }
  else if (gripper_type == "robotiq")
  {
    gripper_object_ = std::unique_ptr<GripperRobotiq>(new GripperRobotiq(gripper_topic));
  }
  else
  {
    ROS_ERROR_STREAM("This gripper type is not supported");
    std::exit(EXIT_FAILURE);
  }
  
}

bool Grippers::open()
{
  gripper_object_->open();

  return true;
}

bool Grippers::close()
{
  gripper_object_->close();

  return true;
}

}  // namespace grippers