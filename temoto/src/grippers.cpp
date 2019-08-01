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
#include "temoto/gripper_base_class.h"

namespace grippers
{
Grippers::Grippers(std::vector<std::string>& gripper_topics)
{
  // Create a publisher for each gripper
  for (std::string topic : gripper_topics)
  {
    ros::Publisher gripper_pub = nh_.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>(topic, 1);
    gripper_publishers_.push_back(std::make_shared<ros::Publisher>(gripper_pub));
  }
}

void Grippers::close(std::string& gripper_topic)
{
  void* handle = dlopen("./myclass.so", RTLD_LAZY);

  MyClass* (*create)();
  void (*destroy)(MyClass*);

  create = (MyClass* (*)())dlsym(handle, "create_object");
  destroy = (void (*)(MyClass*))dlsym(handle, "destroy_object");

  MyClass* myClass = (MyClass*)create();
  myClass->DoSomething();
  destroy( myClass );


  // Find the publisher on this topic
  for (auto pub : gripper_publishers_)
  {
    if (pub->getTopic() == gripper_topic)
    {
      robotiq_2f_gripper_control::Robotiq2FGripper_robot_output gripper_msg;
      gripper_msg.rPR = 255;
      gripper_msg.rACT = 1;
      gripper_msg.rGTO = 1;
      gripper_msg.rATR = 0;
      gripper_msg.rSP = 255;
      gripper_msg.rFR = 150;

      pub->publish(gripper_msg);
      break;
    }
  }
}

void Grippers::open(std::string& gripper_topic)
{
  // Find the publisher on this topic
  for (auto pub : gripper_publishers_)
  {
    if (pub->getTopic() == gripper_topic)
    {
      robotiq_2f_gripper_control::Robotiq2FGripper_robot_output gripper_msg;
      gripper_msg.rPR = 0;
      gripper_msg.rACT = 1;
      gripper_msg.rGTO = 1;
      gripper_msg.rATR = 0;
      gripper_msg.rSP = 255;
      gripper_msg.rFR = 150;

      pub->publish(gripper_msg);
      break;
    }
  }
}

}  // namespace grippers