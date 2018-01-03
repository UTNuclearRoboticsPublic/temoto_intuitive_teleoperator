// Copyright (c) 2015-2016, The University of Texas at Austin
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

/** @file human_frame_broadcaster.cpp
 * 
 *  @brief ROS server that broadcasts the transform to the frame the human input is interpreted in.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

// ROS includes
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

// temoto includes
#include "temoto/temoto_common.h"

namespace human_frame_broadcaster
{
  /** Latest recieved full system status published by start_teleop node. */
  temoto::Status latest_status;

  bool g_natural_perspective = true;		///< Is TRUE for natural interpretation of human input; FALSE for inverted perspective.
  bool g_navigation_control = false;		///< Is TRUE when human input is to be interpred as a navigation goal in base_link frame.
}
using namespace human_frame_broadcaster;

/** This function is executed when change_human2robot_tf service is called.
 *  It updates current_cmd_frame frame based on the client's request.
 *  @param req temoto::ChangeTf service request.
 *  @param res temoto::ChangeTf service response.
 *  @return always true.
 */
bool service_change_tf(	temoto::ChangeTf::Request  &req,
			temoto::ChangeTf::Request &res)
{
  // Get the value from the request
  g_natural_perspective = req.first_person_perspective;
  g_navigation_control = req.navigate;
  
  return true;
} // end service_change_tf

/** Update the status of the robot.
 *  We use this to get the current end effector pose.
 *  @param status A temoto status msg.
 *  @return void.
 */
void statusCallback(temoto::Status status)
{
  latest_status = status;
}

/** Main method. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_frame_broadcaster");
  ros::NodeHandle nh;
  ros::Rate r(10);
  
  // Human input frame.
  std::string human_frame;

  // Navigation control frame.
  std::string mobile_frame;
  
  // Get proper frame names from parameter servers
  ROS_INFO("[human_frame_broadcaster] Getting frame names from parameter server.");
  nh.param<std::string>("/temoto_frames/human_input", human_frame, "current_cmd_frame");
  ROS_INFO("[human_frame_broadcaster] Human frame is: %s", human_frame.c_str());
  nh.param<std::string>("/temoto_frames/mobile_base", mobile_frame, "base_link");
  ROS_INFO("[human_frame_broadcaster] Mobile base frame is: %s", mobile_frame.c_str());
  
  // Create a tranform broadcaster.
  static tf::TransformBroadcaster tf_broadcaster;

  // ROS subscriber on /temoto/status. Used to get the EE pose
  ros::Subscriber sub_status = nh.subscribe("temoto/status", 1, statusCallback);
  
  // Advertise a service for switching between tranfrom rotations.
  ros::ServiceServer service = nh.advertiseService("temoto/change_human2robot_tf", service_change_tf);
  ROS_INFO("[human_frame_broadcaster] Service 'temoto/change_human2robot_tf' up and going.");
  
  // A tranform between current_cmd_frame (child) frame and parent robot frame. 
  tf::Transform hand_frame_to_robot;

  while ( ros::ok() )
  {
    // IF human input is to be used for navigating robot base
    if ( g_navigation_control )
    {
      hand_frame_to_robot.setOrigin( tf::Vector3(0, 0, 0) );
      // in navigation mode, current_cmd_frame rotated RPY=(90, 0, -90) in base_link
      hand_frame_to_robot.setRotation( tf::Quaternion(0.5, -0.5, -0.5, 0.5) );// set current_cmd_frame about base_link
      
      // Broadcast a transform that attaches current_cmd_frame to base_link using the hand_frame_to_robot transform.
      tf_broadcaster.sendTransform( tf::StampedTransform(hand_frame_to_robot, ros::Time::now(), mobile_frame, human_frame) );
    }
    // ELSE: human input is used for moving end effector
    else
    {
      // Set appropriate rotation for how current_cmd_frame data is interpreted.
      if ( g_natural_perspective )						// "natural" means robot arm is direct extension of human hand
      {
        // in manipulation/natural control mode, current_cmd_frame is rotated RPY=(90, 0, -90) about base_link
        hand_frame_to_robot.setRotation( tf::Quaternion(0.5, -0.5, -0.5, 0.5) );// set current_cmd_frame about base_link
      }
      else // not "natural" means human is facing the robot, i.e. left and right are inverted.
      {
      	// in manipulation/inverted control mode, current_cmd_frame is rotated RPY=(90, 0, 90) about base_link
        hand_frame_to_robot.setRotation( tf::Quaternion(0.5, 0.5, 0.5, 0.5) );  // set current_cmd_frame about base_link
      }

      // The origin shifts to the end effector
      hand_frame_to_robot.setOrigin( tf::Vector3(latest_status.end_effector_pose.pose.position.x, latest_status.end_effector_pose.pose.position.y, latest_status.end_effector_pose.pose.position.z) );

      // Broadcast the new transform
      tf_broadcaster.sendTransform( tf::StampedTransform(hand_frame_to_robot, ros::Time::now(), "base_link", human_frame) );
    }

    ros::spinOnce();
    r.sleep();
  }
  
  return 1;
}
