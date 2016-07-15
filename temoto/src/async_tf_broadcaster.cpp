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

// ROS includes
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

// temoto includes
#include "temoto_include.h"

bool leap_motion_frame_natural = true;			///< Is 'true' for natural interpretation of hand motion; 'false' for inverted interpretation.
bool leap_motion_navigate = true;			///< Is TRUE when hand motion is to be interpred as a navigation goal in base_link frame.
// geometry_msgs::TransformStamped hand_frame;		// For debug.

/** This method is executed when temoto/change_tf service is called.
 *  It updates leap_motion_frame_natural based on the client's request.
 *  @param req temoto::ChangeTf service request.
 *  @param res temoto::ChangeTf service response.
 *  @return always true.
 */
bool service_change_tf(	temoto::ChangeTf::Request  &req,
			temoto::ChangeTf::Response &res) {

  // Get the value from the request
  leap_motion_frame_natural = req.first_person_perspective;
  leap_motion_navigate = req.navigate;
  ROS_INFO("[async_tf_broadcaster] New service update requested. Now leap_motion_frame_natural = %d; leap_motion_navigate = %d", leap_motion_frame_natural, leap_motion_navigate);
  
  return true;
} // end service_change_tf

// // For debug.
// void hand_pose_as_frame (temoto::Status status) {
//   hand_frame.child_frame_id = "my_hand";
//   hand_frame.header = status.live_hand_pose.header;
//   hand_frame.header.stamp = ros::Time::now();
//   hand_frame.transform.translation.x = status.live_hand_pose.pose.position.x;
//   hand_frame.transform.translation.y = status.live_hand_pose.pose.position.y;
//   hand_frame.transform.translation.z = status.live_hand_pose.pose.position.z;
//   hand_frame.transform.rotation = status.live_hand_pose.pose.orientation;
// }


/** Main method. */
int main(int argc, char **argv) {

  ros::init(argc, argv, "async_tf_broadcaster");	// ROS init
  ros::NodeHandle nh;					// ROS node handle
//   ros::AsyncSpinner spinner(1);				// using async spinner
//   spinner.start();					// starting async spinner
  ros::Rate r(10); // 10 hz
  
  // Create a tranform broadcaster.
  static tf::TransformBroadcaster tf_broadcaster;
  
  // ROS subscriber on /temoto/status
//   ros::Subscriber sub_status = nh.subscribe("temoto/status", 1, hand_pose_as_frame);

  // Advertise a service for switchig between tranfrom rotations.
  ros::ServiceServer service = nh.advertiseService("temoto/change_tf", service_change_tf);
  ROS_INFO("[async_tf_broadcaster] Service 'temoto/change_tf' up and going.");
  
  // Create a tranform between leap_motion (child) and leap_motion_on_robot (parent). 
  tf::Transform tf_leap_motion_to_robot;

  while ( ros::ok() ) {
    
    // IF hand motion is to be used for navigating robot base
    if (leap_motion_navigate)
    {
      tf_leap_motion_to_robot.setOrigin(tf::Vector3(0, 0, 0));
      // in navigation mode, leap_motion rotated RPY=(90, 0, -90) in base_link
      tf_leap_motion_to_robot.setRotation(tf::Quaternion(0.5, -0.5, -0.5, 0.5));	// set leap_motion about base_link
      // Broadcast a transform that attaches leap_motion to leap_motion_on_robot using the tf_leap_motion_to_robot specified above.
      tf_broadcaster.sendTransform( tf::StampedTransform(tf_leap_motion_to_robot, ros::Time::now(), "base_link", "leap_motion") );
    }
    // ELSE: hand motion is used for moving end effector
    else
    {
      // Set appropriate rotation for how leap_motion data is interpreted.
      if (leap_motion_frame_natural) {						// "natural" means robot arm is direct extension of human hand
	// in natural mode leap_motion and leap_motion_on_robot are oriented exactly the same
	tf_leap_motion_to_robot.setRotation(tf::Quaternion(0, 0, 0, 1));	// identity quaternion, i.e., no rotation
      } else { 									// not "natural" means human is facing the robot, i.e. left and right are inverted.
	// in inverted mode leap_motion is rotated 180° around the y(UP)-axis of leap_motion_on_robot
	tf_leap_motion_to_robot.setRotation(tf::Quaternion(0, 1, 0, 0));	// 180 around y-axis
      }

      // Broadcast a transform that attaches leap_motion to leap_motion_on_robot using the tf_leap_motion_to_robot specified above.
      tf_broadcaster.sendTransform( tf::StampedTransform(tf_leap_motion_to_robot, ros::Time::now(), "leap_motion_on_robot", "leap_motion") );
    }
    // For debug.
//     sleep(1);
//     // Publish hand_frame
//     tf_broadcaster.sendTransform( hand_frame );
    ros::spinOnce();
    r.sleep();
  }
  
  return 1;
}