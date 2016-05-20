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
#include "std_srvs/Empty.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit_msgs/PlanningScene.h"
#include "moveit_msgs/ObjectColor.h"

// temoto includes
#include "temoto_include.h"
#include "move_robot.h"

#include "tf/tf.h"
#include "tf/transform_listener.h"

// int8_t new_move_req = 0; 			///< If new move has been requested by a client, it is set to 1; after calling move(), it is set to 0.
// int8_t new_end_effector_pose = 0;		///< If end effector has a new position, this is set to 1; after requesting rviz camera move, it is 0.
// geometry_msgs::PoseStamped target_pose_stamped;	///< Target pose for the robot.
// std::string req_named_target;			///< Named target for the robot.
// uint8_t req_action_type = 0xff;			///< Action type associated with target request, i.e. PLAN (0x01), EXECUTE PLAN (0x02), or PLAN&EXECUTE (0x03).

// bool use_named_target = false;			///< When named target is requested, use_named_target is set to true.
// moveit::planning_interface::MoveGroup::Plan latest_plan;	///< Latest motion plan.
// int8_t is_new_plan = 0;				///< After calculating a new motion plan, is_new_plan is set to 1; after executing the plan, is_new_plan is set to 0.
// std_srvs::Empty empty_srv;			///< Empty service.

// class MoveRobotInterface {
//  public:
// 
//    moveit::planning_interface::MoveGroup robot_group;
//    bool serviceUpdate(temoto::Goal::Request  &req, temoto::Goal::Response &res);
//    void requestMove();
//    void requestCartesianMove();
//    MoveRobotInterface(std::string mg) :
//    robot_group(mg) {
//    };  
// };

/** This method is executed when temoto/move_robot_service service is called.
 *  It updates target_pose_stamped based on the client's request and sets the new_move_req flag to let main() know that moving of the robot has been requested.
 *  @param req temoto::Goal service request.
 *  @param res temoto::Goal service response.
 *  @return always true.
 */
bool MoveRobotInterface::serviceUpdate(temoto::Goal::Request  &req,
				       temoto::Goal::Response &res) {
//  ROS_INFO("New service update requested.");

  if (req.action_type == req.CARTESIAN_COMPUTE) {
    // waypoints are interpreted in the "leap_motion" ref.frame
    movegroup_.setPoseReferenceFrame("leap_motion");	//TODO Take the value from req
    // Computed Cartesian trajectory.
    moveit_msgs::RobotTrajectory trajectory;
    res.cartesian_fraction = movegroup_.computeCartesianPath(req.cartesian_wayposes,
                                             0.01,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory);
    ROS_INFO("[MoveRobotInterface::serviceUpdate] Cartesian path %.2f%% acheived", res.cartesian_fraction * 100.0);
  
    latest_plan_.trajectory_ = trajectory;
    new_plan_available_ = true;
    
  } else {
  
    // sets the action associated to target pose
    req_action_type_ = req.action_type;			// TODO SHOULD IT BE BEFORE THE IFs
    
    // sets target requested pose
    target_pose_stamped_ = req.goal;
    
    // check for named target
    use_named_target_ = false;					// by default do not use named_target.
    named_target_ = req.named_target;				// get named_target from service request.
    ROS_INFO("[MoveRobotInterface::serviceUpdate] named_target_='%s'", named_target_.c_str());
    if (!named_target_.empty()) use_named_target_ = true;		// if named_target was specified, set use_named_target_ to true.
    if (use_named_target_) ROS_INFO("[MoveRobotInterface::serviceUpdate] use_named_target was set TRUE");
    
    // Set new_move_req to 1 for main() to see
    new_move_requested_ = true;
    ROS_INFO("[MoveRobotInterface::serviceUpdate] new_move_requested_ was set TRUE.");
  
    
  }
  return true;
} // end serviceUpdate

/** Plans and/or executes the motion of the robot.
 *  @param robot MoveGroup object of the robot.
 */
void MoveRobotInterface::requestMove(/*moveit::planning_interface::MoveGroup& robot*/) {

  // Set current state as the start state for planner. For some reason the actual built-in function doesn't do that.
  movegroup_.setStartState( *(movegroup_.getCurrentState()) );
  
  // Get and print current position of the end effector
  geometry_msgs::PoseStamped current_pose = movegroup_.getCurrentPose();
  ROS_INFO("[robot_move/requestMove] === CURRENT POSE ( as given by MoveGroup::getCurrentPose() ) ===");
  ROS_INFO("[robot_move/requestMove] Current pose frame: %s", current_pose.header.frame_id.c_str());
  ROS_INFO("[robot_move/requestMove] Current pose (posit x, y, z): (%f, %f, %f)", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  ROS_INFO("[robot_move/requestMove] Current pose (orien x, y, z, w): (%f, %f, %f, %f)", current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
 
  // Use either named or pose target (named target takes the priority over regular pose target).
  if (use_named_target_) {						// if use_named_target is true
    bool set_target_ok = movegroup_.setNamedTarget(named_target_);	// set target pose as a named target
    if (!set_target_ok) {						// check if setting named target was successful
      ROS_INFO("[robot_move/requestMove] Failed to set named target. Please retry.");
      return;								// return if set target failed
    } else ROS_INFO("[robot_move/requestMove] Using NAMED TARGET for planning and/or moving.");
  } else {
//     robot.setEndEffectorLink("leap_motion_on_robot");
    ROS_INFO("[robot_move/requestMove] Found end effector link: %s", movegroup_.getEndEffectorLink().c_str());
    // use the stamped target pose to set the target pose for robot
    bool set_target_ok = movegroup_.setPoseTarget(target_pose_stamped_/*, "leap_motion_inv"*/);
    if (!set_target_ok) {						// check if set target pose failed
      ROS_INFO("[robot_move/requestMove] Failed to set pose target. Please retry.");
      return;								// return if set target failed
    } else ROS_INFO("[robot_move/requestMove] Using POSE TARGET for planning and/or moving.");
  }

  // TODO
//   // Set goal tolerance based on the actual shift from current positio to target position
//   std::vector <geometry_msgs::Point> current_and_target;		// Vector that contains current and target points
//   current_and_target.push_back(current_pose.pose.position);		// Add current position
//   current_and_target.push_back(target_pose.position);			// Add target position
//   double shift = calculateDistance(current_and_target);			// Calculate the linear distance between the two positions
//   // CALCULATE DISTNACE BETWEEN CURRENT AND TARGET
//   // CALCULATE TOLERANCE AS A PERCENTAGE OF DISTANCE && CONSIDER SOME MIN/MAX LIMITS
//   // SET TOLERANCE FOR PLANNING
  
  // Just checking what is the target pose
  geometry_msgs::PoseStamped current_target = movegroup_.getPoseTarget();
  ROS_INFO("[robot_move/requestMove] Target pose frame: %s", current_target.header.frame_id.c_str());
  ROS_INFO("[robot_move/requestMove] Target pose (posit x, y, z): (%f, %f, %f)", current_target.pose.position.x, current_target.pose.position.y, current_target.pose.position.z);
  ROS_INFO("[robot_move/requestMove] Target pose (orien x, y, z, w): (%f, %f, %f, %f)", current_target.pose.orientation.x, current_target.pose.orientation.y, current_target.pose.orientation.z, current_target.pose.orientation.w);

  // Based on action type: PLAN (0x01), EXECUTE PLAN (0x02), or PLAN&EXECUTE (0x03)
  if ( req_action_type_ == 0x01 ) {
    ROS_INFO("[robot_move/requestMove] Starting to plan ...");
    ROS_INFO("[robot_move/requestMove] Planning frame: %s", movegroup_.getPlanningFrame().c_str());
    movegroup_.plan( latest_plan_ );				// Calculate plan and store it in latest_plan_.
    new_plan_available_ = true;					// Set new_plan_available_ to TRUE.
    ROS_INFO("[robot_move/requestMove] DONE planning.");
  } else if ( req_action_type_ == 0x02 ) {
    ROS_INFO("[robot_move/requestMove] Starting to execute last plan ...");
    if ( new_plan_available_ ) movegroup_.execute( latest_plan_ );	// If there is a new plan, execute latest_plan_.
    else ROS_INFO("[robot_move/requestMove] No plan to execute.");	// Else do nothing but printout "no plan"
    new_plan_available_ = false;					// Either case, set new_plan_available_ to FALSE.
    ROS_INFO("[robot_move/requestMove] DONE executing the plan");
  } else if ( req_action_type_ == 0x03 ) {
    ROS_INFO("[robot_move/requestMove] Starting to move (i.e. plan & execute) ...");
//     robot.move();						// Plan and execute.
    // Since move() has a bug of start state not being current state, I am going to plan and execute sequentally.
    moveit::planning_interface::MoveGroup::Plan move_plan;
    printf("[robot_move/requestMove] Planning ...");
    movegroup_.plan( move_plan );
    printf("[DONE] \n[robot_move/requestMove] and Executing ...\n");
    movegroup_.execute( move_plan );
    ROS_INFO("[robot_move/requestMove] DONE moving.");
    new_plan_available_ = false;					// As any previous plan has become invalid, set new_plan_available_ to FALSE.
  }

  return;
} // end requestMove

/** Plans and executes the motion of the robot as a cartesian move.
 *  @param robot MoveGroup object of the robot.
 */
void MoveRobotInterface::requestCartesianMove(/*moveit::planning_interface::MoveGroup& robot*/) {
 
  // waypoints are interpreted in the "leap_motion" ref.frame
  movegroup_.setPoseReferenceFrame("leap_motion");
  
  // The full plan for Cartesian move.
  moveit::planning_interface::MoveGroup::Plan cartesian_plan;
//   cartesian_plan.start_state_ = *(robot.getCurrentState());
  
  // Waypoints for Cartesian move.
  std::vector<geometry_msgs::Pose> waypoints;
  /// ONLY FOR TESTING I SPECIFY A BUNCH OF WAYPOINTS HERE
  geometry_msgs::Pose wp1, wp2, wp3, wp4, wp5;
  wp2.orientation.w = 1;
  wp2.position.y = -0.14;	//downwards
  waypoints.push_back(wp2);
  wp3.orientation.w = 1;
  wp3.position.x = -0.14;	//to the left
  waypoints.push_back(wp3);
  wp4.orientation.w = 1;
  wp4.position.x = 0.14;	//to the right
  waypoints.push_back(wp4);
  wp5.orientation.w = 1;
  wp5.position.y = 0.14;	//upwards
  waypoints.push_back(wp5);
  wp1.orientation.w = 1;
  wp1.position.x = 0;		//back to the start
  waypoints.push_back(wp1);
  
//   robot.getCurrentPose()
  
  
  // Computed Cartesian trajectory.
  moveit_msgs::RobotTrajectory trajectory;
  double fraction_of_plan = movegroup_.computeCartesianPath(waypoints,
                                             0.01,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory);
  ROS_INFO("Cartesian path %.2f%% acheived", fraction_of_plan * 100.0);
  
  latest_plan_.trajectory_ = trajectory;
  new_plan_available_ = true;
  
  // TODO
  return;
}



/** Main method. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "move_robot");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  // user-specified move_group_name
  std::string move_group_name;
  if (argc > 1) move_group_name = argv[1];
  else {
    ROS_INFO("Usage: move_robot <move_group_name>");
    return -1;
  }
  
  // Create MoveRobotInterface for user-specified move_group
  MoveRobotInterface moveIF(move_group_name);
  moveIF.movegroup_.setPlannerId("RRTConnectkConfigDefault"); 		// using RRTConnectkConfigDefault planner
//   robot_group.setGoalTolerance(0.01);				// default goal tolerance is 0.01 m
  ROS_INFO("[robot_move/main] Planning frame: %s", moveIF.movegroup_.getPlanningFrame().c_str());
  ROS_INFO("[robot_move/main] End effector link: %s", moveIF.movegroup_.getEndEffectorLink().c_str());
//   robot_group.setEndEffector("leap_motion");
//   ROS_INFO("[robot_move/main] End effector link: %s", robot_group.getEndEffectorLink().c_str());
  ROS_INFO("[robot_move/main] End effector: %s", moveIF.movegroup_.getEndEffector().c_str());
  ROS_INFO("[robot_move/main] Goal position tolerance is: %.6f", moveIF.movegroup_.getGoalPositionTolerance());
  ROS_INFO("[robot_move/main] Goal orientation tolerance is: %.6f", moveIF.movegroup_.getGoalOrientationTolerance());
  ROS_INFO("[robot_move/main] Goal joint tolerance is: %.6f", moveIF.movegroup_.getGoalJointTolerance());
  
  // Set up service for move_robot_service; if there's a service request, executes serviceUpdate() function
  ros::ServiceServer service = n.advertiseService("temoto/move_robot_service", &MoveRobotInterface::serviceUpdate, &moveIF);
  ROS_INFO("[robot_move/main] Service 'temoto/move_robot_service' up and going. Ready to send move commands to %s.", move_group_name.c_str());
  
  // Set up publisher for the end effector location
  ros::Publisher pub_end_effector = n.advertise<geometry_msgs::PoseStamped>( "temoto/end_effector_pose", 1 );
  
  // ROS client for /temoto/adjust_rviz_camera
  ros::ServiceClient client_visual = n.serviceClient<std_srvs::Empty>("temoto/adjust_rviz_camera");
  std_srvs::Empty empty_srv;			// Empty service.
  
  while(ros::ok()){

    if (moveIF.new_move_requested_ && moveIF.req_action_type_ < 0x04) {	// if there has been a service request for new move
	moveIF.requestMove(/*robot_group*/);	// plan and execute move using move_group
	moveIF.new_move_requested_ = false;	// set request flag to false
	moveIF.new_end_effector_pose_ = true;	// assumes that request move resulted in new pose for end effector and sets the corresponding flag
    } //end if
    if (moveIF.new_move_requested_ && moveIF.req_action_type_ == 0x04) {	// if there has been a service request for cartesian move
	moveIF.requestCartesianMove(/*robot_group*/);	// launch cartesian move
	moveIF.new_move_requested_ = false;	// set request flag to false
	moveIF.new_end_effector_pose_ = true;	// assumes that request move resulted in new pose for end effector and sets the corresponding flag
    } //end if

    
    // get and publish current end effector pose;
    pub_end_effector.publish( moveIF.movegroup_.getCurrentPose() );

    // If pose of the end effector has changed. Update camera position only if the end effector has moved since the last time camera was positioned.
    if (moveIF.new_end_effector_pose_) {
      client_visual.call(empty_srv);
      moveIF.new_end_effector_pose_ = false;	// set new_end_effector_pose to zero
    } // end if
    
  } // end while

  return 0;
} // end main