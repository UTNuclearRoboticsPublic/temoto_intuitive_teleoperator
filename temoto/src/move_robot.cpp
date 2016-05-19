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

#include "tf/tf.h"
#include "tf/transform_listener.h"

int8_t new_move_req = 0; 			///< If new move has been requested by a client, it is set to 1; after calling move(), it is set to 0.
int8_t new_end_effector_pose = 0;		///< If end effector has a new position, this is set to 1; after requesting rviz camera move, it is 0.
geometry_msgs::PoseStamped target_pose_stamped;	///< Target pose for the robot.
std::string req_named_target;			///< Named target for the robot.
uint8_t req_action_type = 0xff;			///< Action type associated with target request, i.e. PLAN (0x01), EXECUTE PLAN (0x02), or PLAN&EXECUTE (0x03).
bool use_named_target = false;			///< When named target is requested, use_named_target is set to true.
moveit::planning_interface::MoveGroup::Plan latest_plan;	///< Latest motion plan.
int8_t is_new_plan = 0;				///< After calculating a new motion plan, is_new_plan is set to 1; after executing the plan, is_new_plan is set to 0.
std_srvs::Empty empty_srv;			///< Empty service.

/** This method is executed when temoto/move_robot_service service is called.
 *  It updates target_pose_stamped based on the client's request and sets the new_move_req flag to let main() know that moving of the robot has been requested.
 *  @param req temoto::Goal service request.
 *  @param res temoto::Goal service response.
 *  @return always true.
 */
bool serviceUpdate(temoto::Goal::Request  &req,
		   temoto::Goal::Response &res) {
//  ROS_INFO("New service update requested.");
  
  // sets the action associated to target pose
  req_action_type = req.action_type;
  
  // sets target requested pose
  target_pose_stamped = req.goal;
  
  // check for named target
  use_named_target = false;					// by default do not use named_target.
  req_named_target = req.named_target;				// get named_target from service request.
  ROS_INFO("[robot_move/serviceUpdate] req_named_target='%s'", req_named_target.c_str());
  if (!req_named_target.empty()) use_named_target = true;	// if named_target was specified, set use_named_target to true.
  if (use_named_target) ROS_INFO("[robot_move/serviceUpdate] use_named_target flag has been set to true");
  
  // Set new_move_req to 1 for main() to see
  new_move_req = 1;
  ROS_INFO("[robot_move/serviceUpdate] new_move_req flag has been set to 1.");
  
  return true;
} // end serviceUpdate

/** Plans and/or executes the motion of the robot.
 *  @param robot MoveGroup object of the robot.
 */
void requestMove(moveit::planning_interface::MoveGroup& robot) {

  // Set current state as the start state for planner. For some reason the actual built-in function doesn't do that.
  robot.setStartState( *(robot.getCurrentState()) );
  
  // Get and print current position of the end effector
  geometry_msgs::PoseStamped current_pose = robot.getCurrentPose();
  ROS_INFO("[robot_move/requestMove] === CURRENT POSE ( as given by MoveGroup::getCurrentPose() ) ===");
  ROS_INFO("[robot_move/requestMove] Current pose frame: %s", current_pose.header.frame_id.c_str());
  ROS_INFO("[robot_move/requestMove] Current pose (posit x, y, z): (%f, %f, %f)", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  ROS_INFO("[robot_move/requestMove] Current pose (orien x, y, z, w): (%f, %f, %f, %f)", current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
 
  // Use either named or pose target (named target takes the priority over regular pose target).
  if (use_named_target) {						// if use_named_target is true
    bool set_target_ok = robot.setNamedTarget(req_named_target);	// set target pose as a named target
    if (!set_target_ok) {						// check if setting named target was successful
      ROS_INFO("[robot_move/requestMove] Failed to set named target. Please retry.");
      return;								// return if set target failed
    } else ROS_INFO("[robot_move/requestMove] Using NAMED TARGET for planning and/or moving.");
  } else {
//     robot.setEndEffectorLink("leap_motion_on_robot");
    ROS_INFO("[robot_move/requestMove] Found end effector link: %s", robot.getEndEffectorLink().c_str());
    // use the stamped target pose to set the target pose for robot
    bool set_target_ok = robot.setPoseTarget(target_pose_stamped/*, "leap_motion_inv"*/);
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
  geometry_msgs::PoseStamped current_target = robot.getPoseTarget();
  ROS_INFO("[robot_move/requestMove] Target pose frame: %s", current_target.header.frame_id.c_str());
  ROS_INFO("[robot_move/requestMove] Target pose (posit x, y, z): (%f, %f, %f)", current_target.pose.position.x, current_target.pose.position.y, current_target.pose.position.z);
  ROS_INFO("[robot_move/requestMove] Target pose (orien x, y, z, w): (%f, %f, %f, %f)", current_target.pose.orientation.x, current_target.pose.orientation.y, current_target.pose.orientation.z, current_target.pose.orientation.w);

  // Based on action type: PLAN (0x01), EXECUTE PLAN (0x02), or PLAN&EXECUTE (0x03)
  if (req_action_type == 0x01) {
    ROS_INFO("[robot_move/requestMove] Starting to plan ...");
    ROS_INFO("[robot_move/requestMove] Planning frame: %s", robot.getPlanningFrame().c_str());
    robot.plan(latest_plan);					// Calculate plan and store it in latest_plan.
    is_new_plan = 1;						// Set is_new_plan to 1.
    ROS_INFO("[robot_move/requestMove] DONE planning.");
  } else if (req_action_type == 0x02) {
    ROS_INFO("[robot_move/requestMove] Starting to execute last plan ...");
    if (is_new_plan) robot.execute(latest_plan);		// If there is a new plan, execute latest_plan.
    else ROS_INFO("[robot_move/requestMove] No plan to execute.");
    is_new_plan = 0;						// Set is_new_plan to 0.
    ROS_INFO("[robot_move/requestMove] DONE executing the plan");
  } else if (req_action_type == 0x03) {
    ROS_INFO("[robot_move/requestMove] Starting to move (i.e. plan & execute) ...");
//     robot.move();						// Plan and execute.
    // Since move() has a bug of start state not being current state, I am going to plan and execute sequentally.
    moveit::planning_interface::MoveGroup::Plan move_plan;
    printf("[robot_move/requestMove] Planning ...");
    robot.plan(move_plan);
    printf("[DONE] \n[robot_move/requestMove] and Executing ...\n");
    robot.execute(move_plan);
    ROS_INFO("[robot_move/requestMove] DONE moving.");
  }

  return;
} // end requestMove

/** Plans and executes the motion of the robot as a cartesian move.
 *  @param robot MoveGroup object of the robot.
 */
void requestCartesianMove(moveit::planning_interface::MoveGroup& robot) {
 
  // waypoints are interpreted in the "leap_motion" ref.frame
  robot.setPoseReferenceFrame("leap_motion");
  
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
  double fraction_of_plan = robot.computeCartesianPath(waypoints,
                                             0.01,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory);
  ROS_INFO("Cartesian path %.2f%% acheived", fraction_of_plan * 100.0);
  
  latest_plan.trajectory_ = trajectory;
  is_new_plan = 1;
  
  // TODO
  return;
}

// TODO This really should not be here, collision objects should be defined/added in a xacro file.
/** Creates a PlanningScene message with basic predetermined collision objects in the robot's workspace.
 *  @param robot MoveGroup object of the robot.
 *  @return moveit_msgs::PlanningScene message.
 */
moveit_msgs::PlanningScene defineBasicCollisionObjects(moveit::planning_interface::MoveGroup& robot) {

  // PlanningScene message that will contain added collision objects
  moveit_msgs::PlanningScene planning_scene;
  
  //\\ ____ BASE FLOOR ____ //\\
//   // First, we will define the collision object message for base floor.
//   moveit_msgs::CollisionObject base_floor;
//   base_floor.header.frame_id = "base_link"; //robot.getPlanningFrame();
// 
//   // The id of the object is used to identify it.
//   base_floor.id = "base_floor";
// 
//   // Define the base floor as a primitive box.
//   shape_msgs::SolidPrimitive primitive_floor;
//   primitive_floor.type = primitive_floor.BOX;
//   primitive_floor.dimensions.resize(3);
//   primitive_floor.dimensions[0] = 2;
//   primitive_floor.dimensions[1] = 2;
//   primitive_floor.dimensions[2] = 0.2;
// 
//   // A pose for the box-type base floor (specified relative to frame_id)
//   geometry_msgs::Pose floor_pose;
//   floor_pose.orientation.w = 1.0;
//   floor_pose.position.x = 0;
//   floor_pose.position.y = 0;
//   floor_pose.position.z = -0.1;
// 
//   // Add primitive floor and its pose to the base_floor collision object
//   base_floor.primitives.push_back(primitive_floor);
//   base_floor.primitive_poses.push_back(floor_pose);
//   base_floor.operation = base_floor.ADD;
  
  //\\ ____ SIDE WALL ____ //\\
  // Let's define the collision object message for side wall.
  moveit_msgs::CollisionObject side_wall;
  side_wall.header.frame_id = "base_link"; //robot.getPlanningFrame();

  // The id of the object is used to identify it.
  side_wall.id = "side_wall";

  // Define the base floor as a primitive box.
  shape_msgs::SolidPrimitive primitive_wall;
  primitive_wall.type = primitive_wall.BOX;
  primitive_wall.dimensions.resize(3);
  primitive_wall.dimensions[0] = 2;
  primitive_wall.dimensions[1] = 0.2;
  primitive_wall.dimensions[2] = 1;

  // A pose for the box-type base floor (specified relative to frame_id)
  geometry_msgs::Pose wall_pose;
  wall_pose.orientation.w = 1.0;
  wall_pose.position.x = 0;
  wall_pose.position.y = 0.8;
  wall_pose.position.z = 0.5;

  // Add primitive floor and its pose to the base_floor collision object
  side_wall.primitives.push_back(primitive_wall);
  side_wall.primitive_poses.push_back(wall_pose);
  side_wall.operation = side_wall.ADD;
  
  // A color for the side wall
  moveit_msgs::ObjectColor wall_color;
  wall_color.id = side_wall.id;
  wall_color.color.a = 0.1;	// it's barely visible
  wall_color.color.r = 0;
  wall_color.color.g = 0.33;	// may it be dark green
  wall_color.color.b = 0;
  
  //\\ ____ CONE STAND ____ //\\
//   // Collision object message for possible work stand (cone shape)
//   moveit_msgs::CollisionObject cone_stand;
//   cone_stand.header.frame_id = "base_link"; //robot.getPlanningFrame();
// 
//   // The id of the object is used to identify it.
//   cone_stand.id = "cone_stand";
//   
//   // Define a primitive cone
//   shape_msgs::SolidPrimitive primitive_cone;
//   primitive_cone.type = primitive_cone.CONE;
//   primitive_cone.dimensions.resize(2);
//   primitive_cone.dimensions[0] = 0.3;	// height
//   primitive_cone.dimensions[1] = 0.06;	// radius
// 
//   // A pose for the cone (specified relative to frame_id)
//   geometry_msgs::Pose cone_pose;
//   cone_pose.orientation.w = 1.0;
//   cone_pose.position.x = 0.84;
//   cone_pose.position.y = 0;
//   cone_pose.position.z = 0.09;		// 0.15 would be ON TOP OF base_floor, but i submerge it 6 mm
// 
//   // Add primitive cone and its pose to the cone_stand collision object
//   cone_stand.primitives.push_back(primitive_cone);
//   cone_stand.primitive_poses.push_back(cone_pose);
//   cone_stand.operation = cone_stand.ADD;
//   
//   // A color for the cone
//   moveit_msgs::ObjectColor cone_color;
//   cone_color.id = cone_stand.id;
//   cone_color.color.a = 1;	// it's visible
//   cone_color.color.r = 0;
//   cone_color.color.g = 0;
//   cone_color.color.b = 1;	// it's blue
  
  // === PALM CAMERA =========================== \\
//   // Let's define the collision object message for palm camera.
//   moveit_msgs::CollisionObject palm_camera;
//   palm_camera.header.frame_id = "robotiq_palm"; //robot.getPlanningFrame();
// 
//   // The id of the object is used to identify it.
//   palm_camera.id = "palm_camera";
// 
//   // Define the base floor as a primitive box.
//   shape_msgs::SolidPrimitive primitive_camera;
//   primitive_camera.type = primitive_camera.BOX;
//   primitive_camera.dimensions.resize(3);
//   primitive_camera.dimensions[0] = 0.067;	// width
//   primitive_camera.dimensions[1] = 0.013;	// thickness
//   primitive_camera.dimensions[2] = 0.13;	// height
// 
//   // A pose for the box-type base floor (specified relative to frame_id)
//   geometry_msgs::Pose camera_pose;
//   camera_pose.orientation.w = 1.0;
//   camera_pose.position.x = 0;
//   camera_pose.position.y = 0.05 + primitive_camera.dimensions[1]/2;
//   camera_pose.position.z = 0;
// 
//   // Add primitive floor and its pose to the base_floor collision object
//   palm_camera.primitives.push_back(primitive_camera);
//   palm_camera.primitive_poses.push_back(camera_pose);
//   palm_camera.operation = palm_camera.ADD;
//   
//   // A color for the side wall
//   moveit_msgs::ObjectColor camera_color;
//   camera_color.id = palm_camera.id;
//   camera_color.color.a = 0.7;	// it's a bit see through
//   camera_color.color.r = 0;	// may it be black
//   camera_color.color.g = 0;
//   camera_color.color.b = 0;  
  

  //**\\ ____ PLANNING SCENE ____ //**\\
  //Now, let’s add the collision objects into the world
//   planning_scene.world.collision_objects.push_back(base_floor);
  planning_scene.world.collision_objects.push_back(side_wall);
//   planning_scene.world.collision_objects.push_back(cone_stand);
//   planning_scene.world.collision_objects.push_back(palm_camera);
  // Adding object color to planning sceneROS_INFO("[robot_move/main] Planning frame: %s", robot_group.getPlanningFrame().c_str());
  planning_scene.object_colors.push_back(wall_color);
//   planning_scene.object_colors.push_back(cone_color);
//   planning_scene.object_colors.push_back(camera_color);

  planning_scene.is_diff = true;	// must be
  return planning_scene;
  
} // defineBasicCollisionObjects

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
  
  // Create moveit MoveGroup for user-specified move_group
  moveit::planning_interface::MoveGroup robot_group(move_group_name);
  robot_group.setPlannerId("RRTConnectkConfigDefault"); 		// using RRTConnectkConfigDefault planner
//   robot_group.setGoalTolerance(0.01);				// default goal tolerance is 0.01 m
  ROS_INFO("[robot_move/main] Planning frame: %s", robot_group.getPlanningFrame().c_str());
  ROS_INFO("[robot_move/main] End effector link: %s", robot_group.getEndEffectorLink().c_str());
//   robot_group.setEndEffector("leap_motion");
//   ROS_INFO("[robot_move/main] End effector link: %s", robot_group.getEndEffectorLink().c_str());
  ROS_INFO("[robot_move/main] End effector: %s", robot_group.getEndEffector().c_str());
  ROS_INFO("[robot_move/main] Goal position tolerance is: %.6f", robot_group.getGoalPositionTolerance());
  ROS_INFO("[robot_move/main] Goal orientation tolerance is: %.6f", robot_group.getGoalOrientationTolerance());
  ROS_INFO("[robot_move/main] Goal joint tolerance is: %.6f", robot_group.getGoalJointTolerance());
  
  // Set up publisher for PlanningScene messages.
  ros::Publisher publisher_planning_scene = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);
  // Get PlanningScene message with basic collision objects and publish it. Adds base floor and cone stand to robot's workspace.
  moveit_msgs::PlanningScene planning_scene_msg = defineBasicCollisionObjects( robot_group );
  publisher_planning_scene.publish( planning_scene_msg );
  
//   // Stoopid wait for PlanningScene message to actually go through.
//   sleep(5); // THIS ANNOYS ME. IT SHOULD NOT BE THIS STOOPID. I TRYED TO DO THIS ELEGANTLY BUT FAILED. 
//   // Attaching palm_camera to the robot. The collision objects for palm_camera has to be published in PlanningScene message before.
//   std::string str_link = "robotiq_palm";			// palm_camera is attatched to robotiq_palm; create a string with the name of 'robotiq_palm'
//   std::vector<std::string> touch_links;				// Vector of links it is OK to be in-collision with.
//   touch_links.push_back(str_link);				// Add str_link to touch_links (definitely needed, otherwise planning won't work)
//   // Attach palm_camera collision_object to 'robotiq_palm' and ignore self-collisions with all the links in touch_links.
//   robot_group.attachObject("palm_camera", "robotiq_palm", touch_links);

  // Set up service for move_robot_service; if there's a service request, executes serviceUpdate() function
  ros::ServiceServer service = n.advertiseService("temoto/move_robot_service", serviceUpdate);
  ROS_INFO("[robot_move/main] Service 'temoto/move_robot_service' up and going. Ready to send move commands to %s.", move_group_name.c_str());
  
  // Set up publisher for the end effector location
  ros::Publisher pub_end_effector = n.advertise<geometry_msgs::PoseStamped>( "temoto/end_effector_pose", 1 );
  
  // ROS client for /temoto/adjust_rviz_camera
  ros::ServiceClient client_visual = n.serviceClient<std_srvs::Empty>("temoto/adjust_rviz_camera");
  
  while(ros::ok()){

    if (new_move_req == 1 && req_action_type < 0x04) {		// if there has been a service request
	requestMove(robot_group);	// plan and execute move using move_group
	new_move_req = 0;		// set request flag to zero
	new_end_effector_pose = 1;	// assumes that request move resulted in new pose for end effector and sets the corresponding flag
    } //end if
    if (new_move_req == 1 && req_action_type == 0x04) {		// if there has been a service request for cartesian move
	requestCartesianMove(robot_group);	// launch cartesian move
	new_move_req = 0;		// set request flag to zero
	new_end_effector_pose = 1;	// assumes that request move resulted in new pose for end effector and sets the corresponding flag
    } //end if

    
    // get and publish current end effector pose;
    pub_end_effector.publish( robot_group.getCurrentPose() );

    // If pose of the end effector has changed. Update camera position only if the end effector has moved since the last time camera was positioned.
    if (new_end_effector_pose) {
      client_visual.call(empty_srv);
      new_end_effector_pose = 0;	// set new_end_effector_pose to zero
    } // end if
    
  } // end while

  return 0;
} // end main