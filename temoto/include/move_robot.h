#ifndef MOVE_ROBOT_H
#define MOVE_ROBOT_H

class MoveRobotInterface {
 public:
   MoveRobotInterface(std::string mg_name) :
    movegroup_(mg_name)			// I use something called initializer list here, Alex taught me that
   {
     use_named_target_ = false;
     new_plan_available_ = false;
     req_action_type_ = 0xff;
     new_move_requested_ = false;
     new_end_effector_pose_ = true;
   };
   
   moveit::planning_interface::MoveGroup movegroup_;
   
   /** Callback function */
   bool serviceUpdate(temoto::Goal::Request  &req, temoto::Goal::Response &res);

   void requestMove();

   void requestCartesianMove();

   geometry_msgs::PoseStamped target_pose_stamped_;		///< Target pose for the robot.
   std::string named_target_;					///< Named target for the robot.
   moveit::planning_interface::MoveGroup::Plan latest_plan_;	///< Latest motion plan.
   uint8_t req_action_type_;					///< Action type associated with target request, i.e. PLAN (0x01), EXECUTE PLAN (0x02), or PLAN&EXECUTE (0x03).
   
   // Public variables describing the state of MoveRobotInterface
   bool use_named_target_;		///< When named target is requested, use_named_target is set to true.
   bool new_plan_available_;		///< After calculating a new motion plan, is_new_plan is set to 1; after executing the plan, is_new_plan is set to 0.
   bool new_move_requested_; 		///< If new move has been requested by a client, it is set to 1; after calling move(), it is set to 0.
   bool new_end_effector_pose_;		///< If end effector has a new position, this is set to 1; after requesting rviz camera move, it is 0.
};

#endif