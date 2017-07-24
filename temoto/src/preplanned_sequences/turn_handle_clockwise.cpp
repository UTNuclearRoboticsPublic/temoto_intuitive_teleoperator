
#include "temoto/preplanned_sequences/turn_handle_clockwise.h"

// Put the UR5 in force_mode and turn the handle 90 degrees
int turn_handle_clockwise::turn_handle_clockwise()
{
  ros::NodeHandle* nhPtr = &turn_handle_clockwise::nh; // To pass to other functions

  turn_handle_clockwise::enable_compliance();

  // Rotate the end effector, as specified in yaml file
  turn_handle_clockwise::rotate();

  return 0;
}

// Enable compliance
void turn_handle_clockwise::enable_compliance()
{
  // Put the robot in force mode
  // This will be compliant in all directions with a target wrench of 0
  std::vector<float> force_frame {0., 0., 0., 0., 0., 0.};  // A pose
  std::vector<int> selection_vector {1, 1, 1, 1, 1, 1};  // Compliant in all directions
  std::vector<int> target_wrench {0, 0, 0, 0, 0, 0};
  int type = 1;  // Force frame transform
  std::vector<float> limits {0.8, 0.8, 0.8, 1.571, 1.571, 1.571};  // Displacement limits

  turn_handle_clockwise::right.enable_force_mode_( force_frame, selection_vector, target_wrench, type, limits );
}

// Rotate as dictated by the yaml file
int turn_handle_clockwise::rotate()
{
  // Set up the MoveGroup
  moveit::planning_interface::MoveGroupInterface::Plan move_plan;
  turn_handle_clockwise::move_group.setPlannerId("RRTConnectkConfigDefault");
  turn_handle_clockwise::move_group.setMaxVelocityScalingFactor( 0.1 );
  turn_handle_clockwise::move_group.setGoalPositionTolerance(0.02);
  turn_handle_clockwise::move_group.setGoalOrientationTolerance(0.05);

  // Get the robot's current position
  geometry_msgs::PoseStamped pose;
  pose = turn_handle_clockwise::move_group.getCurrentPose();

  std::vector<double> rpy = turn_handle_clockwise::move_group.getCurrentRPY();

  // Get the desired deltas from a yaml file. Add to the current position.
  // The launch file saves the first delta as a list to /temoto_preplanned_motions/turn_handle_clockwise/delta1
  // e.g. [0, 0, 0, -1.57, 0, 0]
  std::vector<double> delta1;
  turn_handle_clockwise::nh.getParam("/temoto_preplanned_motions/turn_handle_clockwise/delta1", delta1);

  pose.pose.position.x += delta1.at(0);
  pose.pose.position.y += delta1.at(1);
  pose.pose.position.z += delta1.at(2);
  rpy.at(0) += delta1.at(3);
  rpy.at(1) += delta1.at(4);
  rpy.at(2) += delta1.at(5);
  // Convert this rpy to quaternion so we can send a target pose
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(rpy.at(0), rpy.at(1), rpy.at(2));
  pose.pose.orientation = q;

  // Move
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pose.pose);
  double fraction = turn_handle_clockwise::move_group.computeCartesianPath(waypoints,
                                             0.01,  // eef_step
                                             3.0,   // jump_threshold
                                             move_plan.trajectory_);
  if ( !turn_handle_clockwise::move_group.execute(move_plan) )
  {
    ROS_WARN("Motion failed. Exiting.");
    return 1;
  }

  return 0;
}
