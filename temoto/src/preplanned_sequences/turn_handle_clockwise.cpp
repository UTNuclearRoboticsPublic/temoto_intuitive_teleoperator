
#include "temoto/preplanned_sequences/turn_handle_clockwise.h"

// Put the UR5 in force_mode and turn the handle 90 degrees
int turn_handle_clockwise()
{
  // arm compliance object. Sends ur commands on this topic
  ur_script_compliance right("/right_ur5_controller/right_ur5_URScript");

  enable_compliance(right);

  // Rotate the end effector, as specified in yaml file

  return 0;
}

// Enable compliance
void enable_compliance(ur_script_compliance& right)
{
  // Put the robot in force mode
  // This will be compliant in all directions with a target wrench of 0
  std::vector<float> force_frame {0., 0., 0., 0., 0., 0.};  // A pose
  std::vector<int> selection_vector {1, 1, 1, 1, 1, 1};  // Compliant in all directions
  std::vector<int> target_wrench {0, 0, 0, 0, 0, 0};
  int type = 1;  // Force frame transform
  std::vector<float> limits {0.8, 0.8, 0.8, 1.571, 1.571, 1.571};  // Displacement limits

  right.enable_force_mode_( force_frame, selection_vector, target_wrench, type, limits );
}
