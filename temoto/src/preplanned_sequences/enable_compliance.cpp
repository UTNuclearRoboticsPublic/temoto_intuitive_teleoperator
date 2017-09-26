
#include "temoto/preplanned_sequences/enable_compliance.h"


// Enable compliance
int enable_compliance::enable_compliance()
{
  // Put the robot in force mode
  // This will be compliant in all directions with a target wrench of 0
  std::vector<float> force_frame {0., 0., 0., 0., 0., 0.};  // A pose
  std::vector<int> selection_vector {1, 1, 1, 1, 1, 1};  // Compliant in all directions
  std::vector<int> target_wrench {0, 0, 0, 0, 0, 0};
  int type = 1;  // Force frame transform
  std::vector<float> limits {0.8, 0.8, 0.8, 1.571, 1.571, 1.571};  // Displacement limits

  ur_script_interface right_arm("/right_ur5_controller/right_ur5_URScript");
  right_arm.enable_force_mode_( force_frame, selection_vector, target_wrench, type, limits );

  return 0;
}
