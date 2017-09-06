#include "temoto/preplanned_sequences/robot_approach.h"

robot_approach::robot_approach()
{
  system("rosrun move_base_to_manip provide_target");

}
