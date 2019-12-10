#ifndef TEMOTO_COMMANDS_H
#define TEMOTO_COMMANDS_H

// Several cpp/header files use this list of commands

namespace temoto_commands
{
  const std::string ABORT = "abort";
  const std::string GO = "go";
  const std::string EXECUTE = "execute";
  const std::string PLAN = "plan";
  const std::string ARM_PLAN_HOME = "arm go home";

  // Button map for SpaceNavigator controllers
  const std::map<int, std::string> spacenav_buttons_ = {
    { 0, "robot please plan" },     // Menu button
    { 1, "robot please execute" },  // Fit button
    { 4, "close gripper" },         // R button
    { 5, "open gripper" },          // F button
    { 8, "bias compliance" },       // Square with arrow button
    { 12, "stop stop" },            // 1 button
    { 13, "unassigned" },           // 2 button
    { 14, "toggle control" },       // 3 button
    { 15, "toggle mode" },          // 4 button
    { 22, "base move" },            // Esc button
    { 23, "cycle camera feed" },    // Alt button
    { 24, "toggle compliance" },    // Shift button
    { 25, "arm plan home" },		    // Ctrl button
    { 26, "next end effector" }     // Rotation button
  };
  /*
  // Empty numbers do not have a physical button to my knowledge
      NUMBER        BUTTON DESCRIPTION
      0             Menu button
      1             Fit button
      2             T button
      3             
      4             R button
      5             F button
      6
      7
      8             Square with arrow button
      9
      10
      11
      12            1 button
      13            2 button
      14            3 button
      15            4 button
      16
      17
      18
      19
      20
      21
      22            Esc button
      23            Alt button
      24            Shift button
      25            Ctrl button
      26            Rotation button
  */
}

#endif

