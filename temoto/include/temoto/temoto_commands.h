#ifndef TEMOTO_COMMANDS_H
#define TEMOTO_COMMANDS_H

// Several cpp/header files use this list of commands

namespace temoto_commands
{
	const std::string ABORT = "abort";
	const std::string GO = "go";
	const std::string EXECUTE = "execute";
	const std::string PLAN = "plan";

  // Button map for SpaceNavigator controllers
  const std::map<int, std::string> spacenav_buttons = {
    { 0, "robot please plan" },     // Menu button
    { 1, "robot please execute" },  // Fit button
    { 12, "jog mode" },             // 1 button
    { 13, "point to point mode" },  // 2 button
    { 14, "navigation" },           // 3 button
    { 15, "manipulation" },         // 4 button
    { 5, "open gripper" },          // F button
    { 4, "close gripper" },         // R button
    { 22, "base move" },            // Esc button
    { 23, "cycle camera feed" },    // Alt button
    { 24, "toggle compliance" },     // Shift button
    { 26, "next end effector" },    // Rotation button
  };
}

#endif
