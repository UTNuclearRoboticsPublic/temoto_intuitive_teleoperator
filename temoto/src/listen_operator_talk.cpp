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

/** @file listen_operator_talk.cpp
 *  Subscribes to "pocketsphinx_recognizer/output" topic and tries to extract valid voice commands
 *  from itt. If valid voice command is extracted, an approproate command code is published on
 *  "temoto/voice_commands".
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "temoto/Command.h"
#include "string.h"
#include "sound_play/sound_play.h"

// TODO: can i implement some general ABORT to ROS, like ROS SHUTDOWN

#define NUMBER_OF_VALID_COMMANDS 21

/** List of valid voice commands strings. */
std::string valid_voice_commands[NUMBER_OF_VALID_COMMANDS] =
{
    "stop stop",		// stop or abort command
    "robot please plan",	// command PLAN
    "robot please execute",	// command EXECUTE plan
    "robot please go",		// command PLAN&EXECUTE
    "robot please plan home",	// command PLAN to a saved home pose
    "robot please go home",	// command PLAN&EXECUTE the home pose
    "natural control mode",	// control mode determines whether operator has natural or inverted view
    "inverted control mode",	// control mode determines whether operator has natural or inverted view
    "free directions",		// complete hand position is used
    "limit directions",		// some directions may be limited
    "consider rotation",	// interpretes hand orientation
    "ignore rotation",		// hand orientation is ignored, i.e. using position only
    "compute cartesian",	// computes cartesian path based on waypoints
    "execute cartesian",	// executes the cartesian path
    "cartesian go",		// computes and executes cartesian path
    "new",			// first point in waypoints
    "add",			// adds a point into waypoints
    "remove last",		// removes the last waypoint
    "cancel cartesian",		// clears all the cartesian waypoints
    "manipulation",		// operator controls robot manipulator (MoveIt!)
    "navigation"		// operator navigates the robot base (ROS_navigation)
};

/** List of valid voice commands strings correspond to "actual" valid commands for which unsigned integers are used. */
uint8_t valid_commands[NUMBER_OF_VALID_COMMANDS] = 
{
  0x00, 0x01, 0x02, 0x03, 0xf1, 0xf3,		// move related
  0x10, 0x11,					// control perspective related
  0x20, 0x21, 0x22, 0x23,			// constraints, related to which data from the hand to use
  0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,	// cartesian move related
  0x40, 0x41					// manipulatation, navigation
};

/** Whenever a valid voice command is detected, latest_voice_command.cmd is set to hold a corresponding valid command. */
temoto::Command latest_voice_command;
/** Whenever a valid voice command is detected, publish_valid_command is set true; after publishing latest_voice_command, publish_valid_command is set false. */
bool publish_valid_command = false;
bool no_valid_command = false;

/** Callback function for pocketsphinx_recognizer/output topic.
 *  It searches the pocketsphinx_recognizer output string for any of the strings specified in valid_voice_commands.
 *  @param last_phrase pocketsphinx_recognizer output string.
 */
void speechToVoiceCommands (std_msgs::String last_phrase) {
  // Set namespace for voice command messages
  latest_voice_command.ns = "temoto_voice_command";
  
  // Print the output of pocketsphinx_recognizer
  ROS_INFO("I think you just said: '%s'", last_phrase.data.c_str());
  
  // Position of the valid voice command string in the input string
  std::size_t found;
  // Match the input string to every element in valid_voice_commands 
  for (int i = 0; i < NUMBER_OF_VALID_COMMANDS; i++ ) {
    found = last_phrase.data.find(valid_voice_commands[i]);	// find a valid voice command in last_phrase
    if (found!=std::string::npos) {				// if valid voice command was found in last_phrase
      latest_voice_command.cmd = valid_commands[i];		// take the corresponding valid command
      publish_valid_command = true;				// set publish_valid_command true
//       ROS_INFO("Ready to publish valid voice command: %x", latest_voice_command.cmd);
//       return;							// return as there is no need to search for other commands in the string
    } // end if
  } // end for
  
  no_valid_command = !publish_valid_command;
  
  // Converts input strings to specific commands (e.g. PLAN, EXECUTE, ABORT) that will be published
  return;
}

/** Displays on the screen all the strings that are considered valid voice commands. */
void displayRecognizedVoiceCommands() {
  ROS_INFO("Here are all the recognized voice commands:");
  for (int i = 0; i < NUMBER_OF_VALID_COMMANDS; i++ ) {
    ROS_INFO(" == '%s'", valid_voice_commands[i].c_str());
  }
  return;
}

/** Main method. */
int main(int argc, char **argv) {

  ros::init(argc, argv, "listen_operator_talk");	// ROS init
  ros::NodeHandle n;					// ROS handle

  // Subscribe to pocketsphinx_recognizer speech-to-text output
  ros::Subscriber sub_pocketsphinx = n.subscribe<std_msgs::String>("pocketsphinx_recognizer/output", 5, speechToVoiceCommands);
  // Publish unambiguous commands based on speech recognition
  ros::Publisher pub_voicecommands = n.advertise<temoto::Command>("temoto/voice_commands", 2);
  
  sound_play::SoundClient sound_client;
  
  ROS_INFO("Listening the operator talk ...");
  ROS_INFO("... and remember that manners maketh man.");
  // Output to the screen all the accepted voice commands
  displayRecognizedVoiceCommands();
  
  while (ros::ok()) {
    if (publish_valid_command)					// if there is a new valid command to publish
    {
      sound_client.say(/*"okay"*/ "affirmative");		// give user auditory confirmation
      pub_voicecommands.publish( latest_voice_command );	// publish latest voice command
      publish_valid_command = false;
    }
    else if (no_valid_command)					// if something was said but failed to extract valid command
    {
      sound_client.say(/*"not okay"*/ "i beg your pardon");	// give user auditory confirmation
      no_valid_command = false;
    }
    ros::spinOnce();						// spins once to update subscribers or something like that
  } // end while
  
  return 0;
} // end main