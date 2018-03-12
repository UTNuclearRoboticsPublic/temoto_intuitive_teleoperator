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
//       documentation and/or other materials provided with the distribution_.
// 
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission_.
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

/** @file interpret_utterance.h
 *  Subscribes to the verbal command topic and tries to extract valid voice commands
 *  from it. If valid voice command is extracted, an appropriate command code is published on
 *  "temoto/voice_commands".
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

// ROS includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sound_play/sound_play.h"

// temoto includes
#include "temoto/temoto_common.h"

// Other includes
#include "string.h"
#include <vector>


class Interpreter
{
public:
  // Constructor
  Interpreter()
  {
    ros::NodeHandle pn("~");  // NodeHandle for accessing private parameters
      
    // Get params from launch file
    pn.param<std::string>("temoto/input_voice_topic", input_voice_topic_, "stt/spoken_text");
  
    // Subscribe to speech recognizer
    sub_utterances_ = n_.subscribe<std_msgs::String>(input_voice_topic_, 5, &Interpreter::utteranceToRecognizedCommand, this);

    // Publish unambiguous commands based on speech recognition
    pub_voice_commands_ = n_.advertise<temoto::Command>("temoto/voice_commands", 2);

    // Output to the screen all the accepted voice commands
    displayRecognizedVoiceCommands();

    // wait for for the sound_client server to come up
    // TODO: change to something that actually checks if the server is online
    sleep(1);
    sound_client_.say("Hello! I am ready to receive verbal instructions.");
  };

  /** Maps verbal instructions to specific command code. */
  std::vector<std::string> command_list_ =
  {
    // Commands for basic Temoto functionality: moving and navigating
    {"robot please plan"},	// command PLAN
    {"robot please execute"},	// command EXECUTE plan
    {"robot plan and go"},		// command PLAN&EXECUTE
    {"manipulation"},		// operator controls robot manipulator (MoveIt!)
    {"navigation"},		// operator navigates the robot base (ROS_navigation)
    {"jog mode"},		// send small motions commands immediately
    {"stop jogging"},		// stop jogging

    // Preplanned sequences: these will interrupt other Temoto commands (except abort)
    // They can be generally any C++ file
    {"open gripper"},		// open gripper, a preplanned sequence
    {"close gripper"},   // close gripper, a preplanned sequence
  };

  ros::NodeHandle n_;
  
  /** Publisher for recognized voice commands. */
  ros::Publisher pub_voice_commands_;
  
  /** Subscriber for utterance strings. */
  ros::Subscriber sub_utterances_;
  
  /** Instance of SoundClient used for text-to-speech synthesis. */
  sound_play::SoundClient sound_client_;

  /** Store a parameter from the launch file. This is the topic where voice commands come in_. */
  std::string input_voice_topic_;
  
  /** Callback for subscribed utterances. */
  void utteranceToRecognizedCommand (std_msgs::String utterance);
  
  /** Prints out all the keys in command_list_. */
  void displayRecognizedVoiceCommands();
};
