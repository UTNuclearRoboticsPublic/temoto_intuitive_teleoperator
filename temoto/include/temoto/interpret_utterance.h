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

/** @file interpret_utterance.h
 *  Subscribes to "pocketsphinx_recognizer/output" topic and tries to extract valid voice commands
 *  from it. If valid voice command is extracted, an approproate command code is published on
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
  /** Constructor */
  Interpreter()
  {
  };
  
  /** Maps verbal instructions to specific command code. */
  std::vector<std::string> command_list_ =
  {
    {"stop stop"},		// stop or abort command
    {"robot please plan"},	// command PLAN
    {"robot please execute"},	// command EXECUTE plan
    {"robot plan and go"},		// command PLAN&EXECUTE
    {"robot plan home"},	// command PLAN to a saved home pose
    {"robot please go home"},	// command PLAN&EXECUTE the home pose
    {"natural control mode"},	// control mode determines whether operator has natural or inverted view
    {"inverted control mode"},	// control mode determines whether operator has natural or inverted view
    {"free directions"},		// complete position of hand is used
    {"limit directions"},		// some directions may be limited
    {"consider rotation"},	// factors in hand orientation
    {"ignore rotation"},		// hand orientation is ignored, i.e. using hand position only
    {"manipulation"},		// operator controls robot manipulator (MoveIt!)
    {"navigation"},		// operator navigates the robot base (ROS_navigation)
    {"turn handle clockwise"},	// a preplanned sequence
    {"turn handle counterclockwise"}	// a preplanned sequence
  };
  
  /** Publisher for recognized voice commands. */
  ros::Publisher pub_voice_commands_;
  
  /** Subscriber for utterance strings. */
  ros::Subscriber sub_utterances_;
  
  /** Instance of SoundClient used for text-to-speech synthesis. */
  sound_play::SoundClient sound_client_;
  
  /** Callback for subscribed utterances. */
  void utteranceToRecognizedCommand (std_msgs::String utterance);
  
  /** Prints out all the keys in command_list_. */
  void displayRecognizedVoiceCommands();
};
