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

/** @file interpret_utterance.cpp
 *  Subscribes to the verbal command topic and tries to extract valid voice commands
 *  from it. If valid voice command is extracted, an approproate command code is published on
 *  "temoto/voice_commands".
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

#include "temoto/interpret_utterance.h"

/** Callback function for the verbal cmd topic.
 *  It searches the voice command string for any of the strings specified in valid_voice_commands.
 *  @param last_phrase pocketsphinx_recognizer output string.
 */
void Interpreter::utteranceToRecognizedCommand(std_msgs::String utterance)
{
  // No valid commands by default
  bool valid_command = false;
  
  // Set namespace for voice command messages
  temoto::Command latest_command;
  latest_command.ns = "temoto_voice_command";
  
  // Print the output of pocketsphinx_recognizer
  //ROS_INFO_STREAM("[interpret_utterance] I think you just said: " << s);
 
  // Compare the input utterance string to every string in command_list_.
  for(int it=0; it<command_list_.size(); ++it)
  {
    if ( utterance.data.c_str() == command_list_.at(it)) {		// if valid voice command was found in utterance
      latest_command.cmd_string = command_list_.at(it);		// take the command
      valid_command = true;
    } // end if
  } // end for
  
  // if a valid command was found in utterance string
  if (valid_command)
  {
    // give operator auditory confirmation
    sound_client_.say("affirmative");
    // publish the voice command
    pub_voice_commands_.publish( latest_command );
  }
  else
  {
    // tell operator that no valid command was recgonized in utterance string
    sound_client_.say("i beg your pardon");
  }
  
  return;
}

/** Displays on the screen all the strings that are considered valid voice commands. */
void Interpreter::displayRecognizedVoiceCommands()
{
  ROS_INFO("All the accepted utterances are:");
  for(int it=0; it<command_list_.size(); ++it)
  {
    std::cout << " == " << command_list_.at(it) << "\n";
  }
  
  return;
}

/** Main method. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "interpret_utterance");
  ros::NodeHandle n;
  
  // Instance of Interpreter
  Interpreter interpreter;
  
  // Subscribe to speech recognizer
  interpreter.sub_utterances_ = n.subscribe<std_msgs::String>("spoken_text", 5, &Interpreter::utteranceToRecognizedCommand, &interpreter);

  // Publish unambiguous commands based on speech recognition
  interpreter.pub_voice_commands_ = n.advertise<temoto::Command>("temoto/voice_commands", 2);
  
  // FYI
  ROS_INFO("Listening the operator talk ...");
  ROS_INFO("... and remember that manners maketh man.");

  // Output to the screen all the accepted voice commands
  interpreter.displayRecognizedVoiceCommands();

  // wait for for the sound_client server to come up
  // TODO: change to something that actually checks if the server is online
  sleep(1);

  // Audible FYI
  interpreter.sound_client_.say("Hello! I am ready to receive verbal instructions.");

  // wait for utterances
  ros::spin();
  
  return 0;
} // end main
