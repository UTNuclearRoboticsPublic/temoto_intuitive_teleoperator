#include "keyboard_reader/Key.h"
#include "map"
#include "ros/ros.h"
#include "std_msgs/String.h"

class fake_voice_commander
{
public:
  ros::NodeHandle n;
  ros::Publisher voiceCommandPublisher;
  ros::Subscriber sub_kb_event;
  std_msgs::String voiceCommand;
  bool publish = false;

  fake_voice_commander()
  {
    // Setup ROS publishers
    voiceCommandPublisher = n.advertise<std_msgs::String>("stt/spoken_text", 2);

    // Setup ROS subscribers
    sub_kb_event = n.subscribe<keyboard_reader::Key>("keyboard", 1, &fake_voice_commander::keyboardCallback, this);
  }

  // Process keypresses
  // To determine these keys, run: "rosrun keyboard_reader
  // keyboard_event_publisher"
  void keyboardCallback(keyboard_reader::Key kbCommand)
  {
    if (kbCommand.key_pressed == true)
    {
      // Run xmodmap -pk to display these

      // Plan trajecory: "l" key
      if (kbCommand.key_code == 0x0026)
        voiceCommand.data = "robot please plan";

      // Execute trajectory: "e" key
      else if (kbCommand.key_code == 0x0012)
        voiceCommand.data = "robot please execute";

      // Manipulation: "m" key
      else if (kbCommand.key_code == 0x0032)
        voiceCommand.data = "manipulation";

      // Navigation: "n" key
      else if (kbCommand.key_code == 0x0031)
        voiceCommand.data = "navigation";

      // Robot please go: "b" key
      else if (kbCommand.key_code == 0x0030)
        voiceCommand.data = "base move";

      // Open gripper: "o" key
      else if (kbCommand.key_code == 0x0018)
        voiceCommand.data = "open gripper";

      // Open gripper: "c" key
      else if (kbCommand.key_code == 0x002e)
        voiceCommand.data = "close gripper";

      // Jog mode: "j" key
      else if (kbCommand.key_code == 0x0024)
        voiceCommand.data = "jog mode";

      // Point-to-point mode: "p" key
      else if (kbCommand.key_code == 0x0019)
        voiceCommand.data = "point to point mode";

      // Next end effector: "tab" key
      else if (kbCommand.key_code == 0x000f)
        voiceCommand.data = "next end effector";

      // Execute trajectory: "1" key
      else if (kbCommand.key_code == 0x0002)
        voiceCommand.data = "set position limited";

      // Execute trajectory: "2" key
      else if (kbCommand.key_code == 0x0003)
        voiceCommand.data = "set position fwd only";

      // Execute trajectory: "3" key
      else if (kbCommand.key_code == 0x0004)
        voiceCommand.data = "set orientation locked";

      // Execute trajectory: "4" key
      else if (kbCommand.key_code == 0x0005)
        voiceCommand.data = "unlock position limited";      
      
      // Execute trajectory: "5" key
      else if (kbCommand.key_code == 0x0006)
        voiceCommand.data = "unlock fwd only";      
      
      // Execute trajectory: "6" key
      else if (kbCommand.key_code == 0x0007)
        voiceCommand.data = "unlock orientation";

      else
        voiceCommand.data = "unrecognized command";

      ROS_INFO("[fake voice commander] Publishing: %s", voiceCommand.data.c_str());
      voiceCommandPublisher.publish(voiceCommand);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_voice_commander");

  std::cout << std::endl << "* * * List of commands: * * *" << std::endl;
  std::cout << "* l - robot please plan" << std::endl;
  std::cout << "* e - robot please execute" << std::endl;  
  std::cout << "* m - manipulation" << std::endl;
  std::cout << "* n - navigation" << std::endl;
  std::cout << "* b - base move" << std::endl;
  std::cout << "* o - open gripper" << std::endl;
  std::cout << "* c - close gripper" << std::endl;
  std::cout << "* j - jog mode" << std::endl;
  std::cout << "* p - point to point mode" << std::endl;
//  std::cout << "* /t - next end effector" << std::endl;

  std::cout << "* 1 - set position limited" << std::endl;
  std::cout << "* 2 - set position fwd only" << std::endl;
  std::cout << "* 3 - set orientation locked" << std::endl;
  std::cout << "* 4 - unlock position limited" << std::endl;
  std::cout << "* 5 - unlock fwd only" << std::endl;
  std::cout << "* 6 - unlock orientation" << std::endl;

  std::cout << "* * * * * * *" << std::endl << std::endl;

  ROS_INFO("Fake voice commander up and running");
  fake_voice_commander fakeVoiceCommander;

  ros::spin();

  return 0;
}  // end main
