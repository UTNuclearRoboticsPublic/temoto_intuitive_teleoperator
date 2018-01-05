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
    void keyboardCallback(keyboard_reader::Key kbCommand)
    {
        if (kbCommand.key_pressed == true)
        {

            // Run xmodmap -pk to display these

            // Plan trajecory: "p" key
            if (kbCommand.key_code == 0x0019 )
            {
                voiceCommand.data = "robot please plan";
                publish = true;
            }

            // Execute trajectory: "e" key
            else if (kbCommand.key_code == 0x0012)
            {
                voiceCommand.data = "robot please execute";
                publish = true;
            }

            // Limit directions: "l" key
            else if (kbCommand.key_code == 0x0026)
            {
                voiceCommand.data = "limit directions";
                publish = true;
            }

            // Limit directions: "f" key
            else if (kbCommand.key_code == 0x0021)
            {
                voiceCommand.data = "free directions";
                publish = true;
            }

            // Ignore rotation: "x" key
            else if (kbCommand.key_code == 0x002d)
            {
                voiceCommand.data = "ignore rotation";
                publish = true;
            }

            // Consider rotation: "r" key
            else if (kbCommand.key_code == 0x0013)
            {
                voiceCommand.data = "consider rotation";
                publish = true;
            }

            // Manipulation: "m" key
            else if (kbCommand.key_code == 0x0032)
            {
                voiceCommand.data = "manipulation";
                publish = true;
            }

            // Navigation: "n" key
            else if (kbCommand.key_code == 0x0031)
            {
                voiceCommand.data = "navigation";
                publish = true;
            }

            // Robot please go: "g" key
            else if (kbCommand.key_code == 0x0067)
            {
                voiceCommand.data = "robot please go";
                publish = true;
            }

            // Open gripper: "o" key
            else if (kbCommand.key_code == 0x0018)
            {
                voiceCommand.data = "open gripper";
                publish = true;
            }

            // Open gripper: "c" key
            else if (kbCommand.key_code == 0x002e)
            {
                voiceCommand.data = "close gripper";
                publish = true;
            }

            if (publish)
            {
                ROS_INFO("[fake voice commander] Publishing: %s", voiceCommand.data.c_str());
                voiceCommandPublisher.publish(voiceCommand);
                publish = false;
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_voice_commander");

    std::cout << std::endl << "* * * List of commands: * * *" << std::endl;
    std::cout << "* p - robot please plan" << std::endl;
    std::cout << "* e - robot please execute" << std::endl;
    std::cout << "* l - limit directions" << std::endl;
    std::cout << "* f - free directions" << std::endl;
    std::cout << "* x - ignore rotation" << std::endl;
    std::cout << "* r - consider rotation" << std::endl;
    std::cout << "* m - manipulation" << std::endl;
    std::cout << "* n - navigation" << std::endl;
    std::cout << "* o - open gripper" << std::endl;
    std::cout << "* c - close gripper" << std::endl;
    std::cout << "* * * * * * *" << std::endl << std::endl;

    ROS_INFO("Fake voice commander up and running");
    fake_voice_commander fakeVoiceCommander;

    ros::spin();

    return 0;
} // end main
