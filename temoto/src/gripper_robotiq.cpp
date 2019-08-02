
#include "temoto/gripper_robotiq.h"

using namespace std;

// GripperRobotiq::GripperRobotiq()
// {
// /*
//   // Create a publisher for each gripper
//   for (std::string topic : gripper_topics)
//   {
//     ros::Publisher gripper_pub = nh_.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>(topic, 1);
//     gripper_publishers_.push_back(std::make_shared<ros::Publisher>(gripper_pub));
//   }
// */
// }

// void GripperRobotiq::close()
// {
//   ROS_ERROR_STREAM("Closing");
// /*
//   void* handle = dlopen("./myclass.so", RTLD_LAZY);

//   MyClass* (*create)();
//   void (*destroy)(MyClass*);

//   create = (MyClass* (*)())dlsym(handle, "create_object");
//   destroy = (void (*)(MyClass*))dlsym(handle, "destroy_object");

//   MyClass* myClass = (MyClass*)create();
//   myClass->DoSomething();
//   destroy( myClass );


//   // Find the publisher on this topic
//   for (auto pub : gripper_publishers_)
//   {
//     if (pub->getTopic() == gripper_topic)
//     {
//       robotiq_2f_gripper_control::Robotiq2FGripper_robot_output gripper_msg;
//       gripper_msg.rPR = 255;
//       gripper_msg.rACT = 1;
//       gripper_msg.rGTO = 1;
//       gripper_msg.rATR = 0;
//       gripper_msg.rSP = 255;
//       gripper_msg.rFR = 150;

//       pub->publish(gripper_msg);
//       break;
//     }
//   }
// */
// }

// void GripperRobotiq::open()
// {
//   ROS_ERROR_STREAM("Opening");
//   // Find the publisher on this topic
// /*
//   for (auto pub : gripper_publishers_)
//   {
//     if (pub->getTopic() == gripper_topic)
//     {
//       robotiq_2f_gripper_control::Robotiq2FGripper_robot_output gripper_msg;
//       gripper_msg.rPR = 0;
//       gripper_msg.rACT = 1;
//       gripper_msg.rGTO = 1;
//       gripper_msg.rATR = 0;
//       gripper_msg.rSP = 255;
//       gripper_msg.rFR = 150;

//       pub->publish(gripper_msg);
//       break;
//     }
//   }
// */
// }
