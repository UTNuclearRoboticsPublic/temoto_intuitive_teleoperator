#include "ros/ros.h"
#include "stdio.h"
#include "stdlib.h"
#include "string"
#include "vector"

// Work with X11 to determine whether Temoto should have access to keyboard commands.
// There is a list of other applications that take priority when they are the
// window in focus.

#ifndef CHECK_FOR_TEMOTO_KEYBOARD_PRIORITY_H
#define CHECK_FOR_TEMOTO_KEYBOARD_PRIORITY_H

namespace keyboard_priority
{

  class TemotoKeyboardPriority
  {
  public:
    TemotoKeyboardPriority();
    ~TemotoKeyboardPriority();

    bool checkForTemotoKeyboardPriority();

  private:
    // Check whether the given window exists
    void checkStatus(int status, unsigned long window);

    // Get a string property from an X11 window
    unsigned char* getStringProperty(char* property_name);

    // Format longer X11 property strings
    unsigned long getLongProperty(char* property_name);

    // Define any applications that should have keyboard priority
    // over Temoto, like terminals and rqt
    std::vector<std::string> applications_with_higher_priority{"rqt", "term", "rviz"};

    unsigned long window_;
    int screen_;
  };

} // End namespace keyboard_priority
#endif