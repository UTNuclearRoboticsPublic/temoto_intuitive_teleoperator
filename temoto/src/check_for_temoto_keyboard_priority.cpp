#include "temoto/check_for_temoto_keyboard_priority.h"

// These aren't in the header due to preprocessor clashing.
// See https://stackoverflow.com/questions/22400905/eigen-and-cimg-compatibility-issues
#include "X11/Xatom.h"
#include "X11/Xlib.h"

#define MAXSTR 1000

Display *display;


namespace keyboard_priority
{

// Constructor
TemotoKeyboardPriority::TemotoKeyboardPriority()
{
  char *display_name = NULL;
  display = XOpenDisplay(display_name);
  if (display == NULL)
    ROS_ERROR_STREAM("[check_for_temoto_keyboard_priority] Unable to open X11 display.");
  screen_ = XDefaultScreen(display);
}

// Destructor
TemotoKeyboardPriority::~TemotoKeyboardPriority()
{
  XCloseDisplay(display);
}

// return true if Temoto has priority
bool TemotoKeyboardPriority::checkForTemotoKeyboardPriority()
{
  window_ = RootWindow(display, screen_);
  window_ = getLongProperty("_NET_ACTIVE_WINDOW");
  ROS_ERROR_STREAM( getStringProperty("WM_CLASS") );

  return true;
}

void TemotoKeyboardPriority::checkStatus(int status, unsigned long window)
{
    if (status == BadWindow) {
        printf("window id # 0x%lx does not exists!", window);
        exit(1);
    }

    if (status != Success) {
        printf("XGetWindowProperty failed!");
        exit(2);
    }
}

unsigned char* TemotoKeyboardPriority::getStringProperty(char* property_name)
{
    Atom actual_type, filter_atom;
    int actual_format, status;
    unsigned long nitems, bytes_after;
    unsigned char *prop;

    filter_atom = XInternAtom(display, property_name, True);
    status = XGetWindowProperty(display, window_, filter_atom, 0, MAXSTR, False, AnyPropertyType,
                                &actual_type, &actual_format, &nitems, &bytes_after, &prop);
    checkStatus(status, window_);
    return prop;
}

unsigned long TemotoKeyboardPriority::getLongProperty(char* property_name)
{
    unsigned char *prop = getStringProperty(property_name);
    unsigned long long_property = prop[0] + (prop[1]<<8) + (prop[2]<<16) + (prop[3]<<24);
    return long_property;
}

} // End namespace keyboard_priority