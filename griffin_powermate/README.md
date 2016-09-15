# griffin_powermate
### ROS driver for Griffin Powermate turn knob

On `/griffin_powermate/events` topic it publishes `griffin_powermate::PowermateEvent` messages that contain direction and integral of the turn wheel as well as the information about push button being pressed or depressed.

#### Instructions
Run:

`rosrun griffin_powermate griffin_powermate`

#### Troubleshooting
**NB!** Accessing linux events requires root privileges. Make sure you have proper access, otherwise this package will not work.

Use `ls -l /dev/input/event*` to learn which group can access the events.

Then add your username to this group with `sudo usermod -a -G <group_name> <user_name>`, log out and log in.

To make it work in most cases, use the following command to add your username to the group `root`, log out, and log in.

`sudo usermod -a -G root <user_name>`
