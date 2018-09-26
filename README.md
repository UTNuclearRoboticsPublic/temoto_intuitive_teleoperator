# temoto_intuitive_teleoperator
手元 (Temoto) is ROS-based hardware-agnostic intuitive teleoperation software using spatial-mapping and natural language input.

# Installation
This package contains submodules. When you clone it, do **git clone --recursive https://github.com/UTNuclearRoboticsPublic/temoto_intuitive_teleoperator.git**

# Setup
Create these two files for your particular robot. Use the other files as examples:

temoto/config/<your_robot>.yaml

temoto/launch/<your_robot>.launch

If you want an accurate visualization of your end-effector to appear, add a mesh file in temoto/meshes. Otherwise you will get a blocky representation of the end-effector.

### Launch file tips
Your robot's launch file needs to do these things:

1) Load the Temoto config file:

**rosparam command="load" file="$(find temoto)/config/vaultbot.yaml"**

2) Run the main start_teleop node:

**node name="start_teleop" pkg="temoto" type="start_teleop"**

3) Launch a pose controller. Current options are an XBox controller or a SpaceNavigator Pro. LeapMotion is supported on a separate, outdated branch and likely will not be updated in the future. Example:

**node name="spacenav_pro" pkg="spacenav_pro" type="spacenav_pro"**

4) If you're using a SpaceNavigator Pro, then it already has enough buttons and you can ignore this step. If you're using an XBox controller, it does not have enough buttons for every Temoto command. You need another way to send commands. Choose keyboard or voice commands:

Google speech-to-text:

**include file="$(find google_speech_to_text)/launch/google_speech_to_text.launch"**

Keyboard commands:

**node name="kb_fake_voice_commander" pkg="temoto" type="kb_fake_voice_commander" output="screen"**

**node name="keyboard_event_publisher" pkg="keyboard_reader" type="keyboard_event_publisher"**

5) Load the RViz display:

**include file="$(find temoto)/launch/vaultbot_moveit_rviz.launch"**

  **arg name="config" value="true"**
  
**include**

6) Start the soundplay node so Temoto can talk back to you (optional):

**node name="soundplay_node" pkg="sound_play" type="soundplay_node.py"**

7) Start a real-time jogger for each arm (optional):

**include file="$(find vaultbot_launch)/launch/jog_arms.launch" output="screen"**
