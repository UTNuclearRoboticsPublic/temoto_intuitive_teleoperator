#include "temoto/get_ros_params.h"

std::vector<std::string> get_ros_params::getStrVecParam(std::string name, ros::NodeHandle& n)
{
  std::vector<std::string> str_vec;
  if (!n.getParam(name, str_vec))
    ROS_ERROR_STREAM("[getStringParam] YAML config file does not contain parameter " << name);
  return str_vec;
}

std::string get_ros_params::getStringParam(std::string s, ros::NodeHandle& n)
{
  if (!n.getParam(s, s))
    ROS_ERROR_STREAM("[getStringParam] YAML config file does not contain parameter " << s);
  return s;
}

double get_ros_params::getDoubleParam(std::string name, ros::NodeHandle& n)
{
  double value;
  if (!n.getParam(name, value))
    ROS_ERROR_STREAM("[getDoubleParam] YAML config file does not contain parameter " << name);
  return value;
}

double get_ros_params::getIntParam(std::string name, ros::NodeHandle& n)
{
  int value;
  if (!n.getParam(name, value))
    ROS_ERROR_STREAM("[getIntParam] YAML config file does not contain parameter " << name);
  return value;
}

bool get_ros_params::getBoolParam(std::string name, ros::NodeHandle& n)
{
  bool value;
  if (!n.getParam(name, value))
    ROS_ERROR_STREAM("[getBoolParam] YAML config file does not contain parameter " << name);
  return value;
}