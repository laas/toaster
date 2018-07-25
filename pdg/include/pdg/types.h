#ifndef TYPES_H
#define TYPES_H

#include <map>
#include <string>

// Message generated class
#include <toaster_msgs/FactList.h>
#include <toaster_msgs/RobotListStamped.h>
#include <toaster_msgs/HumanListStamped.h>
#include <toaster_msgs/ObjectListStamped.h>

struct objectIn_t
{
  std::map<std::string, std::string> Agent_;
  std::map<std::string, std::string> Hand_;
};

struct fullConfig_t
{
  bool Human_ = true; //If false we will use only position and orientation
  bool Robot_ = true; //If false we will use only position and orientation
};

struct toasterList_t
{
  toaster_msgs::ObjectListStamped object_msg;
  toaster_msgs::HumanListStamped human_msg;
  toaster_msgs::RobotListStamped robot_msg;
  toaster_msgs::FactList fact_msg;
};

#endif
