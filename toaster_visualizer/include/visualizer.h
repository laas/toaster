#include "ros/ros.h"
#include "toaster_msgs/Empty.h"
#include "toaster_msgs/Scale.h"

#include <string>

#ifndef VISUALIZER_H
#define VISUALIZER_H

class Visualizer
{
public:
  Visualizer(ros::NodeHandle* node);
  ~Visualizer() {};

  bool mustPrintName() {return printNames_;}
  float getObjectNameScale() {return objectNameScale_; }
  float getHumanNameScale() {return humanNameScale_; }
  float getRobotNameScale() {return robotNameScale_; }
  float getAreaNameScale() {return areaNameScale_; }

  /**
   * In our context names are identifier so we need to create an unique identifier for each input name
   * @param name		name of target
   * @return id		new identifier or target's identifier if his name already have been assigned to an identifier
   */
  int id_generator(std::string name);

private:
  std::vector<std::string> nameList_;
  int idCpt_;
  bool printNames_;

  float objectNameScale_;
  float humanNameScale_;
  float robotNameScale_;
  float areaNameScale_;

  ros::NodeHandle* node_;

  bool switchNamePrint(toaster_msgs::Empty::Request &req, toaster_msgs::Empty::Response &res);
  bool scaleName(toaster_msgs::Scale::Request &req, toaster_msgs::Scale::Response &res);

};

#endif
