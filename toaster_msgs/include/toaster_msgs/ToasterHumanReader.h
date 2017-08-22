
/*
 * File:   TOASTERRobotReader.h
 * Author: gmilliez
 *
 * Created on November 12, 2014, 6:24 PM
 * This class will be used to read toaster-lib humans from topic.

 */

#ifndef TOASTERHUMANREADER_H
#define	TOASTERHUMANREADER_H

#include <ros/ros.h>
#include "toaster-lib/Human.h"
#include "toaster_msgs/HumanListStamped.h"
#include "toaster_msgs/EntityReader.h"
#include <map>

class ToasterHumanReader : public EntityReader<Human>{
public:
    ToasterHumanReader(ros::NodeHandle& node, bool fullHuman, std::string topic="pdg/humanList");

private:
    void humanJointStateCallBack(const toaster_msgs::HumanListStamped::ConstPtr& msg);
    ros::Subscriber sub_;

};

#endif	/* TOASTERHUMANREADER_H */
