
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
#include "toaster_msgs/HumanList.h"
#include <map>

class ToasterHumanReader {
public:
    std::map<std::string, Human*> lastConfig_;
    bool fullHuman_;

    bool isPresent(std::string id);

    ToasterHumanReader(ros::NodeHandle& node, bool fullHuman, std::string topic = "/pdg/humanList");

private:
    void humanJointStateCallBack(const toaster_msgs::HumanList::ConstPtr& msg);
    ros::Subscriber sub_;

};

#endif	/* TOASTERHUMANREADER_H */
