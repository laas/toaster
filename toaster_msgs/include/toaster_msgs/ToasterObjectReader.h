/* 
 * File:   ToasterObjectReader.h
 * Author: gmilliez
 *
 * Created on December 12, 2014, 2:20 PM
 */

#ifndef TOASTEROBJECTREADER_H
#define	TOASTEROBJECTREADER_H

#include <ros/ros.h>
#include "toaster-lib/Object.h"
#include "toaster_msgs/ObjectListStamped.h"
#include <map>

class ToasterObjectReader {
public:
    std::map<std::string, Object*> lastConfig_;

    bool isPresent(std::string id);

    ToasterObjectReader(ros::NodeHandle& node, std::string topic="pdg/objectList");

private:
    void objectStateCallBack(const toaster_msgs::ObjectListStamped::ConstPtr& msg);
    ros::Subscriber sub_;

};

#endif	/* TOASTEROBJECTREADER_H */

