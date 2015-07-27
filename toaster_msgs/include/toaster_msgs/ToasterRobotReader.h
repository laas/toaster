/* 
 * File:   ToasterobotReader.h
 * Author: gmilliez
 *
 * Created on November 12, 2014, 6:24 PM
 */

#ifndef TOASTERROBOTREADER_H
#define	TOASTERROBOTREADER_H


#include <ros/ros.h>
#include "toaster-lib/Robot.h"
#include "toaster_msgs/RobotList.h"
#include <map>

class ToasterRobotReader{

    public:
       std::map<std::string, Robot*> lastConfig_;
       bool fullRobot_;

       bool isPresent(std::string id);

       ToasterRobotReader(ros::NodeHandle& node, bool fullRobot);

    private:
       void robotJointStateCallBack(const toaster_msgs::RobotList::ConstPtr& msg);
       ros::Subscriber sub_;

};

#endif	/* TOASTERROBOTREADER_H */

