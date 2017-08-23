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
#include "toaster_msgs/RobotListStamped.h"
#include "toaster_msgs/EntityReader.h"
#include <map>

class ToasterRobotReader : public EntityReader<Robot>{

    public:
       ToasterRobotReader(ros::NodeHandle& node, bool fullRobot, std::string topic="pdg/robotList");

    private:
       void robotJointStateCallBack(const toaster_msgs::RobotListStamped::ConstPtr& msg);
       ros::Subscriber sub_;

};

#endif	/* TOASTERROBOTREADER_H */
