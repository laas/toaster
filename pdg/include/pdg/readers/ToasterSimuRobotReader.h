/*
 * File:   ToasterSimuRobotReader.h
 * Author: gmilliez
 *
 * Created on February 3, 2016, 12:14 PM
 */

#ifndef TOASTERSIMUROBOTREADER_H
#define	TOASTERSIMUROBOTREADER_H

#include <ros/ros.h>
#include "toaster-lib/Robot.h"
#include "toaster_msgs/RobotListStamped.h"
#include "pdg/readers/RobotReader.h"

class ToasterSimuRobotReader : public RobotReader {
public:
    ToasterSimuRobotReader(ros::NodeHandle& node);
private:
    void robotJointStateCallBack(const toaster_msgs::RobotListStamped::ConstPtr& msg);
    ros::Subscriber sub_;
};

#endif	/* TOASTERSIMUROBOTREADER_H */
