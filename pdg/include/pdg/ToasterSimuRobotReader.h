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
#include "toaster_msgs/RobotList.h"
#include "pdg/RobotReader.h"

class ToasterSimuRobotReader : public RobotReader {
public:
    ToasterSimuRobotReader(ros::NodeHandle& node);
private:
    void robotJointStateCallBack(const toaster_msgs::RobotList::ConstPtr& msg);
    ros::Subscriber sub_;
};

#endif	/* TOASTERSIMUROBOTREADER_H */
