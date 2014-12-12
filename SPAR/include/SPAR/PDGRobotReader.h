/* 
 * File:   PDGRobotReader.h
 * Author: gmilliez
 *
 * Created on November 12, 2014, 6:24 PM
 */

#ifndef PDGROBOTREADER_H
#define	PDGROBOTREADER_H


#include <ros/ros.h>
#include "toaster-lib/Robot.h"
#include "PDG/RobotList.h"
#include <map>

class PDGRobotReader{

    public:
       std::map<unsigned int, Robot*> lastConfig_;
       bool fullRobot_;

       bool isPresent(unsigned int id);

       PDGRobotReader(ros::NodeHandle& node, bool fullRobot);

    private:
       void robotJointStateCallBack(const PDG::RobotList::ConstPtr& msg);
       ros::Subscriber sub_;

};

#endif	/* PDGROBOTREADER_H */

