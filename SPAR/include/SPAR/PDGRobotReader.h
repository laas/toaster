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
#include "PDG/Robot.h"
#include <map>

class PDGRobotReader{

    public:
       std::map<int, Robot*> m_LastConfig;
       bool fullRobot_;

       bool isPresent(int id);

       PDGRobotReader(ros::NodeHandle& node, bool fullRobot);

    private:
       void robotJointStateCallBack(const PDG::Robot::ConstPtr& msg);
       ros::Subscriber sub_;

};

#endif	/* PDGROBOTREADER_H */

