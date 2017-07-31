/*
 * File:   Pr2RobotReader.h
 * Author: Grégoire Milliez
 * mail: gregoire.milliez@laas.fr
 *  Copyright 2014 LAAS/CNRS. All rights reserved.
 *
 * Created on December 3, 2014, 6:19 PM
 */

#ifndef PR2ROBOTREADER_H
#define	PR2ROBOTREADER_H

#include "RobotReader.h"

#include <ostream>
#include <tf/transform_listener.h>
#include "sensor_msgs/JointState.h"

class Pr2RobotReader : public RobotReader {
public:
    Pr2RobotReader(ros::NodeHandle& node, bool fullRobot);
    void updateRobot(tf::TransformListener &listener);

    //Destructor
    virtual ~Pr2RobotReader();

private:
    bool initJointsName_;
    ros::Subscriber sub_;
    std::vector<std::string> pr2JointsName_;
    //void initJointsName();
    void init();
    void setRobotJointLocation(tf::TransformListener &listener, Joint* joint);
    void pr2JointStateCallBack(const sensor_msgs::JointState::ConstPtr& msg);
};

#endif /* PR2ROBOTREADER_H */
