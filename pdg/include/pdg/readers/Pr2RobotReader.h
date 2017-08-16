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
    Pr2RobotReader(bool fullRobot);
    ~Pr2RobotReader();

    void init(ros::NodeHandle* node, std::string param);

    void updateRobot(tf::TransformListener &listener);

private:
    bool initJointsName_;
    ros::Subscriber sub_;
    std::vector<std::string> pr2JointsName_;
    //void initJointsName();

    void setRobotJointLocation(tf::TransformListener &listener, Joint* joint);
    void pr2JointStateCallBack(const sensor_msgs::JointState::ConstPtr& msg);
};

#endif /* PR2ROBOTREADER_H */
