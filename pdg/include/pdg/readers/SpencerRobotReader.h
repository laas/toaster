/*
 * File:   SpencerRobotReader.h
 * Author: gmilliez
 *
 * Created on April 20, 2015, 5:15 PM
 */

#ifndef SPENCERROBOTREADER_H
#define	SPENCERROBOTREADER_H

#include "RobotReader.h"

#include <tf/transform_listener.h>

class SpencerRobotReader : public RobotReader {
public:
    SpencerRobotReader();
    ~SpencerRobotReader() {};

    void init(ros::NodeHandle* node, std::string param);

    void updateRobot(tf::TransformListener &listener);
private:
    ros::Subscriber sub_;
    std::vector<std::string> spencerJointsName_;
    void initJointsName();
    void setRobotJointLocation(tf::TransformListener &listener, Joint* joint);

};

#endif	/* SPENCERROBOTREADER_H */
