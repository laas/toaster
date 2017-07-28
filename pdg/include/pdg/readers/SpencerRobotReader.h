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
    SpencerRobotReader(bool fullRobot);
    void updateRobot(tf::TransformListener &listener);
    
    //Destructor
    virtual ~SpencerRobotReader();
private:
    ros::Subscriber sub_;
    std::vector<std::string> spencerJointsName_;
    void initJointsName();
    void init();
    void setRobotJointLocation(tf::TransformListener &listener, Joint* joint);

};

#endif	/* SPENCERROBOTREADER_H */

