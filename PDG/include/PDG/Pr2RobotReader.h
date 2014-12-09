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

class Pr2RobotReader : public RobotReader{

  public:
    Pr2RobotReader(bool fullRobot);
    void updateRobot(tf::TransformListener &listener);
    
    //Destructor
    ~Pr2RobotReader();

  private:
    ros::Subscriber sub_;
    std::vector<std::string> pr2JointsName_;
    void initJointsName();
    void init();
    void setRobotJointLocation(tf::TransformListener &listener, Joint* joint);
};

#endif /* PR2ROBOTREADER_H */