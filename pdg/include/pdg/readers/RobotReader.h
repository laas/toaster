/*
 * File:   RobotReader.h
 * Author: Grégoire Milliez
 * mail: gregoire.milliez@laas.fr
 *  Copyright 2014 LAAS/CNRS. All rights reserved.
 *
 * Created on December 3, 2014, 6:19 PM
 */

// A robot reader is a class that will read data from a middleware message
// and fill a Robot class from toaster-lib accordingly to publish on a ros topic.

#ifndef ROBOTREADER_H
#define	ROBOTREADER_H

#include <ros/ros.h>
#include "toaster-lib/Robot.h"
#include <map>
#include <string>

class RobotReader{

    public:
        RobotReader();
        ~RobotReader();

        std::map<std::string, Robot*> lastConfig_;

    protected:
        bool fullRobot_;

        bool isPresent(std::string id);
};

#endif /* ROBOTREADER_H */
