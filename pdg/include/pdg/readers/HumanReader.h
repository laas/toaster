/* 
 * File:   HumanReader.h
 * Author: Grégoire Milliez
 * mail: gregoire.milliez@laas.fr
 *  Copyright 2014 LAAS/CNRS. All rights reserved.
 *
 * Created on December 3, 2014, 6:19 PM
 */

// A human reader is a class that will read data from a middleware message
// and fill a Human class from toaster-lib accordingly to publish on a ros topic.


#ifndef HUMANREADER_H
#define	HUMANREADER_H

#include <ros/ros.h>
#include "toaster-lib/Human.h"
#include <map>

class HumanReader{

    public:
        std::map<std::string, Human*> lastConfig_;
        bool fullHuman_;

        bool isPresent(std::string id);

};


#endif	/* HUMANREADER_H */