/* 
 * File:  ObjectReader.h
 * Author: Grégoire Milliez
 * mail: gregoire.milliez@laas.fr
 *  Copyright 2014 LAAS/CNRS. All rights reserved.
 *
 * Created on December 3, 2014, 6:19 PM
 */

// An object reader is a class that will read data from a middleware message
// and fill a DynamicObject class from toaster-lib accordingly to publish on a ros topic.

#ifndef OBJECTREADER_H
#define	OBJECTREADER_H

#include <ros/ros.h>
#include "toaster-lib/MovableObject.h"
#include <map>

class ObjectReader {
public:
    std::map<std::string, MovableObject*> lastConfig_;
    unsigned int nbObjects_; /// total object number

    bool isPresent(std::string id);

};

#endif /* OBJECTREADER_H */
