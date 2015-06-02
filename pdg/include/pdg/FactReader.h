/* 
 * File:   FactReader.h
 * Author: gmilliez
 *
 * Created on May 31, 2015, 1:13 PM
 */

#ifndef FACTREADER_H
#define	FACTREADER_H

#include <ros/ros.h>
#include "toaster_msgs/FactList.h"

class FactReader {
public:
    toaster_msgs::FactList currentFactList_;
    unsigned int nbFacts_; /// total number of facts

};
#endif	/* FACTREADER_H */

