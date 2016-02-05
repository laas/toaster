/* 
 * File:   ToasterSimuObjectReader.h
 * Author: gmilliez
 *
 * Created on February 3, 2016, 12:14 PM
 */

#ifndef TOASTERSIMUOBJECTREADER_H
#define	TOASTERSIMUOBJECTREADER_H

#include <ros/ros.h>
#include "toaster-lib/Object.h"
#include "toaster_msgs/ObjectList.h"
#include "ObjectReader.h"

class ToasterSimuObjectReader : public ObjectReader {
public:
    ToasterSimuObjectReader(ros::NodeHandle& node);

private:
    //Functions
    void objectStateCallBack(const toaster_msgs::ObjectList::ConstPtr& msg);
    ros::Subscriber sub_;
};

#endif	/* TOASTERSIMUOBJECTREADER_H */

