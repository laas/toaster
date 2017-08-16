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
#include "toaster_msgs/ObjectListStamped.h"
#include "ObjectReader.h"

class ToasterSimuObjectReader : public ObjectReader {
public:
    ToasterSimuObjectReader();
    ~ToasterSimuObjectReader() {};

    void init(ros::NodeHandle* node, std::string param);

private:
    //Functions
    void objectStateCallBack(const toaster_msgs::ObjectListStamped::ConstPtr& msg);
};

#endif	/* TOASTERSIMUOBJECTREADER_H */
