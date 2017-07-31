/*
 * File:   OM2MObjectReader.h
 * Author: Dorian Kurzaj
 *
 * Created on August, 2016
 */

#ifndef PDG_OM2MOBJECTREADER_H
#define PDG_OM2MOBJECTREADER_H

#include <ros/ros.h>
#include <string>
#include "ObjectReader.h"
#include "toaster-lib/MovableIoTObject.h"
#include "toaster_msgs/IoTData.h"


class OM2MObjectReader : public ObjectReader {

public:
    OM2MObjectReader(ros::NodeHandle& node, std::string topicOM2M);

    virtual ~OM2MObjectReader() {};

private:
    ros::Subscriber sub_;
    void newValueCallBack(const toaster_msgs::IoTData::ConstPtr& msg);
};

#endif //PDG_OM2MOBJECTREADER_H
