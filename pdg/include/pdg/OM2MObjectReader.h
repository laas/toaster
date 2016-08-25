//
// Created by dkurzaj on 8/23/16.
//

#ifndef PDG_OM2MOBJECTREADER_H
#define PDG_OM2MOBJECTREADER_H

#include <ros/ros.h>
#include <string>
#include "ObjectReader.h"
#include "toaster-lib/MovableIoTObject.h"
#include "iot_bridge/IoTData.h"


class OM2MObjectReader : public ObjectReader {

public:
    OM2MObjectReader(ros::NodeHandle& node, std::string topicOM2M);

private:
    ros::Subscriber sub_;
    void newValueCallBack(const iot_bridge::IoTData::ConstPtr& msg);
};

#endif //PDG_OM2MOBJECTREADER_H
