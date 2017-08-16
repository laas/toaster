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
#include "toaster_msgs/IoTData.h"


class OM2MObjectReader : public ObjectReader {

public:
    OM2MObjectReader();
    virtual ~OM2MObjectReader() {};

    void init(ros::NodeHandle* node, std::string topic, std::string param);

    virtual void Publish(struct toasterList_t& list_msg);

private:
    void newValueCallBack(const toaster_msgs::IoTData::ConstPtr& msg);

    std::string getSubpart(std::string data, std::string start, std::string stop);
    vector<struct preFact_t> readPreFacts(MovableObject* object);
};

#endif //PDG_OM2MOBJECTREADER_H
