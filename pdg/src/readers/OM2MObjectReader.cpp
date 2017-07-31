/*
 * File:   OM2MObjectReader.cpp
 * Author: Dorian Kurzaj
 *
 * Created on August, 2016
 */

#include "pdg/readers/OM2MObjectReader.h"

#include "toaster-lib/MovableIoTObject.h"

OM2MObjectReader::OM2MObjectReader(ros::NodeHandle& node, std::string topicOM2M) {

    ROS_INFO("[pdg][OM2MObjectReader] Initializing");

    sub_ = node.subscribe(topicOM2M, 1, &OM2MObjectReader::newValueCallBack, this);

    ROS_INFO("[pdg][OM2MObjectReader] done");
}

void OM2MObjectReader::newValueCallBack(const toaster_msgs::IoTData::ConstPtr& msg) {

    ros::Time now = ros::Time::now();
    // OM2M objects are by convention considered movable
    MovableIoTObject* curObject;

    //create a new object with the same id and name as the message
    if (lastConfig_.find(msg->data.key) == lastConfig_.end()) {
        curObject = new MovableIoTObject(msg->data.key);
        curObject->setName(msg->data.key);
    } else {
        curObject = (MovableIoTObject*)lastConfig_[msg->data.key];
    }


    //set object position at default : 0,0,0
    bg::model::point<double, 3, bg::cs::cartesian> objectPosition;
    objectPosition.set<0>(0);
    objectPosition.set<1>(0);
    objectPosition.set<2>(0);

    //set the object orientation
    std::vector<double> objectOrientation;
    objectOrientation.push_back(0.0);
    objectOrientation.push_back(0.0);
    objectOrientation.push_back(0.0);


    //put the same orientation and positions as the previous time
    curObject->setOrientation(objectOrientation);
    curObject->setPosition(objectPosition);

    // set the new value of the OM2M object
    curObject->setValue(msg->data.value);

    // set the time
    unsigned long micro_sec = msg->header.stamp.sec * 1000000;
    curObject->setTime(micro_sec);


    lastConfig_[msg->data.key]=curObject;
}
