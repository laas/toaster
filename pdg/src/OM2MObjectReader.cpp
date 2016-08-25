//
// Created by dkurzaj on 8/23/16.
//

#include "pdg/OM2MObjectReader.h"

OM2MObjectReader::OM2MObjectReader(ros::NodeHandle& node, std::string topicOM2M) {
    ROS_INFO("[OM2MObjectReader] Initializing");
    sub_ = node.subscribe(topicOM2M, 1, &OM2MObjectReader::newValueCallBack, this);
    ROS_INFO("[OM2MObjectReader] done");
}

void OM2MObjectReader::newValueCallBack(const iot_bridge::IoTData::ConstPtr& msg) {
    //std::cout << "new value : %s" + msg->data.key;
    ros::Time now = ros::Time::now();
    // OM2M objects are by convention considered movable
    MovableIoTObject* curObject;

    //if the object does not exist : create a new object with the same id and name as the key of message and placed at 0
    if (lastConfig_.find(msg->data.key) == lastConfig_.end()) {
        ROS_INFO("[OM2MObjectReader] New value from new %s : %s", (msg->data.key).c_str(), (msg->data.value).c_str());
        curObject = new MovableIoTObject(msg->data.key);
        curObject->setName(msg->data.key);

        //set object position
        bg::model::point<double, 3, bg::cs::cartesian> objectPosition;
        objectPosition.set<0>(0.0);
        objectPosition.set<1>(0.0);
        objectPosition.set<2>(0.0);

        //set the object orientation
        std::vector<double> objectOrientation;
        objectOrientation.push_back(0.0);
        objectOrientation.push_back(0.0);
        objectOrientation.push_back(0.0);


        //put the same orientation and positions as the previous time
        curObject->setOrientation(objectOrientation);
        curObject->setPosition(objectPosition);
    // otherwise : we just copy the previous object
    } else {
        ROS_INFO("[OM2MObjectReader] New value from existing %s : %s", (msg->data.key).c_str(), (msg->data.value).c_str());
        curObject = (MovableIoTObject*)lastConfig_[msg->data.key];
    }

    // set the new value of the OM2M object
    curObject->setValue(msg->data.value);

    unsigned long milli_sec = msg->header.stamp.sec * 1000;
    curObject->setTime(milli_sec);


    lastConfig_[msg->data.key]=curObject;
}
