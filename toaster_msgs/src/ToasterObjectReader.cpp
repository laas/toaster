/*
 * File:   ToasterObjectReader.cpp
 * Author: gmilliez
 *
 * Created on December 12, 2014, 2:23 PM
 */

#include "toaster_msgs/ToasterObjectReader.h"
#include "tf/transform_datatypes.h"

ToasterObjectReader::ToasterObjectReader(ros::NodeHandle& node, std::string topic) : EntityReader<Object>(false) {
    std::cout << "[Toaster_msgs] Initializing ToasterObjectReader" << std::endl;

    // Starts listening to the topic
    sub_ = node.subscribe(topic, 1, &ToasterObjectReader::objectStateCallBack, this);
}

void ToasterObjectReader::objectStateCallBack(const toaster_msgs::ObjectListStamped::ConstPtr& msg) {
    Object* curObject;
    double roll, pitch, yaw;

    for (unsigned int i = 0; i < msg->objectList.size(); i++) {

        // If this object is not assigned we have to allocate data.
        if (lastConfig_.find(msg->objectList[i].meEntity.id) == lastConfig_.end()) {
            curObject = new Object(msg->objectList[i].meEntity.id);
            curObject->setRoomId(0);
            curObject->setName(msg->objectList[i].meEntity.name);
        } else
            curObject = lastConfig_[msg->objectList[i].meEntity.id];

        std::vector<double> objOrientation;
        bg::model::point<double, 3, bg::cs::cartesian> objPosition;

        curObject->setId(msg->objectList[i].meEntity.id);

        curObject->setTime(msg->objectList[i].meEntity.time);

        objPosition.set<0>(msg->objectList[i].meEntity.pose.position.x);
        objPosition.set<1>(msg->objectList[i].meEntity.pose.position.y);
        objPosition.set<2>(msg->objectList[i].meEntity.pose.position.z);
        curObject->setPosition(objPosition);

        tf::Quaternion q;

        tf::quaternionMsgToTF(msg->objectList[i].meEntity.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        objOrientation.push_back(roll);
        objOrientation.push_back(pitch);
        objOrientation.push_back(yaw);

        curObject->setOrientation(objOrientation);

        if (lastConfig_[msg->objectList[i].meEntity.id] == NULL)
            lastConfig_[curObject->getId()] = curObject;
    }
}
