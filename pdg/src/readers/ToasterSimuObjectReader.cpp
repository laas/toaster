/*
 * File:   ToasterSimuObjectReader.cpp
 * Author: gmilliez
 *
 * Created on February 3, 2016, 12:14 PM
 */

#include "pdg/readers/ToasterSimuObjectReader.h"
#include "tf/transform_datatypes.h"

ToasterSimuObjectReader::ToasterSimuObjectReader(ros::NodeHandle& node) {
    std::cout << " Initializing ToasterSimuObjectReader" << std::endl;

    // Starts listening to the topic
    sub_ = node.subscribe("/toaster_simu/objectList", 1, &ToasterSimuObjectReader::objectStateCallBack, this);
}

void ToasterSimuObjectReader::objectStateCallBack(const toaster_msgs::ObjectListStamped::ConstPtr& msg) {
    //std::cout << "[area_manager][DEBUG] new data for object received" << std::endl;

    MovableObject* curObject;
    double roll, pitch, yaw;
    for (unsigned int i = 0; i < msg->objectList.size(); i++) {

        // If this object is not assigned we have to allocate data.
        if (lastConfig_.find(msg->objectList[i].meEntity.id) == lastConfig_.end()) {
            curObject = new MovableObject(msg->objectList[i].meEntity.id);
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
