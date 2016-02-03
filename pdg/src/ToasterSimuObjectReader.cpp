/* 
 * File:   ToasterSimuObjectReader.cpp
 * Author: gmilliez
 * 
 * Created on February 3, 2016, 12:14 PM
 */

#include "pdg/ToasterSimuObjectReader.h"

ToasterSimuObjectReader::ToasterSimuObjectReader(ros::NodeHandle& node) {
    std::cout << " Initializing ToasterSimuObjectReader" << std::endl;

    // Starts listening to the topic
    sub_ = node.subscribe("/toastersimu/objectList", 1, &ToasterSimuObjectReader::objectStateCallBack, this);
}

void ToasterSimuObjectReader::objectStateCallBack(const toaster_msgs::ObjectList::ConstPtr& msg) {
    //std::cout << "[area_manager][DEBUG] new data for object received" << std::endl;

    MovableObject* curObject;
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

        objPosition.set<0>(msg->objectList[i].meEntity.positionX);
        objPosition.set<1>(msg->objectList[i].meEntity.positionY);
        objPosition.set<2>(msg->objectList[i].meEntity.positionZ);
        curObject->setPosition(objPosition);

        objOrientation.push_back(msg->objectList[i].meEntity.orientationRoll);
        objOrientation.push_back(msg->objectList[i].meEntity.orientationPitch);
        objOrientation.push_back(msg->objectList[i].meEntity.orientationYaw);
        curObject->setOrientation(objOrientation);

        if (lastConfig_[msg->objectList[i].meEntity.id] == NULL)
            lastConfig_[curObject->getId()] = curObject;
    }
}
