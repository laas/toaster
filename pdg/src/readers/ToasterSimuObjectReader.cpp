/*
 * File:   ToasterSimuObjectReader.cpp
 * Author: gmilliez
 *
 * Created on February 3, 2016, 12:14 PM
 */

#include "pdg/readers/ToasterSimuObjectReader.h"
#include "tf/transform_datatypes.h"

ToasterSimuObjectReader::ToasterSimuObjectReader() : ObjectReader() {
    childs_.push_back(this);
}

void ToasterSimuObjectReader::init(ros::NodeHandle* node, std::string param)
{
  std::cout << "[PDG] Initializing ToasterSimuObjectReader" << std::endl;
  Reader<MovableObject>::init(node, param);
  // ******************************************
  // Starts listening to the joint_states
  sub_ = node_->subscribe("/toaster_simu/objectList", 1, &ToasterSimuObjectReader::objectStateCallBack, this);
}

void ToasterSimuObjectReader::objectStateCallBack(const toaster_msgs::ObjectListStamped::ConstPtr& msg) {
    //std::cout << "[area_manager][DEBUG] new data for object received" << std::endl;
  if(activated_)
  {
    MovableObject* curObject;
    double roll, pitch, yaw;
    for (unsigned int i = 0; i < msg->objectList.size(); i++) {

        // If this object is not assigned we have to allocate data.
        lastConfigMutex_.lock();
        if (globalLastConfig_.find(msg->objectList[i].meEntity.id) == globalLastConfig_.end()) {
            curObject = new MovableObject(msg->objectList[i].meEntity.id);
            curObject->setRoomId(0);
            curObject->setName(msg->objectList[i].meEntity.name);
            increaseNbObjects();
        } else
            curObject = globalLastConfig_[msg->objectList[i].meEntity.id];
        lastConfigMutex_.unlock();

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

        lastConfigMutex_.lock();
        if (globalLastConfig_[msg->objectList[i].meEntity.id] == NULL)
            globalLastConfig_[curObject->getId()] = curObject;
        lastConfigMutex_.unlock();
        if (lastConfig_[msg->objectList[i].meEntity.id] == NULL)
            lastConfig_[curObject->getId()] = curObject;
    }
  }
}
