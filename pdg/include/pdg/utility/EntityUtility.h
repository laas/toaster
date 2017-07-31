#include <string>
#include "ros/ros.h"

#include <toaster_msgs/RobotListStamped.h>
#include <toaster_msgs/HumanListStamped.h>
#include <toaster_msgs/ObjectListStamped.h>
#include <toaster_msgs/SetEntityPose.h>

#include "toaster-lib/MovableIoTObject.h"

#ifndef ENTITYUTILITY_H
#define ENTITYUTILITY_H

using namespace std;

// Service client
void EntityUtility_setClient(ros::ServiceClient* m_setPoseClient);

void fillValue(MovableIoTObject* srcObject, toaster_msgs::Object& msgObject);

void fillEntity(Entity* srcEntity, toaster_msgs::Entity& msgEntity);

void updateEntity(Entity& newPoseEnt, Entity* storedEntity);

bool updateToasterSimu(Entity* storedEntity, string type);

bool putAtJointPosition(Entity* storedEntity, string agentId, string joint,
        toaster_msgs::HumanListStamped& humanList_msg, bool toastersimu);

bool putAtJointPosition(Entity* storedEntity, string agentId, string joint,
        toaster_msgs::RobotListStamped robotList_msg, bool toastersimu);

#endif
