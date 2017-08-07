#include "pdg/utility/EntityUtility.h"

//tf
#include <tf/transform_broadcaster.h>

ros::ServiceClient* setPoseClient_;

void EntityUtility_setClient(ros::ServiceClient* m_setPoseClient)
{
  setPoseClient_ = m_setPoseClient;
}

void fillValue(MovableIoTObject* srcObject, toaster_msgs::Object& msgObject) {
    msgObject.value = srcObject->getValue();
}

void fillEntity(Entity* srcEntity, toaster_msgs::Entity& msgEntity) {
    msgEntity.id = srcEntity->getId();
    msgEntity.time = srcEntity->getTime();
    msgEntity.name = srcEntity->getName();
    msgEntity.pose.position.x = srcEntity->getPosition().get<0>();
    msgEntity.pose.position.y = srcEntity->getPosition().get<1>();
    msgEntity.pose.position.z = srcEntity->getPosition().get<2>();

    tf::Quaternion q;
    q.setRPY(srcEntity->getOrientation()[0], srcEntity->getOrientation()[1], srcEntity->getOrientation()[2]);

    msgEntity.pose.orientation.x = q[0];
    msgEntity.pose.orientation.y = q[1];
    msgEntity.pose.orientation.z = q[2];
    msgEntity.pose.orientation.w = q[3];
}

void updateEntity(Entity& newPoseEnt, Entity* storedEntity) {
    ROS_INFO("UPDATE entity");
    storedEntity->position_ = newPoseEnt.getPosition();
    storedEntity->orientation_ = newPoseEnt.getOrientation();
    storedEntity->setTime(newPoseEnt.getTime());
}

bool updateToasterSimu(Entity* storedEntity, std::string type) {
    toaster_msgs::SetEntityPose setPose;
    setPose.request.id = storedEntity->getId();
    setPose.request.type = type;
    setPose.request.pose.position.x = storedEntity->position_.get<0>();
    setPose.request.pose.position.y = storedEntity->position_.get<1>();
    setPose.request.pose.position.z = storedEntity->position_.get<2>();


    tf::Quaternion q;
    q.setRPY(storedEntity->getOrientation()[0], storedEntity->getOrientation()[1], storedEntity->getOrientation()[2]);

    setPose.request.pose.orientation.x = q[0];
    setPose.request.pose.orientation.y = q[1];
    setPose.request.pose.orientation.z = q[2];
    setPose.request.pose.orientation.w = q[3];

    if (setPoseClient_->call(setPose)) {
        ROS_DEBUG("[Request] we request to set Pose in toaster_simu: %s \n", storedEntity->getId().c_str());
        return true;
    } else {
        ROS_INFO("[Request] we failed to request to set Pose in toaster_simu: %s\n", storedEntity->getId().c_str());
        return false;
    }
}

bool putAtJointPosition(Entity* storedEntity, std::string agentId, std::string joint,
        toaster_msgs::HumanListStamped& humanList_msg, bool toastersimu) {

    toaster_msgs::Entity jointEntity;

    //  find back the agent:
    for (std::vector<toaster_msgs::Human>::iterator itAgent = humanList_msg.humanList.begin(); itAgent != humanList_msg.humanList.end(); ++itAgent) {
        if ((*itAgent).meAgent.meEntity.id == agentId) {

            //Find back the joint
            std::vector<std::string>::iterator it = std::find((*itAgent).meAgent.skeletonNames.begin(),
                    (*itAgent).meAgent.skeletonNames.end(), joint);

            if (it != (*itAgent).meAgent.skeletonNames.end()) {
                jointEntity = (*itAgent).meAgent.skeletonJoint[std::distance((*itAgent).meAgent.skeletonNames.begin(), it)].meEntity;

                storedEntity->position_.set<0>(jointEntity.pose.position.x);
                storedEntity->position_.set<1>(jointEntity.pose.position.y);
                storedEntity->position_.set<2>(jointEntity.pose.position.z);

                tf::Quaternion q(jointEntity.pose.orientation.x, jointEntity.pose.orientation.y, jointEntity.pose.orientation.z, jointEntity.pose.orientation.w);
                double roll, pitch, yaw;
                tf::Matrix3x3 m(q);
                m.getEulerYPR(yaw, pitch, roll);


                storedEntity->orientation_[0] = roll;
                storedEntity->orientation_[1] = pitch;
                storedEntity->orientation_[2] = yaw;

                storedEntity->setTime(jointEntity.time);

                (*itAgent).meAgent.hasObjects.push_back(storedEntity->getId());
                (*itAgent).meAgent.busyHands.push_back(jointEntity.id);

                if (toastersimu)
                    updateToasterSimu(storedEntity, "object");

                return true;
            } else
                ROS_WARN("Can't find joint %s for human %s. Couldn't attach object %s to this joint", joint.c_str(), agentId.c_str(), storedEntity->getId().c_str());
        }
    }
    return false;
}

bool putAtJointPosition(Entity* storedEntity, std::string agentId, std::string joint,
        toaster_msgs::RobotListStamped robotList_msg, bool toastersimu) {

    toaster_msgs::Entity jointEntity;

    //  find back the agent:
    for (std::vector<toaster_msgs::Robot>::iterator itAgent = robotList_msg.robotList.begin(); itAgent != robotList_msg.robotList.end(); ++itAgent) {
        if ((*itAgent).meAgent.meEntity.id == agentId) {

            //Find back the joint
            std::vector<std::string>::iterator it = std::find((*itAgent).meAgent.skeletonNames.begin(),
                    (*itAgent).meAgent.skeletonNames.end(), joint);

            if (it != (*itAgent).meAgent.skeletonNames.end()) {
                jointEntity = (*itAgent).meAgent.skeletonJoint[std::distance((*itAgent).meAgent.skeletonNames.begin(), it)].meEntity;

                storedEntity->position_.set<0>(jointEntity.pose.position.x);
                storedEntity->position_.set<1>(jointEntity.pose.position.y);
                storedEntity->position_.set<2>(jointEntity.pose.position.z);

                storedEntity->setTime(jointEntity.time);

                tf::Quaternion q(jointEntity.pose.orientation.x, jointEntity.pose.orientation.y, jointEntity.pose.orientation.z, jointEntity.pose.orientation.w);
                double roll, pitch, yaw;
                tf::Matrix3x3 m(q);
                m.getEulerYPR(yaw, pitch, roll);


                storedEntity->orientation_[0] = roll;
                storedEntity->orientation_[1] = pitch;
                storedEntity->orientation_[2] = yaw;

                (*itAgent).meAgent.hasObjects.push_back(storedEntity->getId());
                (*itAgent).meAgent.busyHands.push_back(jointEntity.id);

                if (toastersimu)
                    updateToasterSimu(storedEntity, "object");

                return true;
            } else
                ROS_WARN("Can't find joint %s for robot %s. Couldn't attach object %s to this joint", joint.c_str(), agentId.c_str(), storedEntity->getId().c_str());
        }
    }
    return false;
}
