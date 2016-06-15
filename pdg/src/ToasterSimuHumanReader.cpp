/* 
 * File:   ToasterSimuHumanReader.cpp
 * Author: gmilliez
 * 
 * Created on February 3, 2016, 12:15 PM
 */

#include "pdg/ToasterSimuHumanReader.h"

ToasterSimuHumanReader::ToasterSimuHumanReader(ros::NodeHandle& node) {
    std::cout << "[area_manager] Initializing ToasterHumanReader" << std::endl;
    // Starts listening to the topic
    sub_ = node.subscribe("/toaster_simu/humanList", 1, &ToasterSimuHumanReader::humanJointStateCallBack, this);
}

void ToasterSimuHumanReader::humanJointStateCallBack(const toaster_msgs::HumanList::ConstPtr& msg) {
    //std::cout << "[area_manager][DEBUG] new data for human received with time " << msg->humanList[0].meAgent.meEntity.time  << std::endl;
    Human * curHuman;
    for (unsigned int i = 0; i < msg->humanList.size(); i++) {

        // If this human is not assigned we have to allocate data.
        if (lastConfig_.find(msg->humanList[i].meAgent.meEntity.id) == lastConfig_.end()) {
            curHuman = new Human(msg->humanList[i].meAgent.meEntity.id);
        } else
            curHuman = lastConfig_[msg->humanList[i].meAgent.meEntity.id];

        std::vector<double> humanOrientation;
        bg::model::point<double, 3, bg::cs::cartesian> humanPosition;

        Mobility curHumanMobility = FULL;
        curHuman->setId(msg->humanList[i].meAgent.meEntity.id);
        curHuman->setName(msg->humanList[i].meAgent.meEntity.name);

        curHuman->setMobility(curHumanMobility);
        curHuman->setTime(msg->humanList[i].meAgent.meEntity.time);
        curHuman->busyHands_ = msg->humanList[i].meAgent.busyHands;

        humanPosition.set<0>(msg->humanList[i].meAgent.meEntity.Pose.position.x);
        humanPosition.set<1>(msg->humanList[i].meAgent.meEntity.Pose.position.y);
        humanPosition.set<2>(msg->humanList[i].meAgent.meEntity.Pose.position.z);
        curHuman->setPosition(humanPosition);

        humanOrientation.push_back(msg->humanList[i].meAgent.meEntity.orientationRoll);
        humanOrientation.push_back(msg->humanList[i].meAgent.meEntity.orientationPitch);
        humanOrientation.push_back(msg->humanList[i].meAgent.meEntity.orientationYaw);
        curHuman->setOrientation(humanOrientation);

        lastConfig_[curHuman->getId()] = curHuman;

        //TODO: fullHuman
        if (msg->humanList[i].meAgent.skeletonJoint.size() > 0) {
            Joint * curJnt;
            for (unsigned int i_jnt = 0; i_jnt < msg->humanList[i].meAgent.skeletonJoint.size(); i_jnt++) {

                // If this joint is not assigned we have to allocate data.
                if (lastConfig_[curHuman->getId()]->skeleton_[msg->humanList[i].meAgent.skeletonJoint[i_jnt].meEntity.name ] == NULL) {
                    curJnt = new Joint(msg->humanList[i].meAgent.skeletonJoint[i_jnt].meEntity.id, msg->humanList[i].meAgent.meEntity.id);
                } else
                    curJnt = lastConfig_[curHuman->getId()]->skeleton_[msg->humanList[i].meAgent.skeletonJoint[i_jnt].meEntity.name ];

                std::vector<double> jointOrientation;
                bg::model::point<double, 3, bg::cs::cartesian> jointPosition;

                curJnt->setName(msg->humanList[i].meAgent.skeletonJoint[i_jnt].meEntity.name);
                curJnt->setAgentId(curHuman->getId());
                curJnt->setTime(msg->humanList[i].meAgent.skeletonJoint[i_jnt].meEntity.time);

                jointPosition.set<0>(msg->humanList[i].meAgent.skeletonJoint[i_jnt].meEntity.Pose.position.x);
                jointPosition.set<1>(msg->humanList[i].meAgent.skeletonJoint[i_jnt].meEntity.Pose.position.y);
                jointPosition.set<2>(msg->humanList[i].meAgent.skeletonJoint[i_jnt].meEntity.Pose.position.z);
                curJnt->setPosition(jointPosition);

                jointOrientation.push_back(msg->humanList[i].meAgent.skeletonJoint[i_jnt].meEntity.orientationRoll);
                jointOrientation.push_back(msg->humanList[i].meAgent.skeletonJoint[i_jnt].meEntity.orientationPitch);
                jointOrientation.push_back(msg->humanList[i].meAgent.skeletonJoint[i_jnt].meEntity.orientationYaw);
                curJnt->setOrientation(jointOrientation);

                curJnt->position = msg->humanList[i].meAgent.skeletonJoint[i_jnt].position;

                lastConfig_[curHuman->getId()]->skeleton_[curJnt->getName()] = curJnt;
            }
        }
    }
}