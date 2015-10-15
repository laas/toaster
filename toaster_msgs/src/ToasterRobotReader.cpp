#include "toaster_msgs/ToasterRobotReader.h"

ToasterRobotReader::ToasterRobotReader(ros::NodeHandle& node, bool fullRobot) {
    fullRobot_ = fullRobot;
    std::cout << "[area_manager] Initializing ToasterRobotReader" << std::endl;

    // Starts listening to the topic
    sub_ = node.subscribe("/pdg/robotList", 1, &ToasterRobotReader::robotJointStateCallBack, this);
}

void ToasterRobotReader::robotJointStateCallBack(const toaster_msgs::RobotList::ConstPtr& msg) {
    //std::cout << "[area_manager][DEBUG] new data for robot received" << std::endl;

    Robot* curRobot;
    for (unsigned int i = 0; i < msg->robotList.size(); i++) {

        // If this robot is not assigned we have to allocate data.
        if (lastConfig_.find(msg->robotList[i].meAgent.meEntity.id) == lastConfig_.end()) {
            curRobot = new Robot(msg->robotList[i].meAgent.meEntity.id);
        } else
            curRobot = lastConfig_[msg->robotList[i].meAgent.meEntity.id];

        std::vector<double> robOrientation;
        bg::model::point<double, 3, bg::cs::cartesian> robPosition;

        Mobility curRobMobility = FULL;
        curRobot->setId(msg->robotList[i].meAgent.meEntity.id);
        curRobot->setName(msg->robotList[i].meAgent.meEntity.name);

        curRobot->setMobility(curRobMobility);
        curRobot->setTime(msg->robotList[i].meAgent.meEntity.time);
        curRobot->busyHands_ = msg->robotList[i].meAgent.busyHands;

        robPosition.set<0>(msg->robotList[i].meAgent.meEntity.positionX);
        robPosition.set<1>(msg->robotList[i].meAgent.meEntity.positionY);
        robPosition.set<2>(msg->robotList[i].meAgent.meEntity.positionZ);
        curRobot->setPosition(robPosition);

        robOrientation.push_back(msg->robotList[i].meAgent.meEntity.orientationRoll);
        robOrientation.push_back(msg->robotList[i].meAgent.meEntity.orientationPitch);
        robOrientation.push_back(msg->robotList[i].meAgent.meEntity.orientationYaw);
        curRobot->setOrientation(robOrientation);

        lastConfig_[curRobot->getId()] = curRobot;

        //TODO: fullRobot case
        if (fullRobot_) {
            Joint * curJnt;
            for (unsigned int i_jnt = 0; i_jnt < msg->robotList[i].meAgent.skeletonJoint.size(); i_jnt++) {

                // If this joint is not assigned we have to allocate data.
                if (lastConfig_[curRobot->getId()]->skeleton_[msg->robotList[i].meAgent.skeletonJoint[i_jnt].meEntity.name ] == NULL) {
                    curJnt = new Joint(msg->robotList[i].meAgent.skeletonJoint[i_jnt].meEntity.id, msg->robotList[i].meAgent.meEntity.id);
                } else
                    curJnt = lastConfig_[curRobot->getId()]->skeleton_[msg->robotList[i].meAgent.skeletonJoint[i_jnt].meEntity.name ];

                std::vector<double> jointOrientation;
                bg::model::point<double, 3, bg::cs::cartesian> jointPosition;

                curJnt->setName(msg->robotList[i].meAgent.skeletonJoint[i_jnt].meEntity.name);
                curJnt->setAgentId(curRobot->getId());
                curJnt->setTime(msg->robotList[i].meAgent.skeletonJoint[i_jnt].meEntity.time);

                jointPosition.set<0>(msg->robotList[i].meAgent.skeletonJoint[i_jnt].meEntity.positionX);
                jointPosition.set<1>(msg->robotList[i].meAgent.skeletonJoint[i_jnt].meEntity.positionY);
                jointPosition.set<2>(msg->robotList[i].meAgent.skeletonJoint[i_jnt].meEntity.positionZ);
                curJnt->setPosition(jointPosition);

                jointOrientation.push_back(msg->robotList[i].meAgent.skeletonJoint[i_jnt].meEntity.orientationRoll);
                jointOrientation.push_back(msg->robotList[i].meAgent.skeletonJoint[i_jnt].meEntity.orientationPitch);
                jointOrientation.push_back(msg->robotList[i].meAgent.skeletonJoint[i_jnt].meEntity.orientationYaw);
                curJnt->setOrientation(jointOrientation);

                lastConfig_[curRobot->getId()]->skeleton_[curJnt->getName()] = curJnt;
            }
        }
    }
}

bool ToasterRobotReader::isPresent(std::string id) {
    timeval curTime;
    gettimeofday(&curTime, NULL);
    unsigned long now = curTime.tv_sec * pow(10, 9) + curTime.tv_usec;
    unsigned long timeThreshold = pow(10, 9);
    //std::cout << "current time: " << now <<  "  robot time: " << m_LastTime << std::endl;
    long timeDif = lastConfig_[id]->getTime() - now;
    //std::cout << "time dif: " << timeDif << std::endl;

    if (fabs(timeDif) < timeThreshold)
        return true;
    else
        return false;
}

