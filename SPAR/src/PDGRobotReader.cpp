#include "SPAR/PDGRobotReader.h"

PDGRobotReader::PDGRobotReader(ros::NodeHandle& node, bool fullRobot) {
    fullRobot_ = fullRobot;
    std::cout << "[SPAR] Initializing PDGRobotReader" << std::endl;

    // Starts listening to the topic
    sub_ = node.subscribe("/PDG/robotList", 1, &PDGRobotReader::robotJointStateCallBack, this);
}

void PDGRobotReader::robotJointStateCallBack(const PDG::RobotList::ConstPtr& msg) {
    //std::cout << "[SPAR][DEBUG] new data for robot received" << std::endl;

    Robot* curRobot;
    for (unsigned int i = 0; i < msg->robotList.size(); i++) {

        // If this robot is not assigned we have to allocate data.
        if (lastConfig_[msg->robotList[i].meAgent.meEntity.id] == NULL){
            curRobot = new Robot(msg->robotList[i].meAgent.meEntity.id);
            curRobot->setName(msg->robotList[i].meAgent.meEntity.name);
        }
        else
            curRobot = lastConfig_[msg->robotList[i].meAgent.meEntity.id];

        std::vector<double> robOrientation;
        bg::model::point<double, 3, bg::cs::cartesian> robPosition;

        Mobility curRobMobility = FULL;

        curRobot->setMobility(curRobMobility);
        curRobot->setTime(msg->robotList[i].meAgent.meEntity.time);

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
        }
    }
}

bool PDGRobotReader::isPresent(unsigned int id) {
    timeval curTime;
    gettimeofday(&curTime, NULL);
    unsigned long now = curTime.tv_sec * pow(10, 9) + curTime.tv_usec;
    unsigned long timeThreshold = pow(10, 9);
    //std::cout << "current time: " << now <<  "  human time: " << m_LastTime << std::endl;
    long timeDif = lastConfig_[id]->getTime() - now;
    //std::cout << "time dif: " << timeDif << std::endl;

    if (fabs(timeDif) < timeThreshold)
        return true;
    else
        return false;
}

