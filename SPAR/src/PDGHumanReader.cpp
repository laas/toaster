#include "SPAR/PDGHumanReader.h"

PDGHumanReader::PDGHumanReader(ros::NodeHandle& node, bool fullHuman) {
    fullHuman_ = fullHuman;
    std::cout << "[SPAR] Initializing PDGHumanReader" << std::endl;

    // Starts listening to the topic
    sub_ = node.subscribe("/humanList", 1, &PDGHumanReader::humanJointStateCallBack, this);
}

void PDGHumanReader::humanJointStateCallBack(const PDG::HumanList::ConstPtr& msg) {
    std::cout << "[SPAR] new data for human received" << std::endl;
    Human * curHuman;
    for (unsigned int i = 0; i < msg->humanList.size(); i++) {
        
        // If this human is not assigned we have to allocate data.
        if (lastConfig_[msg->humanList[i].meAgent.meEntity.id] == NULL)
            curHuman = new Human(msg->humanList[i].meAgent.meEntity.id);
        else
            curHuman = lastConfig_[msg->humanList[i].meAgent.meEntity.id];

        std::vector<double> humanOrientation;
        bg::model::point<double, 3, bg::cs::cartesian> humanPosition;

        Mobility curHumanMobility = FULL;

        curHuman->setMobility(curHumanMobility);
        curHuman->setName("Human1");
        curHuman->setTime(msg->humanList[i].meAgent.meEntity.time);

        humanPosition.set<0>(msg->humanList[i].meAgent.meEntity.positionX);
        humanPosition.set<1>(msg->humanList[i].meAgent.meEntity.positionY);
        humanPosition.set<2>(msg->humanList[i].meAgent.meEntity.positionZ);
        curHuman->setPosition(humanPosition);

        humanOrientation.push_back(msg->humanList[i].meAgent.meEntity.orientationRoll);
        humanOrientation.push_back(msg->humanList[i].meAgent.meEntity.orientationPitch);
        humanOrientation.push_back(msg->humanList[i].meAgent.meEntity.orientationYaw);
        curHuman->setOrientation(humanOrientation);

        lastConfig_[curHuman->getId()] = curHuman;

        //TODO: fullHuman
        if (fullHuman_) {
        }
    }
}

bool PDGHumanReader::isPresent(unsigned int id) {
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

