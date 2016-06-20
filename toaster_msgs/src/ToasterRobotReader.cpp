/* 
 * File:   ToasterObjectReader.cpp
 * Author: gmilliez
 * 
 * Created on December 12, 2014, 2:23 PM
 */

#include "toaster_msgs/ToasterRobotReader.h"
#include "tf/transform_datatypes.h"

ToasterRobotReader::ToasterRobotReader(ros::NodeHandle& node, bool fullRobot, std::string topic) {
    fullRobot_ = fullRobot;
    std::cout << "Initializing ToasterRobotReader" << std::endl;

    // Starts listening to the topic
    sub_ = node.subscribe(topic, 1, &ToasterRobotReader::robotJointStateCallBack, this);
}

void ToasterRobotReader::robotJointStateCallBack(const toaster_msgs::RobotListStamped::ConstPtr& msg) {
    //std::cout << "[area_manager][DEBUG] new data for robot received" << std::endl;

    double roll, pitch, yaw;
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

        robPosition.set<0>(msg->robotList[i].meAgent.meEntity.pose.position.x);
        robPosition.set<1>(msg->robotList[i].meAgent.meEntity.pose.position.y);
        robPosition.set<2>(msg->robotList[i].meAgent.meEntity.pose.position.z);
        curRobot->setPosition(robPosition);

        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->robotList[i].meAgent.meEntity.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        robOrientation.push_back(roll);
        robOrientation.push_back(pitch);
        robOrientation.push_back(yaw);
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

                jointPosition.set<0>(msg->robotList[i].meAgent.skeletonJoint[i_jnt].meEntity.pose.position.x);
                jointPosition.set<1>(msg->robotList[i].meAgent.skeletonJoint[i_jnt].meEntity.pose.position.y);
                jointPosition.set<2>(msg->robotList[i].meAgent.skeletonJoint[i_jnt].meEntity.pose.position.z);
                curJnt->setPosition(jointPosition);


                tf::Quaternion q;
                tf::quaternionMsgToTF(msg->robotList[i].meAgent.skeletonJoint[i_jnt].meEntity.pose.orientation, q);
                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

                jointOrientation.push_back(roll);
                jointOrientation.push_back(pitch);
                jointOrientation.push_back(yaw);
                curJnt->setOrientation(jointOrientation);

                curJnt->position = msg->robotList[i].meAgent.skeletonJoint[i_jnt].position;

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

