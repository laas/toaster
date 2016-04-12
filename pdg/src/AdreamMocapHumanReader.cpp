/* 
 * File:   AdreamMocapHumanReader.cpp
 * Author: sdevin
 * 
 * Created on October 8, 2015, 1:24 PM
 */

#include "pdg/AdreamMocapHumanReader.h"

// A human reader is a class that will read data from human(s)

AdreamMocapHumanReader::AdreamMocapHumanReader(ros::NodeHandle& node, std::string topicHead, std::string topicHand) {
    std::cout << "Initializing AdreamMocapHumanReader" << std::endl;
    // ******************************************
    // Starts listening to the joint_states
    fullHuman_ = false;
    subHead_ = node.subscribe(topicHead, 1, &AdreamMocapHumanReader::optitrackCallbackHead, this);
    subHand_ = node.subscribe(topicHand, 1, &AdreamMocapHumanReader::optitrackCallbackHand, this);
    std::cout << "Done\n";
}

/*
  Gets data from a TrackedPersons msg in the human map. This msg contains a list of agens with
  their positions and orientations.
 */
void AdreamMocapHumanReader::optitrackCallbackHead(const optitrack::or_pose_estimator_state::ConstPtr& msg) {

    ros::Time now = ros::Time::now();
    Human* curHuman;
    Joint* curJoint;

    try {
        std::string humId = "HERAKLES_HUMAN1";
        //create a new human with the same id as the message
        if (lastConfig_.find(humId) == lastConfig_.end()) {
            curHuman = new Human(humId);
            curHuman->setName(humId);
        } else {
            curHuman = lastConfig_[humId];
        }

        if (msg->pos.size() != 0) {
            //set human position
            bg::model::point<double, 3, bg::cs::cartesian> humanPosition;
            humanPosition.set<0>(msg->pos[0].x + 6.407);
            humanPosition.set<1>(msg->pos[0].y + 2.972);
            humanPosition.set<2>(msg->pos[0].z - 1.6);

            //set the human orientation
            std::vector<double> humanOrientation;

            //transform the pose message
            humanOrientation.push_back(0.0);
            humanOrientation.push_back(0.0);

            tf::Quaternion q(msg->pos[0].qx, msg->pos[0].qy, msg->pos[0].qz, msg->pos[0].qw);
            double roll, pitch, yaw;
           tf::Matrix3x3 m(q);
           m.getEulerYPR(yaw,pitch,roll);

            humanOrientation.push_back(yaw);

            //put the data in the human
            curHuman->setOrientation(humanOrientation);
            curHuman->setPosition(humanPosition);
            curHuman->setTime(now.toNSec());

            lastConfig_[humId] = curHuman;

            //update the base
            std::string jointName = "base";

            if (curHuman->skeleton_.find(jointName) == curHuman->skeleton_.end()) {
                curJoint = new Joint(jointName, humId);
                curJoint->setName(jointName);
            } else {
                curJoint = curHuman->skeleton_[jointName];
            }

            curJoint->setPosition(humanPosition);
            curJoint->setOrientation(humanOrientation);
            curJoint->setTime(now.toNSec());

            lastConfig_[humId]->skeleton_[jointName] = curJoint;
            
            //update the head
            std::string jointNameHead = "head";

            if (curHuman->skeleton_.find(jointNameHead) == curHuman->skeleton_.end()) {
                curJoint = new Joint(jointNameHead, humId);
                curJoint->setName(jointNameHead);
            } else {
                curJoint = curHuman->skeleton_[jointNameHead];
            }
           
            bg::model::point<double, 3, bg::cs::cartesian> headPosition;
            headPosition.set<0>(msg->pos[0].x + 6.407);
            headPosition.set<1>(msg->pos[0].y + 2.972);
            headPosition.set<2>(msg->pos[0].z);
            std::vector<double> headOrientation;
            headOrientation.push_back(roll);
            headOrientation.push_back(pitch);
            headOrientation.push_back(yaw);

            curJoint->setPosition(headPosition);
            curJoint->setOrientation(headOrientation);
            curJoint->setTime(now.toNSec());

            lastConfig_[humId]->skeleton_[jointNameHead] = curJoint;
            
        }


    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
}

void AdreamMocapHumanReader::optitrackCallbackHand(const optitrack::or_pose_estimator_state::ConstPtr& msg) {

    ros::Time now = ros::Time::now();
    Human* curHuman;
    Joint* curJoint;

    try {

        std::string humId = "HERAKLES_HUMAN1";
        std::string jointName = "rightHand";

        if (lastConfig_.find(humId) == lastConfig_.end()) {
            curHuman = new Human(humId);
            curHuman->setName(humId);
        } else {
            curHuman = lastConfig_[humId];
        }

        if (curHuman->skeleton_.find(jointName) == curHuman->skeleton_.end()) {
            curJoint = new Joint(jointName, humId);
            curJoint->setName(jointName);
        } else {
            curJoint = curHuman->skeleton_[jointName];
        }

        if (msg->pos.size() != 0) {
            bg::model::point<double, 3, bg::cs::cartesian> jointPosition;
            jointPosition.set<0>(msg->pos[0].x + 6.407);
            jointPosition.set<1>(msg->pos[0].y + 2.972);
            jointPosition.set<2>(msg->pos[0].z);

            std::vector<double> jointOrientation;


           tf::Quaternion q(msg->pos[0].qx, msg->pos[0].qy, msg->pos[0].qz, msg->pos[0].qw);
           double roll, pitch, yaw;
           tf::Matrix3x3 m(q);
           m.getEulerYPR(yaw,pitch,roll);

           jointOrientation.push_back(roll+1.57);
           jointOrientation.push_back(pitch+1.57);
           jointOrientation.push_back(yaw+3.14);


           curJoint->setPosition(jointPosition);
           curJoint->setOrientation(jointOrientation);
           curJoint->setTime(now.toNSec());

           lastConfig_[humId]->skeleton_[jointName] = curJoint;
        }

    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());

    }
}
