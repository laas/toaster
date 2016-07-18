/* 
 * File:   AdreamMocapHumanReader.cpp
 * Author: sdevin
 * 
 * Created on October 8, 2015, 1:24 PM
 */

#include "pdg/AdreamMocapHumanReader.h"

// A human reader is a class that will read data from human(s)

AdreamMocapHumanReader::AdreamMocapHumanReader(ros::NodeHandle& node, std::string topicTorso = "/optitrack/bodies/Rigid_Body_3",
        std::string topicHead = "/optitrack/bodies/Rigid_Body_1", std::string topicHand = "/optitrack/bodies/Rigid_Body_1") {
    std::cout << "Initializing AdreamMocapHumanReader" << std::endl;
    // ******************************************
    // Starts listening to the joint_states
    fullHuman_ = false;
    subTorso_ = node.subscribe(topicTorso, 1, &AdreamMocapHumanReader::optitrackCallbackHand, this);
    subHead_ = node.subscribe(topicHead, 1, &AdreamMocapHumanReader::optitrackCallbackHead, this);
    subHand_ = node.subscribe(topicHand, 1, &AdreamMocapHumanReader::optitrackCallbackHand, this);
    std::cout << "Done\n";
}


void AdreamMocapHumanReader::optitrackCallbackTorso(const optitrack::or_pose_estimator_state::ConstPtr& msg) {

    ros::Time now = ros::Time::now();
    Human* curHuman;
    Joint* curJoint;

    try {
        //Should we keep this "HERAKLES_HUMAN1"?
        //Mb use something more generic...
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
            humanPosition.set<2>(msg->pos[0].z - 1.18);

            //set the human orientation
            std::vector<double> humanOrientation;

            //transform the pose message

            tf::Quaternion q(msg->pos[0].qx, msg->pos[0].qy, msg->pos[0].qz, msg->pos[0].qw);
            double roll, pitch, yaw;
            tf::Matrix3x3 m(q);
            m.getEulerYPR(yaw, pitch, roll);

            humanOrientation.push_back(roll);
            humanOrientation.push_back(pitch);
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

            //update the torso
            std::string jointNameTorsoe = "torso";

            if (curHuman->skeleton_.find(jointNameTorsoe) == curHuman->skeleton_.end()) {
                curJoint = new Joint(jointNameTorsoe, humId);
                curJoint->setName(jointNameTorsoe);
            } else {
                curJoint = curHuman->skeleton_[jointNameTorsoe];
            }

            bg::model::point<double, 3, bg::cs::cartesian> torsoPosition;
            torsoPosition.set<0>(msg->pos[0].x + 6.407);
            torsoPosition.set<1>(msg->pos[0].y + 2.972);
            torsoPosition.set<2>(msg->pos[0].z);
            curJoint->setPosition(torsoPosition);
            curJoint->setOrientation(humanOrientation);
            curJoint->setTime(now.toNSec());

            lastConfig_[humId]->skeleton_[jointNameTorsoe] = curJoint;

        }


    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
}

void AdreamMocapHumanReader::optitrackCallbackHead(const optitrack::or_pose_estimator_state::ConstPtr& msg) {

    ros::Time now = ros::Time::now();
    Human* curHuman;
    Joint* curJoint;

    try {

        std::string humId = "HERAKLES_HUMAN1";
        std::string jointName = "head";

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
            m.getEulerYPR(yaw, pitch, roll);

            jointOrientation.push_back(roll);
            jointOrientation.push_back(pitch);
            jointOrientation.push_back(yaw);


            curJoint->setPosition(jointPosition);
            curJoint->setOrientation(jointOrientation);
            curJoint->setTime(now.toNSec());

            lastConfig_[humId]->skeleton_[jointName] = curJoint;
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
            m.getEulerYPR(yaw, pitch, roll);

            jointOrientation.push_back(roll);
            jointOrientation.push_back(pitch);
            jointOrientation.push_back(yaw);


            curJoint->setPosition(jointPosition);
            curJoint->setOrientation(jointOrientation);
            curJoint->setTime(now.toNSec());

            lastConfig_[humId]->skeleton_[jointName] = curJoint;
        }

    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());

    }
}
