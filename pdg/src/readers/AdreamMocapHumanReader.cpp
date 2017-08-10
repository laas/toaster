/*
 * File:   AdreamMocapHumanReader.cpp
 * Author: sdevin
 *
 * Created on October 8, 2015, 1:24 PM
 */

#include "pdg/readers/AdreamMocapHumanReader.h"

#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"
#include <sys/time.h>
#include <math.h>
#include <ostream>

// A human reader is a class that will read data from human(s)
void AdreamMocapHumanReader::init(ros::NodeHandle* node, std::string topicTorso, std::string topicHead, std::string topicHand, std::string param)
{
  std::cout << "[PDG] Initializing AdreamMocapHumanReader" << std::endl;
  Reader<Human>::init(node, param);
  torso_ = false;
  // ******************************************
  // Starts listening to the joint_states
  subTorso_ = node_->subscribe(topicTorso, 1, &AdreamMocapHumanReader::optitrackCallbackTorso, this);
  subHead_ = node_->subscribe(topicHead, 1, &AdreamMocapHumanReader::optitrackCallbackHead, this);
  subHand_ = node_->subscribe(topicHand, 1, &AdreamMocapHumanReader::optitrackCallbackHand, this);
  std::cout << "Done\n";
}

void AdreamMocapHumanReader::Publish(struct toasterList_t& list_msg)
{
  if(activated_)
  {
    for (std::map<std::string, Human*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it) {
        if (isPresent(it->first))
        {
            toaster_msgs::Fact fact_msg = DefaultFactMsg(it->first, it->second->getTime());
            list_msg.fact_msg.factList.push_back(fact_msg);

            //Human
            toaster_msgs::Human human_msg;
            fillEntity(it->second, human_msg.meAgent.meEntity);

            //if (humanFullConfig_) {
            for (std::map<std::string, Joint*>::iterator itJoint = lastConfig_[it->first]->skeleton_.begin(); itJoint != lastConfig_[it->first]->skeleton_.end(); ++itJoint) {
                toaster_msgs::Joint joint_msg;
                human_msg.meAgent.skeletonNames.push_back(itJoint->first);
                fillEntity((itJoint->second), joint_msg.meEntity);
                joint_msg.jointOwner = it->first;

                human_msg.meAgent.skeletonJoint.push_back(joint_msg);

            }
            //}
            list_msg.human_msg.humanList.push_back(human_msg);
        }
    }
  }
}

void AdreamMocapHumanReader::optitrackCallbackHead(const optitrack::or_pose_estimator_state::ConstPtr & msg) {

    ros::Time now = ros::Time::now();
    Human* curHuman;
    Joint* curJoint;

    if (ros::param::get("mocap_calib_world_x",offset_x))
    {
        if (ros::param::get("mocap_calib_world_y",offset_y));
            if (ros::param::get("mocap_calib_world_z",offset_z));
    } else {
        offset_x=0;
        offset_y=0;
        offset_z=0;
    }

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

            tf::Quaternion q(msg->pos[0].qx, msg->pos[0].qy, msg->pos[0].qz, msg->pos[0].qw);
            double roll, pitch, yaw;
            tf::Matrix3x3 m(q);
            m.getEulerYPR(yaw, pitch, roll);

            if (!torso_) {
                //set human position
                bg::model::point<double, 3, bg::cs::cartesian> humanPosition;
                humanPosition.set<0>(msg->pos[0].x + offset_x);
                humanPosition.set<1>(msg->pos[0].y + offset_y);
                humanPosition.set<2>(msg->pos[0].z + offset_z - 0.17);

                //set the human orientation
                std::vector<double> humanOrientation;

                //transform the pose message
                humanOrientation.push_back(0.0);
                humanOrientation.push_back(0.0);


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
            }

            //update the head
            std::string jointNameHead = "head";

            if (curHuman->skeleton_.find(jointNameHead) == curHuman->skeleton_.end()) {
                curJoint = new Joint(jointNameHead, humId);
                curJoint->setName(jointNameHead);
            } else {
                curJoint = curHuman->skeleton_[jointNameHead];
            }

            bg::model::point<double, 3, bg::cs::cartesian> headPosition;
            headPosition.set<0>(msg->pos[0].x + offset_x);
            headPosition.set<1>(msg->pos[0].y + offset_y);
            headPosition.set<2>(msg->pos[0].z + offset_z);
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

void AdreamMocapHumanReader::optitrackCallbackHand(const optitrack::or_pose_estimator_state::ConstPtr & msg) {

    ros::Time now = ros::Time::now();
    Human* curHuman;
    Joint* curJoint;

    if (ros::param::get("mocap_calib_world_x",offset_x))
    {
        if (ros::param::get("mocap_calib_world_y",offset_y));
            if (ros::param::get("mocap_calib_world_z",offset_z));
    } else {
        offset_x=0;
        offset_y=0;
        offset_z=0;
    }

    try {

        std::string humId = "HERAKLES_HUMAN1";
        std::string jointName = "rightHand";

        if (lastConfig_.find(humId) == lastConfig_.end()) {
            // We wait that head is detected so that the human is created
            return;
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
            jointPosition.set<0>(msg->pos[0].x + offset_x);
            jointPosition.set<1>(msg->pos[0].y + offset_y);
            jointPosition.set<2>(msg->pos[0].z + offset_z);

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

void AdreamMocapHumanReader::optitrackCallbackTorso(const optitrack::or_pose_estimator_state::ConstPtr & msg) {

    ros::Time now = ros::Time::now();
    Human* curHuman;
    Joint* curJoint;
    Joint* jointBase;
    if (ros::param::get("mocap_calib_world_x",offset_x))
    {
        if (ros::param::get("mocap_calib_world_y",offset_y));
            if (ros::param::get("mocap_calib_world_z",offset_z));
    } else {
        offset_x=0;
        offset_y=0;
        offset_z=0;
    }
    try {

        std::string humId = "HERAKLES_HUMAN1";
        std::string jointName = "torso";

        if (lastConfig_.find(humId) == lastConfig_.end()) {
            // We wait that head is detected so that the human is created
            return;
        } else {
            curHuman = lastConfig_[humId];
            if (!torso_)
                torso_ = true;
        }

        if (curHuman->skeleton_.find(jointName) == curHuman->skeleton_.end()) {
            curJoint = new Joint(jointName, humId);
            curJoint->setName(jointName);
        } else {
            curJoint = curHuman->skeleton_[jointName];
        }

        if (msg->pos.size() != 0) {


            //set human position
            bg::model::point<double, 3, bg::cs::cartesian> humanPosition;
            humanPosition.set<0>(msg->pos[0].x + offset_x);
            humanPosition.set<1>(msg->pos[0].y + offset_y);
            humanPosition.set<2>(msg->pos[0].z + offset_z);

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
            std::string jointBaseName = "base";
            std::vector<double> baseOrientation;

            baseOrientation.push_back(0.0);
            baseOrientation.push_back(0.0);
            baseOrientation.push_back(yaw);


            if (curHuman->skeleton_.find(jointBaseName) == curHuman->skeleton_.end()) {
                jointBase = new Joint(jointBaseName, humId);
                jointBase->setName(jointBaseName);
            } else {
                jointBase = curHuman->skeleton_[jointBaseName];
            }

            jointBase->setPosition(humanPosition);
            jointBase->setOrientation(baseOrientation);
            jointBase->setTime(now.toNSec());

            lastConfig_[humId]->skeleton_[jointBaseName] = jointBase;



            bg::model::point<double, 3, bg::cs::cartesian> jointPosition;
            jointPosition.set<0>(msg->pos[0].x + offset_x);
            jointPosition.set<1>(msg->pos[0].y + offset_y);
            jointPosition.set<2>(msg->pos[0].z + offset_z);



            curJoint->setPosition(jointPosition);
            curJoint->setOrientation(humanOrientation);
            curJoint->setTime(now.toNSec());

            lastConfig_[humId]->skeleton_[jointName] = curJoint;
        }

    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());

    }
}
