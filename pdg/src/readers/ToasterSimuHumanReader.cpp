/*
 * File:   ToasterSimuHumanReader.cpp
 * Author: gmilliez
 *
 * Created on February 3, 2016, 12:15 PM
 */

#include "pdg/readers/ToasterSimuHumanReader.h"
#include "tf/transform_datatypes.h"

ToasterSimuHumanReader::ToasterSimuHumanReader(bool fullHuman) : HumanReader()
{
  fullHuman_ = fullHuman;
}

void ToasterSimuHumanReader::init(ros::NodeHandle* node, std::string param)
{
  std::cout << "[PDG] Initializing ToasterHumanReader" << std::endl;
  Reader<Human>::init(node, param);
  // ******************************************
  // Starts listening to the joint_states
  sub_ = node_->subscribe("/toaster_simu/humanList", 1, &ToasterSimuHumanReader::humanJointStateCallBack, this);
}

void ToasterSimuHumanReader::Publish(struct toasterList_t& list_msg)
{
  if(activated_)
  {
    for (std::map<std::string, Human*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it) {
        //Human
        toaster_msgs::Human human_msg;
        fillEntity(it->second, human_msg.meAgent.meEntity);

        human_msg.meAgent.skeletonJoint.clear();
        human_msg.meAgent.skeletonNames.clear();

        for (std::map<std::string, Joint*>::iterator itJoint = lastConfig_[it->first]->skeleton_.begin(); itJoint != lastConfig_[it->first]->skeleton_.end(); ++itJoint) {
            toaster_msgs::Joint joint_msg;
            human_msg.meAgent.skeletonNames.push_back(itJoint->first);
            fillEntity((itJoint->second), joint_msg.meEntity);
            joint_msg.jointOwner = it->first;

            human_msg.meAgent.skeletonJoint.push_back(joint_msg);
        }
        list_msg.human_msg.humanList.push_back(human_msg);
    }
  }
}

void ToasterSimuHumanReader::humanJointStateCallBack(const toaster_msgs::HumanListStamped::ConstPtr& msg) {
    //std::cout << "[area_manager][DEBUG] new data for human received with time " << msg->humanList[0].meAgent.meEntity.time  << std::endl;
    Human * curHuman;
    double roll, pitch, yaw;
    for (unsigned int i = 0; i < msg->humanList.size(); i++)
    {
        auto agent = msg->humanList[i].meAgent;

        // If this human is not assigned we have to allocate data.
        if (lastConfig_.find(agent.meEntity.id) == lastConfig_.end())
          curHuman = new Human(agent.meEntity.id);
        else
          curHuman = lastConfig_[agent.meEntity.id];

        std::vector<double> humanOrientation;
        bg::model::point<double, 3, bg::cs::cartesian> humanPosition;

        Mobility curHumanMobility = FULL;
        curHuman->setId(agent.meEntity.id);
        curHuman->setName(agent.meEntity.name);

        curHuman->setMobility(curHumanMobility);
        curHuman->setTime(agent.meEntity.time);
        curHuman->busyHands_ = agent.busyHands;

        humanPosition.set<0>(agent.meEntity.pose.position.x);
        humanPosition.set<1>(agent.meEntity.pose.position.y);
        humanPosition.set<2>(agent.meEntity.pose.position.z);
        curHuman->setPosition(humanPosition);

        tf::Quaternion q;

        tf::quaternionMsgToTF(agent.meEntity.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        humanOrientation.push_back(roll);
        humanOrientation.push_back(pitch);
        humanOrientation.push_back(yaw);
        curHuman->setOrientation(humanOrientation);

        lastConfig_[curHuman->getId()] = curHuman;
        lastConfig_[curHuman->getId()]->skeleton_.clear();

        //TODO: fullHuman
        if (agent.skeletonJoint.size() > 0)
        {
            Joint * curJnt;
            for (unsigned int i_jnt = 0; i_jnt < agent.skeletonJoint.size(); i_jnt++)
            {
                auto skeletonJoint = agent.skeletonJoint[i_jnt];
                // If this joint is not assigned we have to allocate data.
                if (lastConfig_[curHuman->getId()]->skeleton_[skeletonJoint.meEntity.name ] == NULL) {
                    curJnt = new Joint(skeletonJoint.meEntity.id, agent.meEntity.id);
                } else
                    curJnt = lastConfig_[curHuman->getId()]->skeleton_[skeletonJoint.meEntity.name ];

                std::vector<double> jointOrientation;
                bg::model::point<double, 3, bg::cs::cartesian> jointPosition;

                curJnt->setName(skeletonJoint.meEntity.name);
                curJnt->setAgentId(curHuman->getId());
                curJnt->setTime(skeletonJoint.meEntity.time);

                jointPosition.set<0>(skeletonJoint.meEntity.pose.position.x);
                jointPosition.set<1>(skeletonJoint.meEntity.pose.position.y);
                jointPosition.set<2>(skeletonJoint.meEntity.pose.position.z);
                curJnt->setPosition(jointPosition);


                tf::Quaternion q;

                tf::quaternionMsgToTF(skeletonJoint.meEntity.pose.orientation, q);
                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

                jointOrientation.push_back(roll);
                jointOrientation.push_back(pitch);
                jointOrientation.push_back(yaw);
                curJnt->setOrientation(jointOrientation);

                curJnt->position = skeletonJoint.position;

                lastConfig_[curHuman->getId()]->skeleton_[curJnt->getName()] = curJnt;
            }
        }
    }
}
