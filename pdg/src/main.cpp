
// Humans
#include "pdg/MorseHumanReader.h"
#include "pdg/NiutHumanReader.h"
#include "pdg/GroupHumanReader.h"
#include "pdg/MocapHumanReader.h"

// Robots
#include "pdg/Pr2RobotReader.h"
#include "pdg/SpencerRobotReader.h"

// Objects

// Facts


// Message generated class
#include <toaster_msgs/Entity.h>
#include <toaster_msgs/Agent.h>
#include <toaster_msgs/Joint.h>
#include <toaster_msgs/Robot.h>
#include <toaster_msgs/Human.h>
#include <toaster_msgs/Object.h>
#include <toaster_msgs/Fact.h>
#include <toaster_msgs/FactList.h>
#include <toaster_msgs/RobotList.h>
#include <toaster_msgs/HumanList.h>
#include <toaster_msgs/ObjectList.h>
#include <toaster_msgs/AddStream.h>
#include <toaster_msgs/PutInHand.h>
#include <toaster_msgs/RemoveFromHand.h>

bool humanFullConfig_ = false; //If false we will use only position and orientation
bool robotFullConfig_ = false; //If false we will use only position and orientation

// Stream to activate
bool morseHuman_ = false;
bool niutHuman_ = false;
bool groupHuman_ = false;
bool mocapHuman_ = false;

bool humanCentroid_ =false;

bool pr2Robot_ = false;
bool spencerRobot_ = false;

bool vimanObject_ = false;
bool sparkObject_ = false;
bool sparkFact_ = false;


std::map<std::string, unsigned int> entNameId_;
std::map<unsigned int, unsigned int> objectInAgent_;
std::map<unsigned int, std::string> objectInHand_;

void fillEntity(Entity* srcEntity, toaster_msgs::Entity& msgEntity) {
    msgEntity.id = srcEntity->getId();
    msgEntity.time = srcEntity->getTime();
    msgEntity.name = srcEntity->getName();
    msgEntity.positionX = srcEntity->getPosition().get<0>();
    msgEntity.positionY = srcEntity->getPosition().get<1>();
    msgEntity.positionZ = srcEntity->getPosition().get<2>();
    msgEntity.orientationRoll = srcEntity->getOrientation()[0];
    msgEntity.orientationPitch = srcEntity->getOrientation()[1];
    msgEntity.orientationYaw = srcEntity->getOrientation()[2];
}

bool putAtJointPosition(toaster_msgs::Entity& msgEntity, unsigned int id, std::string joint,
        toaster_msgs::HumanList& humanList_msg) {

    toaster_msgs::Entity jointEntity;

    std::vector<std::string>::iterator it = std::find(humanList_msg.humanList[id].meAgent.skeletonNames.begin(), humanList_msg.humanList[id].meAgent.skeletonNames.end(), joint);
    if (it != humanList_msg.humanList[id].meAgent.skeletonNames.end()) {
        jointEntity = humanList_msg.humanList[id].meAgent.skeletonJoint[std::distance(humanList_msg.humanList[id].meAgent.skeletonNames.begin(), it)].meEntity;

        msgEntity.positionX = jointEntity.positionX;
        msgEntity.positionX = jointEntity.positionY;
        msgEntity.positionX = jointEntity.positionZ;
        msgEntity.orientationRoll = jointEntity.orientationRoll;
        msgEntity.orientationPitch = jointEntity.orientationPitch;
        msgEntity.orientationYaw = jointEntity.orientationYaw;

        humanList_msg.humanList[id].meAgent.hasObjects.push_back(msgEntity.name);
        humanList_msg.humanList[id].meAgent.busyHands.push_back(jointEntity.name);

        return true;
    } else
        return false;

    return true;
}

bool putAtJointPosition(toaster_msgs::Entity& msgEntity, unsigned int id, std::string joint,
        toaster_msgs::RobotList robotList_msg) {

    toaster_msgs::Entity jointEntity;

    std::vector<std::string>::iterator it = std::find(robotList_msg.robotList[id].meAgent.skeletonNames.begin(), robotList_msg.robotList[id].meAgent.skeletonNames.end(), joint);
    if (it != robotList_msg.robotList[id].meAgent.skeletonNames.end()) {
        jointEntity = robotList_msg.robotList[id].meAgent.skeletonJoint[std::distance(robotList_msg.robotList[id].meAgent.skeletonNames.begin(), it)].meEntity;

        msgEntity.positionX = jointEntity.positionX;
        msgEntity.positionX = jointEntity.positionY;
        msgEntity.positionX = jointEntity.positionZ;
        msgEntity.orientationRoll = jointEntity.orientationRoll;
        msgEntity.orientationPitch = jointEntity.orientationPitch;
        msgEntity.orientationYaw = jointEntity.orientationYaw;

        robotList_msg.robotList[id].meAgent.hasObjects.push_back(msgEntity.name);
        robotList_msg.robotList[id].meAgent.busyHands.push_back(msgEntity.name);

        return true;
    } else
        return false;

    return true;
}


//////////////
// Services //
//////////////

bool addStream(toaster_msgs::AddStream::Request &req,
        toaster_msgs::AddStream::Response & res) {

    morseHuman_ = req.morseHuman;
    niutHuman_ = req.niutHuman;
    groupHuman_ = req.groupHuman;
    mocapHuman_ = req.mocapHuman;
    humanCentroid_ = req.humanCentroid;
    pr2Robot_ = req.pr2Robot;
    spencerRobot_ = req.spencerRobot;
    vimanObject_ = req.vimanObject;
    sparkObject_ = req.sparkObject;
    sparkFact_ = req.sparkFact;
    ROS_INFO("[pdg] setting pdg input");
    return true;
}

bool putInHand(toaster_msgs::PutInHand::Request &req,
        toaster_msgs::PutInHand::Response & res) {

    ROS_INFO("[pdg][Request][put_in_hand] we got request to put object %s with id: %d in "
            "agent %s with id: %d joint's %s\n", req.objectName.c_str(), req.objectId, req.agentName.c_str(), req.agentId, req.jointName.c_str());

    unsigned int agentId = 0;
    unsigned int objectId = 0;

    if (req.agentId != 0)
        agentId = req.agentId;
    else if (req.agentName != "")
        if (entNameId_.find(req.agentName) == entNameId_.end()) {
            ROS_INFO("[pdg][Request][put_in_hand][WARNING] we didn't find agent %s\n", req.agentName.c_str());
            return false;
        } else {
            agentId = entNameId_[req.agentName];
        }

    if (req.objectId != 0)
        objectId = req.objectId;
    else if (req.objectName != "")
        if (entNameId_.find(req.objectName) == entNameId_.end()) {
            ROS_INFO("[pdg][Request][put_in_hand][WARNING] we didn't find object %s\n", req.objectName.c_str());
            return false;
        } else {
            objectId = entNameId_[req.objectName];
        }

    if (req.jointName != "") {
        objectInAgent_[objectId] = agentId;
        objectInHand_[objectId] = req.jointName;
        return true;
    } else {
        ROS_INFO("[pdg][Request][put_in_hand][WARNING] joint name is empty %s\n", req.jointName.c_str());
        return false;
    }
    return true;
}

bool removeFromHand(toaster_msgs::RemoveFromHand::Request &req,
        toaster_msgs::RemoveFromHand::Response & res) {

    ROS_INFO("[pdg][Request][remove_from_hand] we got request to remove object %s with id: %d \n", req.objectName.c_str(), req.objectId);

    unsigned int objectId = 0;


    if (req.objectId != 0)
        objectId = req.objectId;
    else if (req.objectName != "")
        if (entNameId_.find(req.objectName) == entNameId_.end()) {
            ROS_INFO("[pdg][Request][put_in_hand][WARNING] we didn't find object %s\n", req.objectName.c_str());
            return false;
        } else {
            objectId = entNameId_[req.objectName];
        }

    objectInAgent_.erase(objectId);
    objectInHand_.erase(objectId);
    return true;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "pdg");
    ros::NodeHandle node;

    // For genom init


    //Data reading
    GroupHumanReader groupHumanRd(node, "/spencer/perception/tracked_groups");
    MorseHumanReader morseHumanRd(node, humanFullConfig_);
    //NiutHumanReader niutHumanRd()
    MocapHumanReader mocapHumanRd(node, "/optitrack_person/tracked_persons");

    Pr2RobotReader pr2RobotRd(robotFullConfig_);
    SpencerRobotReader spencerRobotRd(robotFullConfig_);

    // These 2 use special genom library!


    //Services
    ros::ServiceServer addStreamServ = node.advertiseService("pdg/manage_stream", addStream);
    ROS_INFO("Ready to manage stream.");

    ros::ServiceServer servicePutInHand = node.advertiseService("htn_verbalizer/put_in_hand", putInHand);
    ROS_INFO("[Request] Ready to put object in hand.");

    ros::ServiceServer serviceRemoveFromHand = node.advertiseService("htn_verbalizer/remove_from_hand", removeFromHand);
    ROS_INFO("[Request] Ready to remove object from hand.");

    //Data writing
    ros::Publisher object_pub = node.advertise<toaster_msgs::ObjectList>("pdg/objectList", 1000);
    ros::Publisher human_pub = node.advertise<toaster_msgs::HumanList>("pdg/humanList", 1000);
    ros::Publisher robot_pub = node.advertise<toaster_msgs::RobotList>("pdg/robotList", 1000);
    ros::Publisher fact_pub = node.advertise<toaster_msgs::FactList>("pdg/factList", 1000);





    ros::Rate loop_rate(30);

    tf::TransformListener listener;
    ROS_INFO("[PDG] initializing\n");


    while (node.ok()) {



        toaster_msgs::ObjectList objectList_msg;
        toaster_msgs::HumanList humanList_msg;
        toaster_msgs::RobotList robotList_msg;
        toaster_msgs::FactList factList_msg;
        toaster_msgs::Fact fact_msg;
        toaster_msgs::Object object_msg;
        toaster_msgs::Human human_msg;
        toaster_msgs::Robot robot_msg;
        toaster_msgs::Joint joint_msg;

        // Human centroid
        std::vector<double> centroidPossibleOrientations;
        std::map<unsigned int, unsigned int> centroidOrientationsRepartition;
        bg::model::point<double, 3, bg::cs::cartesian> centroid(0.0, 0.0, 0.0);

        //init if needed
        // These 2 use special genom library!

        //update data

        if (morseHuman_)
            morseHumanRd.updateHumans(listener);

        if (pr2Robot_)
            pr2RobotRd.updateRobot(listener);

        if (spencerRobot_)
            spencerRobotRd.updateRobot(listener);


        ///////////////////////////////////////////////////////////////////////

        //////////////////
        // publish data //
        //////////////////

        ////////////
        // Humans //
        ////////////

        if (morseHuman_)
            for (std::map<unsigned int, Human*>::iterator it = morseHumanRd.lastConfig_.begin(); it != morseHumanRd.lastConfig_.end(); ++it) {
                if (morseHumanRd.isPresent(it->first)) {

                    //Fact
                    fact_msg.property = "isPresent";
                    fact_msg.subjectId = it->first;
                    fact_msg.subjectName = it->second->getName();
                    fact_msg.stringValue = true;
                    fact_msg.confidence = 0.90;
                    fact_msg.factObservability = 1.0;
                    fact_msg.time = it->second->getTime();
                    fact_msg.valueType = 0;

                    factList_msg.factList.push_back(fact_msg);


                    //Human
                    fillEntity(it->second, human_msg.meAgent.meEntity);
                    humanList_msg.humanList.push_back(human_msg);

                    entNameId_[it->second->getName()] = it->first;
                }
            }

        if (mocapHuman_) {
            for (std::map<unsigned int, Human*>::iterator it = mocapHumanRd.lastConfig_.begin(); it != mocapHumanRd.lastConfig_.end(); ++it) {
                if (mocapHumanRd.isPresent(it->first)) {

                    //Fact
                    fact_msg.property = "isPresent";
                    fact_msg.subjectId = it->first;
                    fact_msg.subjectName = it->second->getName();
                    fact_msg.stringValue = true;
                    fact_msg.confidence = 0.90;
                    fact_msg.factObservability = 1.0;
                    fact_msg.time = it->second->getTime();
                    fact_msg.valueType = 0;

                    factList_msg.factList.push_back(fact_msg);


                    //Human
                    fillEntity(it->second, human_msg.meAgent.meEntity);
                    humanList_msg.humanList.push_back(human_msg);

                    entNameId_[it->second->getName()] = it->first;

                    if (humanCentroid_) {
                        centroid.set<0>(centroid.get<0>() + it->second->getPosition().get<0>());
                        centroid.set<1>(centroid.get<1>() + it->second->getPosition().get<1>());
                        centroid.set<2>(centroid.get<2>() + it->second->getPosition().get<2>());
                        if (centroidPossibleOrientations.size() == 0) {
                            centroidPossibleOrientations.push_back(it->second->getOrientation()[2]);
                            centroidOrientationsRepartition[0] = 1;
                        } else
                            for (int i = 0; i < centroidPossibleOrientations.size(); ++i) {
                                if (fabs(centroidPossibleOrientations[i] - it->second->getOrientation()[2]) < 0.53) {
                                    centroidOrientationsRepartition[i]++;
                                    break;
                                } else if (i == centroidPossibleOrientations.size() - 1) {
                                    centroidPossibleOrientations.push_back(it->second->getOrientation()[2]);
                                    centroidOrientationsRepartition[centroidPossibleOrientations.size()] = 1;
                                }
                            }
                    }
                }
            }
            if (humanCentroid_ && !mocapHumanRd.lastConfig_.empty()) {
                unsigned int maxValue = 0.0;
                unsigned int maxIndice = 0;
                human_msg.meAgent.meEntity.id = mocapHumanRd.lastConfig_.size() + 150;
                human_msg.meAgent.meEntity.name = "Human_Centroid";
                human_msg.meAgent.meEntity.positionX = centroid.get<0>() / mocapHumanRd.lastConfig_.size();
                human_msg.meAgent.meEntity.positionY = centroid.get<1>() / mocapHumanRd.lastConfig_.size();
                human_msg.meAgent.meEntity.positionZ = centroid.get<2>() / mocapHumanRd.lastConfig_.size();
                for (std::map<unsigned int, unsigned int>::iterator it = centroidOrientationsRepartition.begin(); it != centroidOrientationsRepartition.end(); ++it) {
                    if (it->second > maxValue) {
                        maxValue = it->second;
                        maxIndice = it->first;
                    }
                }
                human_msg.meAgent.meEntity.orientationYaw = centroidPossibleOrientations[maxIndice];
                humanList_msg.humanList.push_back(human_msg);

                entNameId_["Human_Centroid"] = mocapHumanRd.lastConfig_.size() + 150;
            }
        }

        if (groupHuman_)
            for (std::map<unsigned int, Human*>::iterator it = groupHumanRd.lastConfig_.begin(); it != groupHumanRd.lastConfig_.end(); ++it) {
                if (groupHumanRd.isPresent(it->first)) {

                    //Fact
                    fact_msg.property = "isPresent";
                    fact_msg.subjectId = it->first;
                    fact_msg.subjectName = it->second->getName();
                    fact_msg.stringValue = true;
                    fact_msg.confidence = 0.90;
                    fact_msg.factObservability = 1.0;
                    fact_msg.time = it->second->getTime();
                    fact_msg.valueType = 0;

                    factList_msg.factList.push_back(fact_msg);


                    //Human
                    fillEntity(it->second, human_msg.meAgent.meEntity);
                    humanList_msg.humanList.push_back(human_msg);

                    entNameId_[it->second->getName()] = it->first;
                }
            }

        ////////////////////////////////////////////////////////////////////////

        //////////
        //Robots//
        //////////

        if (pr2Robot_)
            for (std::map<unsigned int, Robot*>::iterator it = pr2RobotRd.lastConfig_.begin(); it != pr2RobotRd.lastConfig_.end(); ++it) {
                if (pr2RobotRd.isPresent(it->first)) {


                    //Fact
                    fact_msg.property = "isPresent";
                    fact_msg.subjectId = it->first;
                    fact_msg.subjectName = it->second->getName();
                    fact_msg.stringValue = true;
                    fact_msg.confidence = 0.90;
                    fact_msg.factObservability = 1.0;
                    fact_msg.time = it->second->getTime();
                    fact_msg.valueType = 0;


                    factList_msg.factList.push_back(fact_msg);


                    //Robot
                    robot_msg.meAgent.mobility = 0;
                    fillEntity(pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_], robot_msg.meAgent.meEntity);

                    if (robotFullConfig_) {
                        unsigned int i = 0;
                        for (std::map<std::string, Joint*>::iterator it = pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->skeleton_.begin(); it != pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->skeleton_.end(); ++it) {
                            robot_msg.meAgent.skeletonNames[i] = it->first;
                            fillEntity((it->second), joint_msg.meEntity);

                            joint_msg.jointOwner = 1;

                            robot_msg.meAgent.skeletonJoint.push_back(joint_msg);
                            i++;
                        }
                    }
                    robotList_msg.robotList.push_back(robot_msg);
                }

                entNameId_[it->second->getName()] = it->first;
            }


        if (spencerRobot_) {
            for (std::map<unsigned int, Robot*>::iterator it = spencerRobotRd.lastConfig_.begin(); it != spencerRobotRd.lastConfig_.end(); ++it) {
                //if (spencerRobotRd.isPresent(it->first)) {

                //Fact
                fact_msg.property = "isPresent";
                fact_msg.subjectId = it->first;
                fact_msg.subjectName = it->second->getName();
                fact_msg.stringValue = true;
                fact_msg.confidence = 0.90;
                fact_msg.factObservability = 1.0;
                fact_msg.time = it->second->getTime();
                fact_msg.valueType = 0;


                factList_msg.factList.push_back(fact_msg);

                //Robot
                robot_msg.meAgent.mobility = 0;
                fillEntity(spencerRobotRd.lastConfig_[spencerRobotRd.robotIdOffset_], robot_msg.meAgent.meEntity);

                /*if (robotFullConfig_) {
                    unsigned int i = 0;
                    for (std::map<std::string, Joint*>::iterator it = pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->skeleton_.begin(); it != pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->skeleton_.end(); ++it) {
                        robot_msg.meAgent.skeletonNames[i] = it->first;
                        fillEntity((it->second), joint_msg.meEntity);

                        joint_msg.jointOwner = 1;

                        robot_msg.meAgent.skeletonJoint.push_back(joint_msg);
                        i++;
                    }
                }*/
                robotList_msg.robotList.push_back(robot_msg);
                //}

                entNameId_[it->second->getName()] = it->first;
            }
        }

        ////////////////////////////////////////////////////////////////////////

        /////////////
        // Objects //
        /////////////

        //printf("[PDG][DEBUG] Nb object from SPARK: %d\n", sparkObjectRd.nbObjects_);

       

        //ROS_INFO("%s", msg.data.c_str());
        human_pub.publish(humanList_msg);
        robot_pub.publish(robotList_msg);
        fact_pub.publish(factList_msg);

        ros::spinOnce();

        /*       // Clear vectors
               objectList_msg.objectList.clear();
               humanList_msg.humanList.clear();
               robotList_msg.robotList.clear();
               robot_msg.meAgent.skeletonJoint.clear();
               factList_msg.factList.clear();
         */
        loop_rate.sleep();

    }
    return 0;
}
