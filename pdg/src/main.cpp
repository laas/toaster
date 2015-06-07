
// Humans
#include "pdg/MorseHumanReader.h"
#include "pdg/NiutHumanReader.h"
#include "pdg/GroupHumanReader.h"
#include "pdg/MocapHumanReader.h"

// Robots
#include "pdg/Pr2RobotReader.h"
#include "pdg/SpencerRobotReader.h"

// Objects
#include "pdg/VimanObjectReader.h"
#include "pdg/SparkObjectReader.h"

// Facts
#include "pdg/SparkFactReader.h"


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
    bool initSparkObject = false;
    bool initVimanObject = false;
    bool initSparkFact = false;


    //Data reading
    GroupHumanReader groupHumanRd(node, "/spencer/perception/tracked_groups");
    MorseHumanReader morseHumanRd(node, humanFullConfig_);
    //NiutHumanReader niutHumanRd()
    MocapHumanReader mocapHumanRd(node, "/optitrack_person/tracked_persons");

    Pr2RobotReader pr2RobotRd(robotFullConfig_);
    SpencerRobotReader spencerRobotRd(robotFullConfig_);

    // These 2 use special genom library!
    SparkObjectReader sparkObjectRd;
    VimanObjectReader vimanObjectRd;
    SparkFactReader sparkFactRd;


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
    ros::Publisher fact_pub_spark = node.advertise<toaster_msgs::FactList>("spark/factList", 1000);





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

        //init if needed
        // These 2 use special genom library!

        if (sparkObject_ && !initSparkObject) {
            sparkObjectRd.init("sparkEnvironment");
            initSparkObject = true;
        }
        if (vimanObject_ && !initVimanObject) {
            vimanObjectRd.init("morseViman");
            initVimanObject = true;
        }
        if (sparkFact_ && !initSparkFact) {
            sparkFactRd.init("sparkFactList");
            initSparkFact = true;
        }

        //update data

        if (vimanObject_)
            vimanObjectRd.updateObjects();

        if (sparkObject_)
            sparkObjectRd.updateObjects();

        if (morseHuman_)
            morseHumanRd.updateHumans(listener);

        if (pr2Robot_)
            pr2RobotRd.updateRobot(listener);

        if (spencerRobot_)
            spencerRobotRd.updateRobot(listener);

        if (sparkFact_)
            sparkFactRd.updateFacts();

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
                    fact_msg.stringValue = "true";
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
                    fact_msg.stringValue = "true";
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
                    fact_msg.stringValue = "true";
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

        if (sparkObject_)
            for (std::map<unsigned int, MovableObject*>::iterator it = sparkObjectRd.lastConfig_.begin(); it != sparkObjectRd.lastConfig_.end(); ++it) {
                //if (sparkObjectRd.isPresent(sparkObjectRd.objectIdOffset_ + i)) {

                //Fact message
                fact_msg.property = "IsPresent";
                fact_msg.propertyType = "position";
                fact_msg.subjectId = it->first;
                fact_msg.confidence = 0.90;
                fact_msg.factObservability = 1.0;
                fact_msg.time = it->second->getTime();
                fact_msg.subjectName = it->second->getName();
                fact_msg.valueType = 0;
                fact_msg.stringValue = "true";


                factList_msg.factList.push_back(fact_msg);


                //Message for object
                fillEntity(it->second, object_msg.meEntity);

                // If in hand, modify position:
                if (objectInAgent_.find(it->first) != objectInAgent_.end()) {
                    bool addFactHand = true;
                    if (!putAtJointPosition(object_msg.meEntity, objectInAgent_[it->first], objectInHand_[it->first], humanList_msg))
                        if (!putAtJointPosition(object_msg.meEntity, objectInAgent_[it->first], objectInHand_[it->first], robotList_msg)) {
                            ROS_INFO("[pdg][put_in_hand] couldn't find joint %s for agent %d \n", objectInHand_[it->first].c_str(), objectInAgent_[it->first]);
                            addFactHand = false;
                        }

                    if (addFactHand) {
                        std::string agentName = "";
                        for (std::map<std::string, unsigned int>::iterator itName = entNameId_.begin(); itName != entNameId_.end(); ++itName)
                            if (itName->second == objectInAgent_[it->first])
                                agentName = it->first;

                        //Fact message
                        fact_msg.property = "IsInHand";
                        fact_msg.propertyType = "position";
                        fact_msg.subProperty = "object";
                        fact_msg.subjectId = it->first;
                        fact_msg.subjectName = it->second->getName();
                        fact_msg.targetId = 0;
                        fact_msg.targetName = objectInHand_[it->first];
                        fact_msg.ownerId = objectInAgent_[it->first];
                        fact_msg.ownerName = agentName;
                        fact_msg.confidence = 1.0;
                        fact_msg.factObservability = 0.8;
                        fact_msg.time = it->second->getTime();
                        fact_msg.valueType = 0;
                        fact_msg.stringValue = "true";


                        factList_msg.factList.push_back(fact_msg);
                    }

                }

                objectList_msg.objectList.push_back(object_msg);

                entNameId_[it->second->getName()] = it->first;

                //printf("[PDG] Last time object %d: %lu\n", i, vimanObjectRd.lastConfig_[vimanObjectRd.objectIdOffset_ + i]->getTime());
                //printf("[PDG] object %d named %s is present\n", vimanObjectRd.objectIdOffset_ + i, vimanObjectRd.lastConfig_[vimanObjectRd.objectIdOffset_ + i]->getName().c_str());
                //}
            }

        // To compute which object are seen by the robot, we use Viman:
        if (vimanObject_)
            for (std::map<unsigned int, MovableObject*>::iterator it = vimanObjectRd.lastConfig_.begin(); it != vimanObjectRd.lastConfig_.end(); ++it) {
                if (vimanObjectRd.isPresent(it->first)) {


                    //Fact is seen from viman
                    fact_msg.property = "IsSeen";
                    fact_msg.propertyType = "affordance";
                    fact_msg.subjectId = it->first;
                    fact_msg.confidence = 0.90;
                    fact_msg.factObservability = 0.5;
                    fact_msg.time = it->second->getTime();
                    fact_msg.subjectName = it->second->getName();
                    fact_msg.valueType = 0;
                    fact_msg.stringValue = "true";

                    factList_msg.factList.push_back(fact_msg);
                }

                // We don't use Viman to publish object if spark is on...
                if (!sparkObject_) {
                    fillEntity(it->second, object_msg.meEntity);

                    //Message for object
                    fillEntity(it->second, object_msg.meEntity);

                    // If in hand, modify position:
                    if (objectInAgent_.find(it->first) != objectInAgent_.end()) {
                        bool addFactHand = true;
                        if (!putAtJointPosition(object_msg.meEntity, objectInAgent_[it->first], objectInHand_[it->first], humanList_msg))
                            if (!putAtJointPosition(object_msg.meEntity, objectInAgent_[it->first], objectInHand_[it->first], robotList_msg)) {
                                ROS_INFO("[pdg][put_in_hand] couldn't find joint %s for agent %d \n", objectInHand_[it->first].c_str(), objectInAgent_[it->first]);
                                addFactHand = false;
                            }

                        if (addFactHand) {

                            std::string agentName = "";
                            for (std::map<std::string, unsigned int>::iterator itName = entNameId_.begin(); itName != entNameId_.end(); ++itName)
                                if (itName->second == objectInAgent_[it->first])
                                    agentName = it->first;

                            //Fact message
                            fact_msg.property = "IsInHand";
                            fact_msg.propertyType = "position";
                            fact_msg.subProperty = "object";
                            fact_msg.subjectId = it->first;
                            fact_msg.subjectName = it->second->getName();
                            fact_msg.targetId = 0;
                            fact_msg.targetName = objectInHand_[it->first];
                            fact_msg.ownerId = objectInAgent_[it->first];
                            fact_msg.ownerName = agentName;
                            fact_msg.confidence = 1.0;
                            fact_msg.factObservability = 0.8;
                            fact_msg.time = it->second->getTime();
                            fact_msg.valueType = 0;
                            fact_msg.stringValue = "true";


                            factList_msg.factList.push_back(fact_msg);
                        }

                    }

                    objectList_msg.objectList.push_back(object_msg);

                    entNameId_[it->second->getName()] = it->first;
                }

                //printf("[PDG] Last time object %d: %lu\n", i, vimanObjectRd.lastConfig_[vimanObjectRd.objectIdOffset_ + i]->getTime());
                //printf("[PDG] object %d named %s is seen\n", vimanObjectRd.objectIdOffset_ + i, vimanObjectRd.lastConfig_[vimanObjectRd.objectIdOffset_ + i]->getName().c_str());
                //}
            }

        ////////////////////////////////////////////////////////////////////////



        //ROS_INFO("%s", msg.data.c_str());

        object_pub.publish(objectList_msg);
        human_pub.publish(humanList_msg);
        robot_pub.publish(robotList_msg);
        fact_pub.publish(factList_msg);
        fact_pub_spark.publish(sparkFactRd.currentFactList_);

        ros::spinOnce();

        loop_rate.sleep();

    }
    return 0;
}
