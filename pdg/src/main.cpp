
// Humans
#include "pdg/MorseHumanReader.h"
#include "pdg/NiutHumanReader.h"
#include "pdg/GroupHumanReader.h"
#include "pdg/MocapHumanReader.h"
#include "pdg/AdreamMocapHumanReader.h"

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
#include <toaster_msgs/SetEntityPose.h>

bool humanFullConfig_ = true; //If false we will use only position and orientation
bool robotFullConfig_ = true; //If false we will use only position and orientation

// Stream to activate
bool morseHuman_ = false;
bool niutHuman_ = false;
bool groupHuman_ = false;
bool mocapHuman_ = false;
bool adreamMocapHuman_ = false;

bool pr2Robot_ = false;
bool spencerRobot_ = false;

bool vimanObject_ = false;
bool sparkObject_ = false;

bool sparkFact_ = false;

// Use to put an object in agent's hand
std::map<std::string, std::string> objectInAgent_;
std::map<std::string, std::string> objectInHand_;

//Used to change position of an entity
Entity newPoseEnt_("");

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

void updateEntity(Entity& newPoseEnt, Entity* storedEntity) {
    ROS_INFO("UPDATE entity");
    storedEntity->position_ = newPoseEnt.getPosition();
    storedEntity->orientation_ = newPoseEnt.getOrientation();
}

bool putAtJointPosition(toaster_msgs::Entity& msgEntity, std::string agentId, std::string joint,
        toaster_msgs::HumanList& humanList_msg) {

    toaster_msgs::Entity jointEntity;

    //  find back the agent:
    for (std::vector<toaster_msgs::Human>::iterator itAgent = humanList_msg.humanList.begin(); itAgent != humanList_msg.humanList.end(); ++itAgent) {
        if ((*itAgent).meAgent.meEntity.id == agentId) {

            //Find back the joint
            std::vector<std::string>::iterator it = std::find((*itAgent).meAgent.skeletonNames.begin(),
                    (*itAgent).meAgent.skeletonNames.end(), joint);

            if (it != (*itAgent).meAgent.skeletonNames.end()) {
                jointEntity = (*itAgent).meAgent.skeletonJoint[std::distance((*itAgent).meAgent.skeletonNames.begin(), it)].meEntity;

                msgEntity.positionX = jointEntity.positionX;
                msgEntity.positionY = jointEntity.positionY;
                msgEntity.positionZ = jointEntity.positionZ;
                msgEntity.orientationRoll = jointEntity.orientationRoll;
                msgEntity.orientationPitch = jointEntity.orientationPitch;
                msgEntity.orientationYaw = jointEntity.orientationYaw;

                (*itAgent).meAgent.hasObjects.push_back(msgEntity.id);
                (*itAgent).meAgent.busyHands.push_back(jointEntity.id);

                return true;
            } else
                ROS_WARN("Can't find joint %s for human %s. Couldn't attach object %s to this joint", joint.c_str(), agentId.c_str(), msgEntity.id.c_str());
        }
    }
    return false;
}

bool putAtJointPosition(toaster_msgs::Entity& msgEntity, std::string agentId, std::string joint,
        toaster_msgs::RobotList robotList_msg) {

    toaster_msgs::Entity jointEntity;

    //  find back the agent:
    for (std::vector<toaster_msgs::Robot>::iterator itAgent = robotList_msg.robotList.begin(); itAgent != robotList_msg.robotList.end(); ++itAgent) {
        if ((*itAgent).meAgent.meEntity.id == agentId) {

            //Find back the joint
            std::vector<std::string>::iterator it = std::find((*itAgent).meAgent.skeletonNames.begin(),
                    (*itAgent).meAgent.skeletonNames.end(), joint);

            if (it != (*itAgent).meAgent.skeletonNames.end()) {
                jointEntity = (*itAgent).meAgent.skeletonJoint[std::distance((*itAgent).meAgent.skeletonNames.begin(), it)].meEntity;

                msgEntity.positionX = jointEntity.positionX;
                msgEntity.positionY = jointEntity.positionY;
                msgEntity.positionZ = jointEntity.positionZ;
                msgEntity.orientationRoll = jointEntity.orientationRoll;
                msgEntity.orientationPitch = jointEntity.orientationPitch;
                msgEntity.orientationYaw = jointEntity.orientationYaw;

                (*itAgent).meAgent.hasObjects.push_back(msgEntity.id);
                (*itAgent).meAgent.busyHands.push_back(jointEntity.id);

                return true;
            } else
                ROS_WARN("Can't find joint %s for robot %s. Couldn't attach object %s to this joint", joint.c_str(), agentId.c_str(), msgEntity.id.c_str());
        }
    }
    return false;
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
    adreamMocapHuman_ = req.adreamMocapHuman;

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

    ROS_INFO("[pdg][Request][put_in_hand] we got request to put object %s in "
            "agent %s  joint's %s\n", req.objectId.c_str(), req.agentId.c_str(), req.jointName.c_str());


    if (req.agentId == "") {
        ROS_INFO("[pdg][Request][put_in_hand][WARNING] we didn't find agent %s\n", req.agentId.c_str());
        return false;
    }

    if (req.objectId == "") {
        ROS_INFO("[pdg][Request][put_in_hand][WARNING] we didn't find object %s\n", req.objectId.c_str());
        return false;
    }

    if (req.jointName != "") {
        objectInAgent_[req.objectId] = req.agentId;
        objectInHand_[req.objectId] = req.jointName;
        return true;
    } else {
        ROS_INFO("[pdg][Request][put_in_hand][WARNING] joint name is empty %s\n", req.jointName.c_str());
        return false;
    }
    return true;
}

bool removeFromHand(toaster_msgs::RemoveFromHand::Request &req,
        toaster_msgs::RemoveFromHand::Response & res) {

    ROS_INFO("[pdg][Request][remove_from_hand] we got request to remove object %s \n", req.objectId.c_str());

    if (req.objectId == "") {
        ROS_INFO("[pdg][Request][put_in_hand][WARNING] we didn't find object %s\n", req.objectId.c_str());
        return false;
    }

    objectInAgent_.erase(req.objectId);
    objectInHand_.erase(req.objectId);

    return true;
}

bool setEntityPose(toaster_msgs::SetEntityPose::Request &req,
        toaster_msgs::SetEntityPose::Response & res) {

    if (req.id != "") {
        newPoseEnt_.setId(req.id);
        newPoseEnt_.position_.set<0>(req.x);
        newPoseEnt_.position_.set<1>(req.y);
        newPoseEnt_.position_.set<2>(req.z);
        newPoseEnt_.orientation_[0] = req.roll;
        newPoseEnt_.orientation_[1] = req.pitch;
        newPoseEnt_.orientation_[2] = req.yaw;
        ROS_INFO("[toaster_simu][Request][INFO] request to set entity pose with "
                "id %s successful", req.id.c_str());
        res.answer = true;

    } else {

        ROS_WARN("[toaster_simu][Request] request to set entity pose with "
                "no id specified, sending back response: false");
        res.answer = false;
    }
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
    AdreamMocapHumanReader adreamMocapHumanRd(node, "/optitrack/bodies/Rigid_Body_1", "/optitrack/bodies/Rigid_Body_2");

    Pr2RobotReader pr2RobotRd(node, robotFullConfig_);
    SpencerRobotReader spencerRobotRd(robotFullConfig_);

    // These 2 use special genom library!
    SparkObjectReader sparkObjectRd;
    VimanObjectReader vimanObjectRd;

    SparkFactReader sparkFactRd;


    //Services
    ros::ServiceServer addStreamServ = node.advertiseService("pdg/manage_stream", addStream);
    ROS_INFO("Ready to manage stream.");

    ros::ServiceServer servicePutInHand = node.advertiseService("pdg/put_in_hand", putInHand);
    ROS_INFO("[Request] Ready to put object in hand.");

    ros::ServiceServer serviceRemoveFromHand = node.advertiseService("pdg/remove_from_hand", removeFromHand);
    ROS_INFO("[Request] Ready to remove object from hand.");

    ros::ServiceServer setEntPose = node.advertiseService("pdg/set_entity_pose", setEntityPose);
    ROS_INFO("Ready to set Entity position.");

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
            //vimanObjectRd.init("vimanObjectPose");
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
            for (std::map<std::string, Human*>::iterator it = morseHumanRd.lastConfig_.begin(); it != morseHumanRd.lastConfig_.end(); ++it) {
                if (newPoseEnt_.getId() == it->first)
                    updateEntity(newPoseEnt_, it->second);
                if (morseHumanRd.isPresent(it->first)) {

                    //Fact
                    fact_msg.property = "isPresent";
                    fact_msg.subjectId = it->first;
                    fact_msg.stringValue = "true";
                    fact_msg.confidence = 0.90;
                    fact_msg.factObservability = 1.0;
                    fact_msg.time = it->second->getTime();
                    fact_msg.valueType = 0;

                    factList_msg.factList.push_back(fact_msg);


                    //Human
                    fillEntity(it->second, human_msg.meAgent.meEntity);
                    humanList_msg.humanList.push_back(human_msg);

                }
            }

        if (mocapHuman_) {
            for (std::map<std::string, Human*>::iterator it = mocapHumanRd.lastConfig_.begin(); it != mocapHumanRd.lastConfig_.end(); ++it) {
                if (newPoseEnt_.getId() == it->first)
                    updateEntity(newPoseEnt_, it->second);
                if (mocapHumanRd.isPresent(it->first)) {

                    //Fact
                    fact_msg.property = "isPresent";
                    fact_msg.subjectId = it->first;
                    fact_msg.stringValue = "true";
                    fact_msg.confidence = 0.90;
                    fact_msg.factObservability = 1.0;
                    fact_msg.time = it->second->getTime();
                    fact_msg.valueType = 0;

                    factList_msg.factList.push_back(fact_msg);

                    //Human
                    fillEntity(it->second, human_msg.meAgent.meEntity);
                    humanList_msg.humanList.push_back(human_msg);

                }
            }
        }

        if (adreamMocapHuman_) {
            for (std::map<std::string, Human*>::iterator it = adreamMocapHumanRd.lastConfig_.begin(); it != adreamMocapHumanRd.lastConfig_.end(); ++it) {
                if (newPoseEnt_.getId() == it->first)
                    updateEntity(newPoseEnt_, it->second);
                if (adreamMocapHumanRd.isPresent(it->first)) {

                    //Fact
                    fact_msg.property = "isPresent";
                    fact_msg.subjectId = it->first;
                    fact_msg.stringValue = "true";
                    fact_msg.confidence = 0.90;
                    fact_msg.factObservability = 1.0;
                    fact_msg.time = it->second->getTime();
                    fact_msg.valueType = 0;

                    factList_msg.factList.push_back(fact_msg);

                    //Human
                    fillEntity(it->second, human_msg.meAgent.meEntity);

                    //if (humanFullConfig_) {
                    for (std::map<std::string, Joint*>::iterator itJoint = adreamMocapHumanRd.lastConfig_[it->first]->skeleton_.begin(); itJoint != adreamMocapHumanRd.lastConfig_[it->first]->skeleton_.end(); ++itJoint) {
                        human_msg.meAgent.skeletonNames.push_back(itJoint->first);
                        fillEntity((itJoint->second), joint_msg.meEntity);
                        joint_msg.jointOwner = it->first;

                        human_msg.meAgent.skeletonJoint.push_back(joint_msg);

                    }
                    //}
                    humanList_msg.humanList.push_back(human_msg);
                }
            }
        }

        if (groupHuman_)
            for (std::map<std::string, Human*>::iterator it = groupHumanRd.lastConfig_.begin(); it != groupHumanRd.lastConfig_.end(); ++it) {
                if (newPoseEnt_.getId() == it->first)
                    updateEntity(newPoseEnt_, it->second);
                if (groupHumanRd.isPresent(it->first)) {

                    //Fact
                    fact_msg.property = "isPresent";
                    fact_msg.subjectId = it->first;
                    fact_msg.stringValue = true;
                    fact_msg.confidence = 0.90;
                    fact_msg.factObservability = 1.0;
                    fact_msg.time = it->second->getTime();
                    fact_msg.valueType = 0;

                    factList_msg.factList.push_back(fact_msg);

                    //Human
                    fillEntity(it->second, human_msg.meAgent.meEntity);
                    humanList_msg.humanList.push_back(human_msg);
                }
            }

        ////////////////////////////////////////////////////////////////////////

        //////////
        //Robots//
        //////////

        if (pr2Robot_)
            for (std::map<std::string, Robot*>::iterator it = pr2RobotRd.lastConfig_.begin(); it != pr2RobotRd.lastConfig_.end(); ++it) {
                if (newPoseEnt_.getId() == it->first)
                    updateEntity(newPoseEnt_, it->second);
                //if (pr2RobotRd.isPresent(it->first)) {


                    //Fact
                    fact_msg.property = "isPresent";
                    fact_msg.subjectId = it->first;
                    fact_msg.stringValue = "true";
                    fact_msg.confidence = 0.90;
                    fact_msg.factObservability = 1.0;
                    fact_msg.time = it->second->getTime();
                    fact_msg.valueType = 0;


                    factList_msg.factList.push_back(fact_msg);


                    //Robot
                    robot_msg.meAgent.mobility = 0;
                    fillEntity(pr2RobotRd.lastConfig_[it->first], robot_msg.meAgent.meEntity);

                    if (robotFullConfig_) {
                        for (std::map<std::string, Joint*>::iterator itJoint = pr2RobotRd.lastConfig_[it->first]->skeleton_.begin(); itJoint != pr2RobotRd.lastConfig_[it->first]->skeleton_.end(); ++itJoint) {
                            robot_msg.meAgent.skeletonNames.push_back(itJoint->first);
                            fillEntity((itJoint->second), joint_msg.meEntity);

                            joint_msg.jointOwner = it->first;
                            joint_msg.position = itJoint->second->position;

                            robot_msg.meAgent.skeletonJoint.push_back(joint_msg);
                        }
                    }
                    robotList_msg.robotList.push_back(robot_msg);
                //}
            }


        if (spencerRobot_) {
            for (std::map<std::string, Robot*>::iterator it = spencerRobotRd.lastConfig_.begin(); it != spencerRobotRd.lastConfig_.end(); ++it) {
                if (newPoseEnt_.getId() == it->first)
                    updateEntity(newPoseEnt_, it->second);
                //if (spencerRobotRd.isPresent(it->first)) {

                //Fact
                fact_msg.property = "isPresent";
                fact_msg.subjectId = it->first;
                fact_msg.stringValue = true;
                fact_msg.confidence = 0.90;
                fact_msg.factObservability = 1.0;
                fact_msg.time = it->second->getTime();
                fact_msg.valueType = 0;


                factList_msg.factList.push_back(fact_msg);

                //Robot
                robot_msg.meAgent.mobility = 0;
                fillEntity(spencerRobotRd.lastConfig_[it->first], robot_msg.meAgent.meEntity);

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
            }
        }

        ////////////////////////////////////////////////////////////////////////

        /////////////
        // Objects //
        /////////////

        //printf("[PDG][DEBUG] Nb object from SPARK: %d\n", sparkObjectRd.nbObjects_);

        if (sparkObject_)
            for (std::map<std::string, MovableObject*>::iterator it = sparkObjectRd.lastConfig_.begin(); it != sparkObjectRd.lastConfig_.end(); ++it) {
                if (newPoseEnt_.getId() == it->first)
                    updateEntity(newPoseEnt_, it->second);
                //if (sparkObjectRd.isPresent(sparkObjectRd.objectIdOffset_ + i)) {

                //Fact message
                fact_msg.property = "IsPresent";
                fact_msg.propertyType = "position";
                fact_msg.subjectId = it->first;
                fact_msg.confidence = 0.90;
                fact_msg.factObservability = 1.0;
                fact_msg.time = it->second->getTime();
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
                            ROS_INFO("[pdg][put_in_hand] couldn't find joint %s for agent %s \n", objectInHand_[it->first].c_str(), objectInAgent_[it->first].c_str());
                            addFactHand = false;
                        }

                    if (addFactHand) {

                        //Fact message
                        fact_msg.property = "IsInHand";
                        fact_msg.propertyType = "position";
                        fact_msg.subProperty = "object";
                        fact_msg.subjectId = it->first;
                        fact_msg.targetId = objectInAgent_[it->first];
                        fact_msg.targetOwnerId = objectInAgent_[it->first];
                        fact_msg.confidence = 1.0;
                        fact_msg.factObservability = 0.8;
                        fact_msg.time = it->second->getTime();
                        fact_msg.valueType = 0;
                        fact_msg.stringValue = "true";


                        factList_msg.factList.push_back(fact_msg);
                    }

                }

                objectList_msg.objectList.push_back(object_msg);

                //printf("[PDG] Last time object %d: %lu\n", i, vimanObjectRd.lastConfig_[vimanObjectRd.objectIdOffset_ + i]->getTime());
                //printf("[PDG] object %d named %s is present\n", vimanObjectRd.objectIdOffset_ + i, vimanObjectRd.lastConfig_[vimanObjectRd.objectIdOffset_ + i]->getName().c_str());
                //}
            }

        // To compute which objects are seen by the robot, we use Viman:
        if (vimanObject_)
            for (std::map<std::string, MovableObject*>::iterator it = vimanObjectRd.lastConfig_.begin(); it != vimanObjectRd.lastConfig_.end(); ++it) {
                if (newPoseEnt_.getId() == it->first) {
                    updateEntity(newPoseEnt_, it->second);
                    //Reset newPoseEnt_
                    newPoseEnt_.setId("");
                    ROS_INFO("got true");
                }
                if (vimanObjectRd.isPresent(it->first)) {

                    //Fact is seen from viman
                    fact_msg.property = "IsSeen";
                    fact_msg.propertyType = "affordance";
                    fact_msg.subjectId = it->first;
                    fact_msg.confidence = 0.90;
                    fact_msg.factObservability = 0.5;
                    fact_msg.time = it->second->getTime();
                    fact_msg.valueType = 0;
                    fact_msg.stringValue = "true";

                    factList_msg.factList.push_back(fact_msg);
                }

                // We don't use Viman to publish object if spark is on...
                if (!sparkObject_) {

                    //Message for object
                    fillEntity(it->second, object_msg.meEntity);

                    // If in hand, modify position:
                    if (objectInAgent_.find(it->first) != objectInAgent_.end()) {
                        bool addFactHand = true;
                        if (!putAtJointPosition(object_msg.meEntity, objectInAgent_[it->first], objectInHand_[it->first], humanList_msg))
                            if (!putAtJointPosition(object_msg.meEntity, objectInAgent_[it->first], objectInHand_[it->first], robotList_msg)) {
                                ROS_INFO("[pdg][put_in_hand] couldn't find joint %s for agent %s \n", objectInHand_[it->first].c_str(), objectInAgent_[it->first].c_str());
                                addFactHand = false;
                            }

                        if (addFactHand) {

                            //Fact message
                            fact_msg.property = "IsInHand";
                            fact_msg.propertyType = "position";
                            fact_msg.subProperty = "object";
                            fact_msg.subjectId = it->first;
                            fact_msg.targetId = objectInAgent_[it->first];
                            fact_msg.targetOwnerId = objectInAgent_[it->first];
                            fact_msg.confidence = 1.0;
                            fact_msg.factObservability = 0.8;
                            fact_msg.time = it->second->getTime();
                            fact_msg.valueType = 0;
                            fact_msg.stringValue = "true";


                            factList_msg.factList.push_back(fact_msg);
                        }

                    }

                    objectList_msg.objectList.push_back(object_msg);
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
