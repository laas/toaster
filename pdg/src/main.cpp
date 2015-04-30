
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

void feelEntity(Entity* srcEntity, toaster_msgs::Entity& msgEntity) {
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
    ROS_INFO("[pdg] setting pdg input");
    return true;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "pdg");

    ros::NodeHandle node;

    //Data reading
    GroupHumanReader groupHumanRd(node, "/spencer/perception/tracked_groups");
    MorseHumanReader morseHumanRd(node, humanFullConfig_);
    //NiutHumanReader niutHumanRd()
    MocapHumanReader mocapHumanRd(node, "/spencer/perception/tracked_persons");

    Pr2RobotReader pr2RobotRd(robotFullConfig_);
    SpencerRobotReader spencerRobotRd(robotFullConfig_);

    // These 2 use special genom library!
    SparkObjectReader sparkObjectRd("sparkEnvironment");
    VimanObjectReader vimanObjectRd("morseViman");


    //Services
    ros::ServiceServer addStreamServ = node.advertiseService("pdg/manage_stream", addStream);
    ROS_INFO("Ready to manage stream.");

    //Data writing
    ros::Publisher object_pub = node.advertise<toaster_msgs::ObjectList>("pdg/objectList", 1000);
    ros::Publisher human_pub = node.advertise<toaster_msgs::HumanList>("pdg/humanList", 1000);
    ros::Publisher robot_pub = node.advertise<toaster_msgs::RobotList>("pdg/robotList", 1000);
    ros::Publisher fact_pub = node.advertise<toaster_msgs::FactList>("pdg/factList", 1000);


    ros::Rate loop_rate(30);

    tf::TransformListener listener;
    ROS_INFO("[PDG] initializing\n");

    toaster_msgs::ObjectList objectList_msg;
    toaster_msgs::HumanList humanList_msg;
    toaster_msgs::RobotList robotList_msg;
    toaster_msgs::FactList factList_msg;
    toaster_msgs::Fact fact_msg;
    toaster_msgs::Object object_msg;
    toaster_msgs::Human human_msg;
    toaster_msgs::Robot robot_msg;
    toaster_msgs::Joint joint_msg;

    while (node.ok()) {

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

        //////////////////
        // publish data //
        //////////////////

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
                feelEntity(it->second, object_msg.meEntity);
                objectList_msg.objectList.push_back(object_msg);

                //printf("[PDG] Last time object %d: %lu\n", i, vimanObjectRd.lastConfig_[vimanObjectRd.objectIdOffset_ + i]->getTime());
                //printf("[PDG] object %d named %s is present\n", vimanObjectRd.objectIdOffset_ + i, vimanObjectRd.lastConfig_[vimanObjectRd.objectIdOffset_ + i]->getName().c_str());
                //}
            }

        // To compute which object are seen by the robot, we use Viman:
        if (vimanObject_)
            for (std::map<unsigned int, MovableObject*>::iterator it = vimanObjectRd.lastConfig_.begin(); it != vimanObjectRd.lastConfig_.end(); ++it) {


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

                // We don't use Viman to publish object if spark is on...
                if (!sparkObject_) {
                    feelEntity(it->second, object_msg.meEntity);
                    objectList_msg.objectList.push_back(object_msg);
                }

                //printf("[PDG] Last time object %d: %lu\n", i, vimanObjectRd.lastConfig_[vimanObjectRd.objectIdOffset_ + i]->getTime());
                //printf("[PDG] object %d named %s is seen\n", vimanObjectRd.objectIdOffset_ + i, vimanObjectRd.lastConfig_[vimanObjectRd.objectIdOffset_ + i]->getName().c_str());
                //}
            }

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
                    feelEntity(it->second, human_msg.meAgent.meEntity);
                    humanList_msg.humanList.push_back(human_msg);
                }
            }

        if (mocapHuman_)
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
                    feelEntity(it->second, human_msg.meAgent.meEntity);
                    humanList_msg.humanList.push_back(human_msg);
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
                    feelEntity(it->second, human_msg.meAgent.meEntity);
                    humanList_msg.humanList.push_back(human_msg);
                }
            }

        //Robots
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
                    feelEntity(pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_], robot_msg.meAgent.meEntity);

                    if (robotFullConfig_) {
                        unsigned int i = 0;
                        for (std::map<std::string, Joint*>::iterator it = pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->skeleton_.begin(); it != pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->skeleton_.end(); ++it) {
                            robot_msg.meAgent.skeletonNames[i] = it->first;
                            feelEntity((it->second), joint_msg.meEntity);

                            joint_msg.jointOwner = 1;

                            robot_msg.meAgent.skeletonJoint.push_back(joint_msg);
                            i++;
                        }
                    }
                    robotList_msg.robotList.push_back(robot_msg);
                }
            }


        if (spencerRobot_)
            for (std::map<unsigned int, Robot*>::iterator it = spencerRobotRd.lastConfig_.begin(); it != spencerRobotRd.lastConfig_.end(); ++it) {
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
                    feelEntity(pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_], robot_msg.meAgent.meEntity);

                    /*if (robotFullConfig_) {
                        unsigned int i = 0;
                        for (std::map<std::string, Joint*>::iterator it = pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->skeleton_.begin(); it != pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->skeleton_.end(); ++it) {
                            robot_msg.meAgent.skeletonNames[i] = it->first;
                            feelEntity((it->second), joint_msg.meEntity);

                            joint_msg.jointOwner = 1;

                            robot_msg.meAgent.skeletonJoint.push_back(joint_msg);
                            i++;
                        }
                    }*/
                    robotList_msg.robotList.push_back(robot_msg);
                }
            }



        //ROS_INFO("%s", msg.data.c_str());

        object_pub.publish(objectList_msg);
        human_pub.publish(humanList_msg);
        robot_pub.publish(robotList_msg);
        fact_pub.publish(factList_msg);

        ros::spinOnce();

        // Clear vectors
        objectList_msg.objectList.clear();
        humanList_msg.humanList.clear();
        robotList_msg.robotList.clear();
        robot_msg.meAgent.skeletonJoint.clear();
        factList_msg.factList.clear();

        loop_rate.sleep();

    }
    return 0;
}
