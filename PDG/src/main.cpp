#include "PDG/MorseHumanReader.h"
#include "PDG/Pr2RobotReader.h"
#include "PDG/VimanObjectReader.h"

// Message generated class
#include <PDG/Entity.h>
#include <PDG/Agent.h>
#include <PDG/Joint.h>
#include <PDG/Robot.h>
#include <PDG/Human.h>
#include <PDG/Object.h>
#include <PDG/RobotList.h>
#include <PDG/HumanList.h>
#include <PDG/ObjectList.h>

void feelEntity(Entity* srcEntity, PDG::Entity& msgEntity) {
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

int main(int argc, char** argv) {
    bool object_present = false;
    const bool AGENT_FULL_CONFIG = false; //If false we will use only position and orientation

    ros::init(argc, argv, "PDG");

    ros::NodeHandle node;

    //Data reading
    MorseHumanReader morseHumanRd(node, AGENT_FULL_CONFIG);
    Pr2RobotReader pr2RobotRd(AGENT_FULL_CONFIG);
    VimanObjectReader vimanObjectRd("morseViman");

    //Data writing
    ros::Publisher object_pub = node.advertise<PDG::ObjectList>("objectList", 1000);
    ros::Publisher human_pub = node.advertise<PDG::HumanList>("humanList", 1000);
    ros::Publisher robot_pub = node.advertise<PDG::RobotList>("robotList", 1000);


    ros::Rate loop_rate(30);

    tf::TransformListener listener;
    printf("[PDG] initializing\n");

    PDG::ObjectList objectList_msg;
    PDG::HumanList humanList_msg;
    PDG::RobotList robotList_msg;
    PDG::Object object_msg;
    PDG::Human human_msg;
    PDG::Robot robot_msg;
    PDG::Joint joint_msg;

    while (node.ok()) {

        //update data

        if (object_present)
            vimanObjectRd.updateObjects();
        morseHumanRd.updateHumans(listener);
        pr2RobotRd.updateRobot(listener);

        //publish data

        //Objects

        if (object_present)
            for (unsigned int i = 0; i < vimanObjectRd.nbObjects_; i++) {
                if (vimanObjectRd.isPresent(vimanObjectRd.objectIdOffset_ + i)) {
                    //Object
                    feelEntity(vimanObjectRd.lastConfig_[vimanObjectRd.objectIdOffset_ + i], object_msg.meEntity);
                    objectList_msg.objectList.push_back(object_msg);

                    //printf("[PDG] Last time object %d: %lu\n", i, vimanObjectRd.lastConfig_[vimanObjectRd.objectIdOffset_ + i]->getTime());
                    //printf("[PDG] object %d named %s is present\n", vimanObjectRd.objectIdOffset_ + i, vimanObjectRd.lastConfig_[vimanObjectRd.objectIdOffset_ + i]->getName().c_str());
                }
            }

        //Humans
        for (unsigned int i = 0; i < morseHumanRd.lastConfig_.size(); i++) {
            if (morseHumanRd.isPresent(morseHumanRd.humanIdOffset_ + i)) {
                //Human
                feelEntity(morseHumanRd.lastConfig_[morseHumanRd.humanIdOffset_ + i], human_msg.meAgent.meEntity);
                humanList_msg.humanList.push_back(human_msg);
            }
        }

        //Robots
        for (unsigned int i = 0; i < pr2RobotRd.lastConfig_.size(); i++) {
            if (pr2RobotRd.isPresent(pr2RobotRd.robotIdOffset_)) {
                //Robot
                robot_msg.meAgent.mobility = 0;
                feelEntity(pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_], robot_msg.meAgent.meEntity);

                if (AGENT_FULL_CONFIG) {
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
        //ROS_INFO("%s", msg.data.c_str());

        object_pub.publish(objectList_msg);
        human_pub.publish(humanList_msg);
        robot_pub.publish(robotList_msg);

        ros::spinOnce();

        // Clear vectors
        objectList_msg.objectList.clear();
        humanList_msg.humanList.clear();
        robotList_msg.robotList.clear();
        robot_msg.meAgent.skeletonJoint.clear();

        loop_rate.sleep();

    }
    return 0;
}
