#include "PDG/MorseHumanReader.h"
#include "PDG/Pr2RobotReader.h"
#include "PDG/VimanObjectReader.h"

#include <PDG/Entity.h>
#include <PDG/Agent.h>
#include <PDG/Joint.h>
#include <PDG/Robot.h>
#include <PDG/Human.h>

int main(int argc, char** argv) {
    const bool AGENT_FULL_CONFIG = false; //If false we will use only position and orientation

    ros::init(argc, argv, "PDG");

    ros::NodeHandle node;

    //Data reading
    MorseHumanReader morseHumanRd(node, AGENT_FULL_CONFIG);
    Pr2RobotReader pr2RobotRd(AGENT_FULL_CONFIG);
    VimanObjectReader vimanObjectRd("morseViman");

    //Data writting
    ros::Publisher human_pub = node.advertise<PDG::Human>("human/human1", 1000);
    ros::Publisher robot_pub = node.advertise<PDG::Robot>("robot/pr2", 1000);


    ros::Rate loop_rate(30);

    tf::TransformListener listener;
    printf("[PDG] initializing\n");

    while (node.ok()) {
        PDG::Human human_msg;
        PDG::Robot robot_msg;
        PDG::Joint joint_msg;


        //update data
        vimanObjectRd.updateObjects();
        morseHumanRd.updateHumans(listener);
        pr2RobotRd.updateRobot(listener);

        //publish data

        //Objects
        if (vimanObjectRd.nbObjects_ != 0) {
            printf("[PDG] nbobject %d\n", vimanObjectRd.nbObjects_);
            for (unsigned int i = 0; i < vimanObjectRd.nbObjects_; i++) {
                if (vimanObjectRd.isPresent(vimanObjectRd.objectIdOffset_ + i)) {
                    printf("[PDG] Last time object %d: %lu\n", i, vimanObjectRd.lastConfig_[vimanObjectRd.objectIdOffset_ + i]->getTime());
                    printf("[PDG] object %d named %s is present\n", vimanObjectRd.objectIdOffset_ + i, vimanObjectRd.lastConfig_[vimanObjectRd.objectIdOffset_ + i]->getName().c_str());
                }
            }
        }


        //Human
        if (morseHumanRd.lastConfig_.size() > 0) {
            for (unsigned int i = 0; i < morseHumanRd.lastConfig_.size(); i++) {
                if (morseHumanRd.isPresent(morseHumanRd.humanIdOffset_ + i)) {
                    //Human
                    human_msg.meAgent.mobility = 0;
                    human_msg.meAgent.meEntity.id = morseHumanRd.lastConfig_[morseHumanRd.humanIdOffset_]->getId();
                    human_msg.meAgent.meEntity.time = morseHumanRd.lastConfig_[morseHumanRd.humanIdOffset_]->getTime();
                    //      human_msg.meAgent.meEntity.name = morseHumanRd.lastConfig_[morseHumanRd.humanIdOffset_]->getName;
                    human_msg.meAgent.meEntity.positionX = morseHumanRd.lastConfig_[morseHumanRd.humanIdOffset_]->getPosition().get<0>();
                    human_msg.meAgent.meEntity.positionY = morseHumanRd.lastConfig_[morseHumanRd.humanIdOffset_]->getPosition().get<1>();
                    human_msg.meAgent.meEntity.positionZ = morseHumanRd.lastConfig_[morseHumanRd.humanIdOffset_]->getPosition().get<2>();
                    human_msg.meAgent.meEntity.orientationRoll = morseHumanRd.lastConfig_[morseHumanRd.humanIdOffset_]->getOrientation()[0];
                    human_msg.meAgent.meEntity.orientationPitch = morseHumanRd.lastConfig_[morseHumanRd.humanIdOffset_]->getOrientation()[1];
                    human_msg.meAgent.meEntity.orientationYaw = morseHumanRd.lastConfig_[morseHumanRd.humanIdOffset_]->getOrientation()[2];

                }
            }
        }

        if (pr2RobotRd.lastConfig_.size() > 0) {
            if ( pr2RobotRd.isPresent(pr2RobotRd.robotIdOffset_) ) {
                //Robot
                robot_msg.meAgent.mobility = 0;
                robot_msg.meAgent.meEntity.id = pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->getId();
                robot_msg.meAgent.meEntity.time = pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->getTime();
                //    robot_msg.meAgent.meEntity.name = pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->name;
                robot_msg.meAgent.meEntity.positionX = pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->getPosition().get<0>();
                robot_msg.meAgent.meEntity.positionY = pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->getPosition().get<1>();
                robot_msg.meAgent.meEntity.positionZ = pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->getPosition().get<2>();
                robot_msg.meAgent.meEntity.orientationRoll = pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->getOrientation()[0];
                robot_msg.meAgent.meEntity.orientationPitch = pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->getOrientation()[1];
                robot_msg.meAgent.meEntity.orientationYaw = pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->getOrientation()[2];

                if (AGENT_FULL_CONFIG) {
                    unsigned int i = 0;
                    for (std::map<std::string, Joint*>::iterator it = pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->skeleton_.begin(); it != pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->skeleton_.end(); ++it) {
                        robot_msg.meAgent.skeletonNames[i] = it->first;
                        joint_msg.meEntity.id = (it->second)->getId();
                        joint_msg.meEntity.time = (it->second)->getTime();
                        joint_msg.meEntity.positionX = (it->second)->getPosition().get<0>();
                        joint_msg.meEntity.positionY = (it->second)->getPosition().get<1>();
                        joint_msg.meEntity.positionZ = (it->second)->getPosition().get<2>();
                        joint_msg.meEntity.orientationRoll = (it->second)->getOrientation()[0];
                        joint_msg.meEntity.orientationPitch = (it->second)->getOrientation()[1];
                        joint_msg.meEntity.orientationYaw = (it->second)->getOrientation()[2];
                        joint_msg.jointOwner = 1;

                        robot_msg.meAgent.skeletonJoint[i] = joint_msg;
                        i++;
                    }
                }
            }
        }
        //ROS_INFO("%s", msg.data.c_str());

        human_pub.publish(human_msg);
        robot_pub.publish(robot_msg);
        ros::spinOnce();

        loop_rate.sleep();

    }
    return 0;
}
