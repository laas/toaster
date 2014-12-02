#include "PDG/MorseHumanReader.h"
#include "PDG/Pr2RobotReader.h"

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
    Pr2RobotReader pr2RobotRd(node, 1, AGENT_FULL_CONFIG);

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
        morseHumanRd.updateHumans(listener);
        if (morseHumanRd.m_LastConfig[101] != NULL)
            printf("[PDG] Last time human 101: %lu\n", morseHumanRd.m_LastConfig[101]->getTime());

        pr2RobotRd.updateRobot(listener);
        if (pr2RobotRd.m_LastConfig[1] != NULL)
            printf("[PDG] Last time robot pr2: %lu\n", pr2RobotRd.m_LastConfig[1]->getTime());

        //publish data
        if (morseHumanRd.isPresent(101)) {

            //Human
            human_msg.meAgent.mobility = 0;
            human_msg.meAgent.meEntity.id = morseHumanRd.m_LastConfig[101]->getId();
            human_msg.meAgent.meEntity.time = morseHumanRd.m_LastConfig[101]->getTime();
            //      human_msg.meAgent.meEntity.name = morseHumanRd.m_LastConfig[101]->getName;
            human_msg.meAgent.meEntity.positionX = morseHumanRd.m_LastConfig[101]->getPosition().get<0>();
            human_msg.meAgent.meEntity.positionY = morseHumanRd.m_LastConfig[101]->getPosition().get<1>();
            human_msg.meAgent.meEntity.positionZ = morseHumanRd.m_LastConfig[101]->getPosition().get<2>();
            human_msg.meAgent.meEntity.orientationRoll = morseHumanRd.m_LastConfig[101]->getOrientation()[0];
            human_msg.meAgent.meEntity.orientationPitch = morseHumanRd.m_LastConfig[101]->getOrientation()[1];
            human_msg.meAgent.meEntity.orientationYaw = morseHumanRd.m_LastConfig[101]->getOrientation()[2];

        }

        if ( pr2RobotRd.isPresent(1) ) {
            //Robot
            robot_msg.meAgent.mobility = 0;
            robot_msg.meAgent.meEntity.id = pr2RobotRd.m_LastConfig[1]->getId();
            robot_msg.meAgent.meEntity.time = pr2RobotRd.m_LastConfig[1]->getTime();
            //    robot_msg.meAgent.meEntity.name = pr2RobotRd.m_LastConfig[1]->name;
            robot_msg.meAgent.meEntity.positionX = pr2RobotRd.m_LastConfig[1]->getPosition().get<0>();
            robot_msg.meAgent.meEntity.positionY = pr2RobotRd.m_LastConfig[1]->getPosition().get<1>();
            robot_msg.meAgent.meEntity.positionZ = pr2RobotRd.m_LastConfig[1]->getPosition().get<2>();
            robot_msg.meAgent.meEntity.orientationRoll = pr2RobotRd.m_LastConfig[1]->getOrientation()[0];
            robot_msg.meAgent.meEntity.orientationPitch = pr2RobotRd.m_LastConfig[1]->getOrientation()[1];
            robot_msg.meAgent.meEntity.orientationYaw = pr2RobotRd.m_LastConfig[1]->getOrientation()[2];

            if (AGENT_FULL_CONFIG) {
                unsigned int i = 0;
                for (std::map<std::string, Joint*>::iterator it = pr2RobotRd.m_LastConfig[1]->skeleton.begin(); it != pr2RobotRd.m_LastConfig[1]->skeleton.end(); ++it) {
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
        //ROS_INFO("%s", msg.data.c_str());

        human_pub.publish(human_msg);
        robot_pub.publish(robot_msg);

        ros::spinOnce();

        loop_rate.sleep();

    }
    return 0;
}
