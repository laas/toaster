// This main will compute the facts for the requested agent.

#include "SPAR/PDGHumanReader.h"
#include "SPAR/PDGRobotReader.h"
#include "toaster-lib/TRBuffer.h"


int main(int argc, char** argv) {
    // Set this in a ros service
    const bool AGENT_FULL_CONFIG = false; //If false we will use only position and orientation

    // Set this in a ros service
    // TODO: Make it a vector?
    unsigned int agentMonitored = 101;
    
    // Map of Timed Ring Buffer Entities
    std::map<unsigned int, TRBuffer<Entity*>> mapTRBEntity;


    ros::init(argc, argv, "AGENT_MONITOR");
    ros::NodeHandle node;

    //Data reading
    PDGHumanReader humanRd(node, AGENT_FULL_CONFIG);
    PDGRobotReader robotRd(node, AGENT_FULL_CONFIG);
    //PDGObjectReader objectRd(node);

    // Set this in a ros service?
    ros::Rate loop_rate(30);


    /************************/
    /* Start of the Ros loop*/
    /************************/

    while (node.ok()) {
        if ((humanRd.lastConfig_[101] != NULL) && (robotRd.lastConfig_[1] != NULL)) {

        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
