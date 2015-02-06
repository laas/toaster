/* 
 * File:   main.cpp
 * Author: gmilliez
 *
 * Created on February 5, 2015, 2:49 PM
 */


#include "SPAR/PDGFactReader.h"
#include "SPAR/PDGObjectReader.h"
#include "PDG/FactList.h"
#include "PDG/Fact.h"

/*
 * 
 */
int main(int argc, char** argv) {
    // Set this in a ros service

    ros::init(argc, argv, "BELIEF_MANAGER");
    ros::NodeHandle node;

    //Data reading
    PDGFactReader factRdSpark(node, "SPARK/factList");
    PDGFactReader factRdSpar(node, "SPAR/factList");
    PDGFactReader factRdAM(node, "AGENT_MONITOR/factList");
    PDGObjectReader objectRd(node);

    // Agent with monitored belief
    // TODO make a ros service
    std::vector<std::string> agentsTracked;
    agentsTracked.push_back("HERAKLES_HUMAN1");

    static ros::Publisher fact_pub = node.advertise<PDG::FactList>("BELIEF_MANAGER/HERAKLES_HUMAN1/factList", 1000);

    PDG::FactList factList_msg;
    PDG::Fact fact_msg;

    // Set this in a ros service?
    ros::Rate loop_rate(30);

    // Vector of Objects
    // It should be possible to add an area on the fly with a ros service.
    std::map<std::string, Object*> mapObject;


    /************************/
    /* Start of the Ros loop*/
    /************************/

    while (node.ok()) {
        fact_pub.publish(factList_msg);

        ros::spinOnce();

        factList_msg.factList.clear();

        loop_rate.sleep();
    }
    return 0;
}

