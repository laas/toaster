/* 
 * File:   PDGFactReader.cpp
 * Author: gmilliez
 * 
 * Created on February 6, 2015, 3:24 AM
 */

#include "area_manager/PDGFactReader.h"

PDGFactReader::PDGFactReader(ros::NodeHandle& node, std::string subTopic) {
    std::cout << "[area_manager] Initializing PDGFactReader" << std::endl;

    // Starts listening to the topic
    sub_ = node.subscribe(subTopic, 1, &PDGFactReader::factCallBack, this);
}

void PDGFactReader::factCallBack(const pdg::FactList::ConstPtr& msg) {
    lastMsgFact.factList = msg->factList;
}
