/* 
 * File:   ToasterFactReader.cpp
 * Author: gmilliez
 * 
 * Created on February 6, 2015, 3:24 AM
 */

#include "toaster_msgs/ToasterFactReader.h"

ToasterFactReader::ToasterFactReader(ros::NodeHandle& node, std::string subTopic) {
    //std::cout << " Initializing ToasterFactReader" << std::endl;

    // Starts listening to the topic
    sub_ = node.subscribe(subTopic, 1, &ToasterFactReader::factCallBack, this);
}

void ToasterFactReader::factCallBack(const toaster_msgs::FactList::ConstPtr& msg) {
   
    lastMsgFact.factList = msg->factList;
   
}
