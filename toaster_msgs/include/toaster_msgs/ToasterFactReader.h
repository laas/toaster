/* 
 * File:   PDGFactReader.h
 * Author: gmilliez
 *
 * Created on February 6, 2015, 3:24 AM
 */

#ifndef TOASTERFACTREADER_H
#define	TOASTERFACTREADER_H

#include <ros/ros.h>
#include "toaster_msgs/FactList.h"

class ToasterFactReader {
public:
    toaster_msgs::FactList lastMsgFact;

    ToasterFactReader(ros::NodeHandle& node, std::string subTopic);

private:
    void factCallBack(const toaster_msgs::FactList::ConstPtr& msg);
    ros::Subscriber sub_;

};

#endif	/* TOASTERFACTREADER_H */

