/* 
 * File:   AdaptationHumanReader.h
 * Author: sdevin
 *
 * Created on July 22, 2016, 12:15 PM
 */

#ifndef ADAPTATIONHUMANREADER_H
#define	ADAPTATIONHUMANREADER_H

#include <ros/ros.h>
#include "toaster-lib/Human.h"
#include "toaster_msgs/HumanListStamped.h"
#include "pdg/HumanReader.h"

class AdaptationHumanReader : public HumanReader {
public:
    AdaptationHumanReader(ros::NodeHandle& node, std::string topic);

private:
    void humanJointStateCallBack(const toaster_msgs::HumanListStamped::ConstPtr& msg);
    ros::Subscriber sub_;
};

#endif	/* ADAPTATIONHUMANREADER_H */
