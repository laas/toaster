/*
 * File:   ToasterSimuHumanReader.h
 * Author: gmilliez
 *
 * Created on February 3, 2016, 12:15 PM
 */

#ifndef TOASTERSIMUHUMANREADER_H
#define	TOASTERSIMUHUMANREADER_H

#include <ros/ros.h>
#include "toaster-lib/Human.h"
#include "toaster_msgs/HumanListStamped.h"
#include "pdg/readers/HumanReader.h"

class ToasterSimuHumanReader : public HumanReader {
public:
    ToasterSimuHumanReader(ros::NodeHandle& node);

    ~ToasterSimuHumanReader() {};

private:
    void humanJointStateCallBack(const toaster_msgs::HumanListStamped::ConstPtr& msg);
    ros::Subscriber sub_;
};

#endif	/* TOASTERSIMUHUMANREADER_H */
