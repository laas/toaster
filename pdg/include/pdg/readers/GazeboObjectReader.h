/*
 * File:   GazeboObjectReader.h
 * Author: Sandra Devin
 *
 * Created on November, 2016
 */

#ifndef GAZEBOOBJECTREADER_H
#define GAZEBOOBJECTREADER_H

//This class reads topic from Ar_track_alvar and converts data into toaster-lib type.

#include "ObjectReader.h"
#include <ros/ros.h>
#include <string>
#include "gazebo_msgs/ModelStates.h"

class GazeboObjectReader : public ObjectReader {

public:
    GazeboObjectReader(ros::NodeHandle& node, std::string topic);
		~GazeboObjectReader() {};

private:
    ros::Subscriber sub_;

    void CallbackObj(const gazebo_msgs::ModelStates::ConstPtr& msg);
};

#endif	/* GAZEBOOBJECTREADER_H */
