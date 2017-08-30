/*
 * File:   MocapObjectReader.h
 * Author: gsarthou
 *
 * Created on 30/08/2017
 */

#ifndef MOCAPOBJECTREADER_H
#define	MOCAPOBJECTREADER_H

//This class read topic from mocap and converts data into toaster-lib type.

#include "ObjectReader.h"

#include <ros/ros.h>
#include <string>
#include "optitrack/or_pose_estimator_state.h"

class MocapObjectReader : public ObjectReader {
public:
    MocapObjectReader();
    virtual ~MocapObjectReader() {};

    void init(ros::NodeHandle* node,
              std::string topic,
              std::string id,
              std::string param = "/pdg/MocapObject");

private:
    double offset_x;
    double offset_y;
    double offset_z;
    std::string id_;
    ros::Subscriber sub_;
    void optitrackCallback(const optitrack::or_pose_estimator_state::ConstPtr& msg);
};

#endif	/* MOCAPOBJECTREADER_H */
