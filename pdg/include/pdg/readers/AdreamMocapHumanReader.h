/*
 * File:   AdreamMocapHumanReader.h
 * Author: sdevin
 *
 * Created on October 8, 2015, 1:24 PM
 */

#ifndef ADREAMMOCAPHUMANREADER_H
#define	ADREAMMOCAPHUMANREADER_H

//This class read topic from mocap and converts data into toaster-lib type.

#include "HumanReader.h"

#include <ros/ros.h>
#include <string>
#include "optitrack/or_pose_estimator_state.h"

class AdreamMocapHumanReader : public HumanReader {
public:
    AdreamMocapHumanReader() {};
    virtual ~AdreamMocapHumanReader() {};

    void init(ros::NodeHandle* node,
              std::string topicTorso = "/optitrack/bodies/Rigid_Body_3",
              std::string topicHead = "/optitrack/bodies/Rigid_Body_1",
              std::string topicHand = "/optitrack/bodies/Rigid_Body_2",
              std::string param = "/pdg/adreamMocapHuman");

    virtual void Publish(struct toasterList_t& list_msg);

private:
    bool torso_;
    double offset_x;
    double offset_y;
    double offset_z;
    ros::Subscriber subTorso_;
    ros::Subscriber subHead_;
    ros::Subscriber subHand_;
    void optitrackCallbackTorso(const optitrack::or_pose_estimator_state::ConstPtr& msg);
    void optitrackCallbackHead(const optitrack::or_pose_estimator_state::ConstPtr& msg);
    void optitrackCallbackHand(const optitrack::or_pose_estimator_state::ConstPtr& msg);
};

#endif	/* ADREAMMOCAPHUMANREADER_H */
