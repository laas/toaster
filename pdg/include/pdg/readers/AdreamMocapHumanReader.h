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
#include "tf/transform_listener.h"
#include <string>
#include "geometry_msgs/PoseStamped.h"
#include "optitrack/or_pose_estimator_state.h"
#include "tf/transform_listener.h"
#include <sys/time.h>
#include <math.h>
#include <ostream>

class AdreamMocapHumanReader : public HumanReader {
public:
    AdreamMocapHumanReader(ros::NodeHandle& node, std::string topicTorso, std::string topicHead, std::string topicHand);

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

