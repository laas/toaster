// This class read topic from mocap and convert data into toaster-lib type.

#ifndef MOCAPHUMANREADER_H
#define MOCAPHUMANREADER_H

#include "HumanReader.h"

#include <ros/ros.h>
#include <string>
#include "spencer_tracking_msgs/TrackedPersons.h"
#include "spencer_tracking_msgs/TrackedPerson.h"
#include "tf/transform_listener.h"

class MocapHumanReader : public HumanReader {
public:
    MocapHumanReader() {};
    ~MocapHumanReader() {};

    void init(ros::NodeHandle* node, std::string topic, std::string param);

private:
    ros::Subscriber sub_;
    void optitrackCallback(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg);
    tf::TransformListener listener_;
};

#endif
