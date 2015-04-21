/* 
 * File:   GroupHumanReader.h
 * Author: gmilliez
 *
 * Created on April 21, 2015, 12:51 AM
 */

#ifndef GROUPHUMANREADER_H
#define	GROUPHUMANREADER_H

#include "HumanReader.h"

#include <ros/ros.h>
#include "tf/transform_listener.h"
#include <string>
#include "geometry_msgs/PoseStamped.h"
#include "spencer_tracking_msgs/TrackedGroups.h"
#include "spencer_tracking_msgs/TrackedGroup.h"
#include "tf/transform_listener.h"
#include <sys/time.h>
#include <math.h>
#include <ostream>

class GroupHumanReader : public HumanReader {
public:
    GroupHumanReader(ros::NodeHandle& node, std::string topic);

private:
    ros::Subscriber sub_;
    void groupTrackCallback(const spencer_tracking_msgs::TrackedGroups::ConstPtr& msg);
    tf::TransformListener listener_;
};

#endif	/* GROUPHUMANREADER_H */

