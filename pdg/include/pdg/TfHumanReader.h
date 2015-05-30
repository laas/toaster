/*
 * TfHumanReader.h
 *
 *  Created on: May 30, 2015
 *      Author: mfiore
 */

#ifndef SOURCE_DIRECTORY__TOASTER_PDG_INCLUDE_PDG_TFHUMANREADER_H_
#define SOURCE_DIRECTORY__TOASTER_PDG_INCLUDE_PDG_TFHUMANREADER_H_

// This class read topic from mocap and convert data into toaster-lib type.

#include "HumanReader.h"

#include <ros/ros.h>
#include "tf/transform_listener.h"
#include <string>
#include "geometry_msgs/PoseStamped.h"
#include "spencer_tracking_msgs/TrackedPersons.h"
#include "spencer_tracking_msgs/TrackedPerson.h"
#include "tf/transform_listener.h"
#include <sys/time.h>
#include <math.h>
#include <ostream>
#include <boost/thread.hpp>

class TfHumanReader : public HumanReader {
public:
    TfHumanReader(ros::NodeHandle& node);
    void readTf();

private:
    tf::TransformListener listener_;
};


#endif
