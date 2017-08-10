/*
 * File:   ArObjectReader.cpp
 * Author: JPaul Marcade
 *
 * Created on July, 2016
 */

#include "pdg/readers/ArObjectReader.h"

#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include <math.h>
#include <sys/time.h>
#include <ostream>

ArObjectReader::ArObjectReader(ros::NodeHandle& node, std::string topicAR) {

    ROS_INFO("[ArObjectReader] Initializing");

    sub_ = node.subscribe(topicAR, 1, &ArObjectReader::CallbackObj, this);

    ROS_INFO("[ArObjectReader] done");
}

void ArObjectReader::CallbackObj(const visualization_msgs::Marker::ConstPtr& msg) {

	ros::Time now = ros::Time::now();
	MovableObject* curObject;

	//create a new object with the same id as the message
  lastConfigMutex_.lock();
	if (globalLastConfig_.find(msg->ns) == globalLastConfig_.end()) {
		curObject = new MovableObject(msg->ns);
		curObject->setName(msg->ns);
    increaseNbObjects();
	} else {
		curObject = globalLastConfig_[msg->ns];
	}
  lastConfigMutex_.unlock();


	//set object position
	bg::model::point<double, 3, bg::cs::cartesian> objectPosition;
	objectPosition.set<0>(msg->pose.position.x);
	objectPosition.set<1>(msg->pose.position.y);
	objectPosition.set<2>(msg->pose.position.z);

	//set the object orientation
	std::vector<double> objectOrientation;



	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3 m(q);
	m.getEulerYPR(yaw,pitch,roll);

	objectOrientation.push_back(roll);
	objectOrientation.push_back(pitch);
	objectOrientation.push_back(yaw);

	//put the data in the object
	curObject->setOrientation(objectOrientation);
	curObject->setPosition(objectPosition);
	curObject->setTime(now.toNSec());      //Similar to AdreamMoCapHumanReader. Is it better to use time stamp from msg

  lastConfigMutex_.lock();
	globalLastConfig_[msg->ns]=curObject;
  lastConfigMutex_.unlock();
  lastConfig_[msg->ns]=curObject;
}
