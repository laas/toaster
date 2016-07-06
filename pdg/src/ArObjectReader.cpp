/*
 * File:   ArObjectReader.h
 * Author: JPaul Marcade
 *
 * Created on July, 2016
 */


#include "pdg/ArObjectReader.h"

ArObjectReader::ArObjectReader(ros::NodeHandle& node, std::string topicAR) {

    ROS_INFO("[ArObjectReader] Initializing");

    subAR_ = node.subscribe(topicAR, 1, &ArObjectReader::CallbackObj, this);

    ROS_INFO("[ArObjectReader] done");
}

void ArObjectReader::CallbackObj(const visualization_msgs::Marker::ConstPtr& msg) {

    tf::TransformListener listener;
	ros::Time now = ros::Time::now();
	tf::StampedTransform transform;
	MovableObject* curObject;

	//create a new object with the same id as the message
	if (lastConfig_.find(msg->ns) == lastConfig_.end()) {
		curObject = new MovableObject(msg->ns);
		curObject->setName(msg->ns);
	} else {
		curObject = lastConfig_[msg->ns];
	}


    listener.waitForTransform("/map", msg->header.frame_id,
                              now, ros::Duration(0.0));
    listener.lookupTransform("/map", msg->header.frame_id,
                             now, transform);


	//set object position
	bg::model::point<double, 3, bg::cs::cartesian> objectPosition;
	objectPosition.set<0>(transform.getOrigin().x());
	objectPosition.set<1>(transform.getOrigin().y());
	objectPosition.set<2>(transform.getOrigin().z());

	//set the object orientation
	std::vector<double> objectOrientation;

	objectOrientation.push_back(0.0);
	objectOrientation.push_back(0.0);

	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3 m(q);
	m.getEulerYPR(yaw,pitch,roll);

	objectOrientation.push_back(yaw);

	//put the data in the object
	curObject->setOrientation(objectOrientation);
	curObject->setPosition(objectPosition);
	curObject->setTime(now.toNSec());      //Similar to AdreamMoCapHumanReader. Is it better to use time stamp from msg

	lastConfig_[msg->ns]=curObject;

}

