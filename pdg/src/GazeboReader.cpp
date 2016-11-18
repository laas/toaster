/*
 * File:   GazeboReader.cpp
 * Author: Sandra Devin
 *
 * Created on November, 2016
 */

#include "pdg/GazeboReader.h"

GazeboReader::GazeboReader(ros::NodeHandle& node, std::string topic) {

    ROS_INFO("[GazeboReader] Initializing");

    sub_ = node.subscribe(topic, 1, &GazeboReader::CallbackObj, this);

    ROS_INFO("[GazeboReader] done");
}

void GazeboReader::CallbackObj(const gazebo_msgs::ModelStates::ConstPtr& msg) {

	ros::Time now = ros::Time::now();
	MovableObject* curObject;
	std::vector<std::string> objectsName = msg->name;
	std::vector<geometry_msgs::Pose> objectsPose = msg->pose;

	if(objectsName.size() != objectsPose.size()){
		ROS_ERROR("[GazeboReader] Topic msg invalid: nb of objects diff from nb of poses!");
		return;
	}

	for (int i = 0; i < objectsName.size(); i++) {

		if(objectsName[i] == "pr2" || objectsName[i] == "ground_plane"){
			//we ignore these objects
			break;
		}

		// If this object is not assigned we have to allocate data.
		if (lastConfig_.find(objectsName[i]) == lastConfig_.end()) {
		    curObject = new MovableObject(objectsName[i]);
		    curObject->setRoomId(0);
		    curObject->setName(objectsName[i]);
		} else{
		    curObject = lastConfig_[objectsName[i]];
		}

		std::vector<double> objOrientation;
		bg::model::point<double, 3, bg::cs::cartesian> objPosition;

		curObject->setId(objectsName[i]);

		curObject->setTime(now.toNSec());

		objPosition.set<0>(objectsPose[i].position.x);
		objPosition.set<1>(objectsPose[i].position.y);
		objPosition.set<2>(objectsPose[i].position.z);
		curObject->setPosition(objPosition);


		tf::Quaternion q;
		double roll, pitch, yaw;

		tf::quaternionMsgToTF(objectsPose[i].orientation, q);
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

		objOrientation.push_back(roll);
		objOrientation.push_back(pitch);
		objOrientation.push_back(yaw);
		curObject->setOrientation(objOrientation);

		lastConfig_[objectsName[i]] = curObject;
	}

}

