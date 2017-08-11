/*
 * File:   GazeboObjectReader.cpp
 * Author: Sandra Devin
 *
 * Created on November, 2016
 */

#include "pdg/readers/GazeboObjectReader.h"

#include "tf/transform_listener.h"
#include <math.h>
#include <sys/time.h>
#include <ostream>

GazeboObjectReader::GazeboObjectReader() {
    childs_.push_back(this);
}

void GazeboObjectReader::init(ros::NodeHandle* node, std::string topic, std::string param)
{
  std::cout << "[PDG] Initializing GazeboObjectReader" << std::endl;
  Reader<MovableObject>::init(node, param);
  // ******************************************
  // Starts listening to the joint_states
  sub_ = node_->subscribe(topic, 1, &GazeboObjectReader::CallbackObj, this);
  std::cout << "Done\n";
}

void GazeboObjectReader::CallbackObj(const gazebo_msgs::ModelStates::ConstPtr& msg) {

  if(activated_)
  {
  	ros::Time now = ros::Time::now();
  	MovableObject* curObject;
  	std::vector<std::string> objectsName = msg->name;
  	std::vector<geometry_msgs::Pose> objectsPose = msg->pose;

  	if(objectsName.size() != objectsPose.size()){
  		ROS_ERROR("[GazeboObjectReader] Topic msg invalid: nb of objects diff from nb of poses!");
  		return;
  	}

  	for (int i = 0; i < objectsName.size(); i++) {

  		if(objectsName[i] == "pr2" || objectsName[i] == "ground_plane"){
  			//we ignore these objects
  			continue;
  		}

  		// If this object is not assigned we have to allocate data.
      lastConfigMutex_.lock();
  		if (globalLastConfig_.find(objectsName[i]) == globalLastConfig_.end()) {
  		    curObject = new MovableObject(objectsName[i]);
  		    curObject->setRoomId(0);
  		    curObject->setName(objectsName[i]);
          increaseNbObjects();
  		} else{
  		    curObject = globalLastConfig_[objectsName[i]];
  		}
      lastConfigMutex_.unlock();

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

      lastConfigMutex_.lock();
  		globalLastConfig_[objectsName[i]] = curObject;
      lastConfigMutex_.unlock();
      lastConfig_[objectsName[i]] = curObject;
  	}
  }
}
