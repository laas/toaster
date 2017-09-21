/*
 * File:   MocapObjectReader.cpp
 * Author: gsarthou
 *
 */

#include "pdg/readers/MocapObjectReader.h"

#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"
#include <sys/time.h>
#include <math.h>
#include <ostream>

MocapObjectReader::MocapObjectReader() : ObjectReader()
{
}

// A human reader is a class that will read data from human(s)
void MocapObjectReader::init(ros::NodeHandle* node, std::string topic, std::string id, std::string param)
{
  std::cout << "[PDG] Initializing MocapObjectReader" << std::endl;
  Reader<MovableObject>::init(node, param);
  id_ = id;

  // ******************************************
  // Starts listening to the joint_states
  sub_ = node_->subscribe(topic, 1, &MocapObjectReader::optitrackCallback, this);

    if (node_->hasParam("mocap_calib_world_x"))
      node_->getParam("mocap_calib_world_x", offset_x);
    else
    {
      offset_x = 6.164;
      std::cout << "param mocap_calib_world_x note find : use default value : " << offset_x << std::endl;
    }

    if (node_->hasParam("mocap_calib_world_y"))
      node_->getParam("mocap_calib_world_y", offset_y);
    else
    {
      offset_y = 2.956;
      std::cout << "param mocap_calib_world_y note find : use default value : " << offset_y << std::endl;
    }

    if (node_->hasParam("mocap_calib_world_z"))
      node_->getParam("mocap_calib_world_z", offset_z);
    else
    {
      offset_z = 0;
      std::cout << "param mocap_calib_world_z note find : use default value : " << offset_z << std::endl;
    }
}

void MocapObjectReader::optitrackCallback(const optitrack::or_pose_estimator_state::ConstPtr & msg)
{
  if(activated_)
  {
    ros::Time now = ros::Time::now();
    MovableObject* curObject;

    try {
        //create a new object
        lastConfigMutex_.lock();
        if (globalLastConfig_.find(id_) == globalLastConfig_.end()) {
            curObject = new MovableObject(id_);
            curObject->setName(id_);
        } else
            curObject = globalLastConfig_[id_];
        lastConfigMutex_.unlock();

        if (msg->pos.size() != 0) {

            tf::Quaternion q(msg->pos[0].qx, msg->pos[0].qy, msg->pos[0].qz, msg->pos[0].qw);
            double roll, pitch, yaw;
            tf::Matrix3x3 m(q);
            m.getEulerYPR(yaw, pitch, roll);

            //set position
            bg::model::point<double, 3, bg::cs::cartesian> objectPosition;
            objectPosition.set<0>(msg->pos[0].x + offset_x);
            objectPosition.set<1>(msg->pos[0].y + offset_y);
            objectPosition.set<2>(msg->pos[0].z + offset_z);

            //set the orientation
            std::vector<double> objectOrientation;

            //transform the pose message
            objectOrientation.push_back(roll);
            objectOrientation.push_back(pitch);
            objectOrientation.push_back(yaw);

            //put the data in the object
            curObject->setOrientation(objectOrientation);
            curObject->setPosition(objectPosition);
            curObject->setTime(now.toNSec());

            lastConfigMutex_.lock();
            globalLastConfig_[id_] = curObject;
            lastConfigMutex_.unlock();
            lastConfig_[id_] = curObject;
        }

    } catch (tf::TransformException ex) {
      std::string err = "[Mocap " + id_ + " transfor] " + std::string(ex.what());
      ROS_ERROR("%s", err.c_str());
    }
  }
}
