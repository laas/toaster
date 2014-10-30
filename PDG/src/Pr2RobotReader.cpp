#include "PDG/Pr2RobotReader.h"

Pr2RobotReader::Pr2RobotReader(ros::NodeHandle& node, int id, bool fullRobot): pr2Id_(id) {
  node_ = node;
  fullRobot_ = fullRobot;
  std::cout << "Initializing Pr2RobotReader" << std::endl;
  init();
}

void Pr2RobotReader::init(){
  // Starts listening to the joint_states topic
  if(fullRobot_){
    //ros::Subscriber sub = node.subscribe("joint_states", 1, pr2JointStateCallBack);
  }
}

void Pr2RobotReader::updateRobot(tf::TransformListener &listener){
  tf::StampedTransform transform;
  Robot* curRobot = new Robot(pr2Id_);
  std::vector<double> robotOrientation;
  bg::model::point<double, 3, bg::cs::cartesian> robotPosition;

  try{
    ros::Time now = ros::Time::now();
    listener.waitForTransform("/map", "/base_link",
        now, ros::Duration(3.0));
        listener.lookupTransform("/map", "/base_link",
        now, transform);

        //Robot position
        robotPosition.set<0>(transform.getOrigin().x());
        robotPosition.set<1>(transform.getOrigin().y());
        robotPosition.set<2>(transform.getOrigin().z());

        //Robot orientation
        //curRobot->orientation.push_back(tf::getRoll(transform.getRotation()));
        //curRobot->orientation.push_back(tf::getPitch(transform.getRotation()));
        robotOrientation.push_back(0.0);
        robotOrientation.push_back(0.0);
        robotOrientation.push_back(tf::getYaw(transform.getRotation()));
        
        curRobot->setOrientation(robotOrientation);
        curRobot->setPosition(robotPosition);

        m_LastConfig[pr2Id_] = curRobot;
        m_LastTime[pr2Id_] = now.toNSec();

  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
}


//TODO: full robot case
