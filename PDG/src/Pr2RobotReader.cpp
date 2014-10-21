#include "Pr2RobotReader.h"

Pr2RobotReader::Pr2RobotReader(ros::NodeHandle& node, fullRobot): node_(node), fullRobot_(fullRobot){
  std::cout << "Initializing Pr2RobotReader" << std::endl;
  // ******************************************
  // Starts listening to the joint_states topic
  // ******************************************

  if(fullRobot_){
    //ros::Subscriber sub = node.subscribe("joint_states", 1, pr2JointStateCallBack);
  }
  m_LastTime = 0;
}


void Pr2RobotReader::updateRobot(tf::TransformListener &listener){
  tf::StampedTransform transform;
  Robot curRobot(robId);
  try{
    ros::Time now = ros::Time::now();
    listener.waitForTransform("/map", "/base_link",
        now, ros::Duration(3.0));
        listener.lookupTransform("/map", "/base_link",
        now, transform);

        //Robot position
        curRobot.position.set<0>(transform.getOrigin().x());
        curRobot.position.set<1>(transform.getOrigin().y());
        curRobot.position.set<2>(transform.getOrigin().z());

        //Robot orientation
        curRobot.orientation.push_back(tf::getRoll(transform.getRotation()));
        curRobot.orientation.push_back(tf::getPitch(transform.getRotation()));
        curRobot.orientation.push_back(tf::getYaw(transform.getRotation()));
        
        m_LastConfig = curRobot;
        m_LastTime = now.toNSec();

  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
}

