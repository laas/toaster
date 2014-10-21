#include "MorseHumanReader.h"


MorseHumanReader::MorseHumanReader(ros::NodeHandle& node, fullHuman): node_(node), fullHuman_(fullHuman){
  std::cout << "Initializing MorseHumanReader" << std::endl;
  // ******************************************
  // Starts listening to the joint_states topic
  // ******************************************

  if(fullHuman_){
    //ros::Subscriber sub = node.subscribe("/human/armature/joint_states", 1, humanJointStateCallBack);
  }
  m_LastTime = 0;
}

void MorseHumanReader::updateHumans(tf::TransformListener &listener) {
  //update 1st human, this should be extended for multi human
  updateHuman(listener, 101, "/human_base")
}

void MorseHumanReader::updateHuman(tf::TransformListener &listener, int humId, std::string humanBase){
  tf::StampedTransform transform;
  Human curHuman(humId);
  try{
    ros::Time now = ros::Time::now();
    listener.waitForTransform("/map", humanBase,
        now, ros::Duration(3.0));
        listener.lookupTransform("/map", humanBase,
        now, transform);

        //Human position
        curHuman.position.set<0>(transform.getOrigin().x());
        curHuman.position.set<1>(transform.getOrigin().y());
        curHuman.position.set<2>(transform.getOrigin().z());

        //Human orientation
        curHuman.orientation.push_back(tf::getRoll(transform.getRotation()));
        curHuman.orientation.push_back(tf::getPitch(transform.getRotation()));
        curHuman.orientation.push_back(tf::getYaw(transform.getRotation()));
        
        m_LastConfig[humId] = curHuman;
        m_LastTime[humId] = now.toNSec();

  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
}

//TODO: full human case
/*static void MorseHumanReader::humanJointStateCallBack(const sensor_msgs::JointState::ConstPtr& msg){
  humanConf.dofNb = 50;
  if(msg->position.size() > humanConf.dofNb){
    ROS_ERROR("Nb Dof error: Poster max nbDof %d < Ros nbDof %d", humanConf.dofNb, msg->position.size());
  return;
  }
  humanConf.dof[14] = msg->position[13]; //TorsoX
  humanConf.dof[15] = - msg->position[20]; //Head
  humanConf.dof[18] = 1.39626; //Shoulder Right
humanConf.dof[19] = 0.10472; //Shoulder Right
humanConf.dof[20] = -0.346273; //Shoulder Right
humanConf.dof[22] = msg->position[18]; //Right elbow
humanConf.dof[24] = msg->position[19]; //Right wrist
humanConf.dof[27] = -1.39626; //Left shoulder
humanConf.dof[28] = - msg->position[14]; //Left shoulder
humanConf.dof[29] = - 0.174533; //Left shoulder
humanConf.dof[31] = msg->position[15]; //Left elbow
humanConf.dof[33] = msg->position[16]; //Left wrist
humanConf.dof[37] = msg->position[24]; //Right hip
humanConf.dof[39] = msg->position[25]; //Right knee
humanConf.dof[40] = msg->position[22]; //Right ankle
humanConf.dof[44] = msg->position[10]; //Left hip
humanConf.dof[46] = msg->position[11]; //Left knee
humanConf.dof[47] = msg->position[5]; //Left ankle
}*/

