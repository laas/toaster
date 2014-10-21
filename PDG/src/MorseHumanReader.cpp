
MorseHumanReader::MorseHumanReader(ros::NodeHandle& node): node_(node), fullHuman_(fullHuman){
  std::cout << "Initializing MorseHumanReader" << std::endl;
  // ******************************************
  // Starts listening to the joint_states topic
  // ******************************************

  if(fullHuman_){
    //ros::Subscriber sub = node.subscribe("/human/armature/joint_states", 1, humanJointStateCallBack);
  }
  m_LastTime = 0;
}

void MorseHumanReader::updateHuman(tf::TransformListener &listener) {
  tf::StampedTransform transform;
  try{
    ros::Time now = ros::Time::now();
    listener.waitForTransform("/map", "/human_base",
        now, ros::Duration(3.0));
        listener.lookupTransform("/map", "/human_base",
        now, transform);
        //TODO: replace this with human from toaster!
        humanConf.dof[0] = transform.getOrigin().x();
        humanConf.dof[1] = transform.getOrigin().y();
        humanConf.dof[2] = transform.getOrigin().z()+1.0;
        humanConf.dof[3] = tf::getRoll(transform.getRotation());
        humanConf.dof[4] = tf::getPitch(transform.getRotation());
        humanConf.dof[5] = tf::getYaw(transform.getRotation());
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  return TRUE;
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

