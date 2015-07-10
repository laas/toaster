#include "pdg/MorseHumanReader.h"


MorseHumanReader::MorseHumanReader(ros::NodeHandle& node, bool fullHuman){
  node_ = node;
  fullHuman_ = fullHuman;
  std::cout << "[PDG] Initializing MorseHumanReader" << std::endl;
  init();
}

void MorseHumanReader::init(){
  if(fullHuman_){
    // Starts listening to the joint_states topic
    //ros::Subscriber sub_ = node_.subscribe("/human/armature/joint_states", 1, humanJointStateCallBack);
  }
}

void MorseHumanReader::updateHumans(tf::TransformListener &listener) {
  //update 1st human, this should be extended for multi human
  updateHuman(listener, humanIdOffset_, "/human_base");
}

void MorseHumanReader::updateHuman(tf::TransformListener &listener, int humId, std::string humanBase){
  tf::StampedTransform transform;
  Human* curHuman = new Human(humId);
  //TODO set name with humId
  curHuman->setName("Human1");
  std::vector<double> humanOrientation;
  bg::model::point<double, 3, bg::cs::cartesian> humanPosition;


  try{
    ros::Time now = ros::Time::now();
    listener.waitForTransform("/map", humanBase,
        now, ros::Duration(3.0));
        listener.lookupTransform("/map", humanBase,
        now, transform);

        //Human position
        humanPosition.set<0>(transform.getOrigin().x());
        humanPosition.set<1>(transform.getOrigin().y());
        humanPosition.set<2>(transform.getOrigin().z());

        //Human orientation
        //curHuman->orientation.push_back(tf::getRoll(transform.getRotation()));
        //curHuman->orientation.push_back(tf::getPitch(transform.getRotation()));
        humanOrientation.push_back(0.0);
        humanOrientation.push_back(0.0);
        humanOrientation.push_back(tf::getYaw(transform.getRotation()));
        
        curHuman->setOrientation(humanOrientation);
        curHuman->setPosition(humanPosition);
        curHuman->setTime(now.toNSec());

        lastConfig_[humId] = curHuman;

  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
}

//TODO: full human case
/*static void MorseHumanReader::humanJointStateCallBack(const sensor_msgs::JointState::ConstPtr& msg){
  if(m_LastConfig[101] != NULL){
      m_LastConfig[101]->skeleton[torso] =   msg->position[13]; //TorsoX
      m_LastConfig[101]->skeleton[head] =  - msg->position[20]; //Head
      m_LastConfig[101]->skeleton[r_elbow] = msg->position[18]; //Right elbow
      m_LastConfig[101]->skeleton[r_wrist] = msg->position[19]; //Right wrist
      m_LastConfig[101]->skeleton[l_shoulder] = - msg->position[14]; //Left shoulder
      m_LastConfig[101]->skeleton[l_elbow] = msg->position[15]; //Left elbow
      m_LastConfig[101]->skeleton[l_wrist] = msg->position[16]; //Left wrist
      m_LastConfig[101]->skeleton[r_hip] =   msg->position[24]; //Right hip
      m_LastConfig[101]->skeleton[r_knee] =  msg->position[25]; //Right knee
      m_LastConfig[101]->skeleton[r_ankle] = msg->position[22]; //Right ankle
      m_LastConfig[101]->skeleton[l_hip] =   msg->position[10]; //Left hip
      m_LastConfig[101]->skeleton[l_knee] =  msg->position[11]; //Left knee
      m_LastConfig[101]->skeleton[l_ankle] = msg->position[5]; //Left ankle
  }
}*/

//Destructor
MorseHumanReader::~MorseHumanReader(){
    for(std::map<unsigned int, Human*>::iterator it = lastConfig_.begin() ; it != lastConfig_.end() ; ++it){
        delete it->second;
    }
}
