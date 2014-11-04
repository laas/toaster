#include "SPAR/HumanReader.h"
#include "PDG/Human.h"

HumanReader::HumanReader(ros::NodeHandle& node, bool fullHuman){
  fullHuman_ = fullHuman;
  std::cout << "[SPAR] Initializing HumanReader" << std::endl;

  // Starts listening to the topic
  ros::Subscriber sub = node.subscribe("/human/human1", 1, humanJointStateCallBack);
}

static void HumanReader::humanJointStateCallBack(const PDG::Human::ConstPtr& msg){
  
  Human* curHuman = new Human(0);
  std::vector<double> humanOrientation;
  bg::model::point<double, 3, bg::cs::cartesian> humanPosition;

  curHuman.setMobility(msg->meAgent.mobility);
  curHuman.setId(msg->meAgent.meEntity.id);
  curHuman.setTime(msg->meAgent.meEntity.time);

  humanPosition.set<0>(msg->meAgent.meEntity.positionX);
  humanPosition.set<1>(msg->meAgent.meEntity.positionY);
  humanPosition.set<2>(msg->meAgent.meEntity.positionZ);
  curHuman.setPosition(humanPosition);

  humanOrientation.push_back(msg->meAgent.meEntity.orientationRoll);
  humanOrientation.push_back(msg->meAgent.meEntity.orientationPitch);
  humanOrientation.push_back(msg->meAgent.meEntity.orientationYaw);
  curHuman.setOrientation(humanOrientation);

  m_LastConfig[curHuman.getId()] = curHuman;

  if(fullHuman_){
  }
}

bool HumanReader::isPresent(int id){
  timeval curTime;
  gettimeofday(&curTime, NULL);
  unsigned long now = curTime.tv_sec * pow(10,9) + curTime.tv_usec;
  unsigned long timeThreshold = pow(10,9);
  //std::cout << "current time: " << now <<  "  human time: " << m_LastTime << std::endl;
  long timeDif = m_LastConfig[id]->getTime() - now;
  //std::cout << "time dif: " << timeDif << std::endl;

  if ( fabs(timeDif) < timeThreshold)
      return true;
  else
      return false;
}

