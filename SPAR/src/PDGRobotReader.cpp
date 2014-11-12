#include "SPAR/PDGRobotReader.h"
#include "PDG/Robot.h"

PDGRobotReader::PDGRobotReader(ros::NodeHandle& node, bool fullRobot){
  fullRobot_ = fullRobot;
  std::cout << "[SPAR] Initializing PDGRobotReader" << std::endl;

  // Starts listening to the topic
  sub_ = node.subscribe("/robot/pr2", 1, &PDGRobotReader::robotJointStateCallBack, this);
}

void PDGRobotReader::robotJointStateCallBack(const PDG::Robot::ConstPtr& msg){
  std::cout << "[SPAR] new data for robot received" << std::endl;
  
  Robot* curRobot = new Robot(msg->meAgent.meEntity.id);
  std::vector<double> robOrientation;
  bg::model::point<double, 3, bg::cs::cartesian> robPosition;

  Mobility curRobMobility = FULL;

  curRobot->setMobility(curRobMobility);
  curRobot->setTime(msg->meAgent.meEntity.time);

  robPosition.set<0>(msg->meAgent.meEntity.positionX);
  robPosition.set<1>(msg->meAgent.meEntity.positionY);
  robPosition.set<2>(msg->meAgent.meEntity.positionZ);
  curRobot->setPosition(robPosition);

  robOrientation.push_back(msg->meAgent.meEntity.orientationRoll);
  robOrientation.push_back(msg->meAgent.meEntity.orientationPitch);
  robOrientation.push_back(msg->meAgent.meEntity.orientationYaw);
  curRobot->setOrientation(robOrientation);

  m_LastConfig[curRobot->getId()] = curRobot;

  if(fullRobot_){
  }
}

bool PDGRobotReader::isPresent(int id){
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

