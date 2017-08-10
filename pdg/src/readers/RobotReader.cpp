#include "pdg/readers/RobotReader.h"

RobotReader::RobotReader()
{
  fullRobot_ = false;
}

RobotReader::~RobotReader()
{
  for(std::map<std::string, Robot*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it)
    delete it->second;
}

bool RobotReader::isPresent(std::string id){
  timeval curTime;
  gettimeofday(&curTime, NULL);
  unsigned long now = curTime.tv_sec * pow(10,9) + curTime.tv_usec;
  unsigned long timeThreshold = pow(10,9);

  long timeDif = lastConfig_[id]->getTime() - now;

  if ( fabs(timeDif) < timeThreshold)
      return true;
  else
      return false;
}

toaster_msgs::Fact RobotReader::DefaultFactMsg(std::string subjectId, uint64_t factTime)
{
  toaster_msgs::Fact fact_msg;

  //Fact
  fact_msg.property = "isPresent";
  fact_msg.subjectId = subjectId;
  fact_msg.stringValue = "true";
  fact_msg.confidence = 0.90;
  fact_msg.factObservability = 1.0;
  fact_msg.time = factTime;
  fact_msg.valueType = 0;

  return fact_msg;
}
