// A human reader is a class that will read data from a middleware message
// and fill a Human class from toaster-lib accordingly to publish on a ros topic.
#include "pdg/readers/HumanReader.h"

HumanReader::HumanReader()
{
  fullHuman_ = false;
}

HumanReader::~HumanReader()
{
  for(std::map<std::string, Human*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it)
    delete it->second;
}

bool HumanReader::isPresent(std::string id){
  timeval curTime;
  gettimeofday(&curTime, NULL);
  unsigned long now = curTime.tv_sec * pow(10,9) + curTime.tv_usec;
  unsigned long timeThreshold = pow(10,9);
  //std::cout << "current time: " << now <<  "  human time: " << m_LastTime << std::endl;
  long timeDif = lastConfig_[id]->getTime() - now;
  //std::cout << "time dif: " << timeDif << std::endl;

  if ( fabs(timeDif) < timeThreshold)
      return true;
  else
      return false;
}

toaster_msgs::Fact HumanReader::DefaultFactMsg(std::string subjectId, uint64_t factTime)
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

void HumanReader::Publish(struct toasterList_t& list_msg)
{
  if(activated_)
  {
    for (std::map<std::string, Human *>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it)
    {
      if (isPresent(it->first))
      {
          toaster_msgs::Fact fact_msg = DefaultFactMsg(it->first, it->second->getTime());
          list_msg.fact_msg.factList.push_back(fact_msg);

          toaster_msgs::Human human_msg;
          fillEntity(it->second, human_msg.meAgent.meEntity);
          list_msg.human_msg.humanList.push_back(human_msg);
      }
    }
  }
}
