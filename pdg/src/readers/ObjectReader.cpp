#include "pdg/readers/ObjectReader.h"

//init static variables
unsigned int ObjectReader::nbObjects_ = 0;
std::map<std::string, MovableObject*> ObjectReader::globalLastConfig_;
bool ObjectReader::lastConfigMutex_ = false;
unsigned int ObjectReader::nbReaders_ = 0;

ObjectReader::ObjectReader()
{
  nbLocalObjects_ = 0;
  nbReaders_++;
}

ObjectReader::~ObjectReader()
{
  nbReaders_--;

  //delete globalLastConfig_ only if there are no more readers
  if(!nbReaders_)
  {
    WaitMutex(1000);
    for(std::map<std::string, MovableObject*>::iterator it = globalLastConfig_.begin(); it != lastConfig_.end(); ++it)
      delete it->second;
    releaseMutex();
  }
}

bool ObjectReader::isPresent(std::string id){
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

void ObjectReader::increaseNbObjects()
{
  nbObjects_++; /// total object number
  nbLocalObjects_--;
}

toaster_msgs::Fact ObjectReader::DefaultFactMsg(std::map<std::string, MovableObject *>::iterator it, struct objectIn_t& objectIn)
{
  toaster_msgs::Fact fact_msg;

  //Fact message
  fact_msg.property = "IsInHand";
  fact_msg.propertyType = "position";
  fact_msg.subProperty = "object";
  fact_msg.subjectId = it->first;
  fact_msg.targetId = objectIn.Agent_[it->first];
  fact_msg.targetOwnerId = objectIn.Agent_[it->first];
  fact_msg.confidence = 1.0;
  fact_msg.factObservability = 0.8;
  fact_msg.time = it->second->getTime();
  fact_msg.valueType = 0;
  fact_msg.stringValue = "true";

  return fact_msg;
}
