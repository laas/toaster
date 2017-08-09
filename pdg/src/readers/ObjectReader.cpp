#include "pdg/readers/ObjectReader.h"

//init static variables
unsigned int ObjectReader::nbObjects_ = 0;
std::map<std::string, MovableObject*> ObjectReader::lastConfig_;
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

  //delete lastConfig_ only if there are no more readers
  if(!nbReaders_)
  {
    WaitMutex(1000);
    for(std::map<std::string, MovableObject*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it)
      delete it->second;
    releaseMutex();
  }
}

bool ObjectReader::isPresent(std::string id){
  timeval curTime;
  gettimeofday(&curTime, NULL);
  unsigned long now = curTime.tv_sec * pow(10,9) + curTime.tv_usec;
  unsigned long timeThreshold = pow(10,9);
  WaitMutex(1000);
  long timeDif = lastConfig_[id]->getTime() - now;
  releaseMutex();

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
