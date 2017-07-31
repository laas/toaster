#include "pdg/readers/ObjectReader.h"

ObjectReader::ObjectReader()
{
  nbObjects_ = 0;
}

ObjectReader::~ObjectReader()
{
  for(std::map<std::string, MovableObject*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it)
    delete it->second;
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
