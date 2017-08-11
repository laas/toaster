#include "pdg/readers/ObjectReader.h"
#include <ostream>

//init static variables
unsigned int ObjectReader::nbObjects_ = 0;
std::vector<ObjectReader*> ObjectReader::childs_;
std::map<std::string, MovableObject*> ObjectReader::globalLastConfig_;
std::mutex ObjectReader::lastConfigMutex_;
unsigned int ObjectReader::nbReaders_ = 0;

ObjectReader::ObjectReader() : Reader<MovableObject>()
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
    lastConfigMutex_.lock();
    for(std::map<std::string, MovableObject*>::iterator it = globalLastConfig_.begin(); it != globalLastConfig_.end(); ++it)
      delete it->second;
    lastConfigMutex_.unlock();
  }
}

bool ObjectReader::isPresent(std::string id)
{
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

void ObjectReader::Publish(struct toasterList_t& list_msg, struct objectIn_t& objectIn)
{
  for(std::vector<ObjectReader*>::iterator it = childs_.begin(); it != childs_.end(); ++it)
    (*it)->Publish(list_msg);

  lastConfigMutex_.lock();
  for (std::map<std::string, MovableObject *>::iterator it = globalLastConfig_.begin();
       it != globalLastConfig_.end(); ++it)
  {
      // If in hand, modify position:
      putInHand(objectIn, it->first, it->second, list_msg);
      //Message for object
      toaster_msgs::Object object_msg;
      fillEntity(it->second, object_msg.meEntity);
      list_msg.object_msg.objectList.push_back(object_msg);
  }
  lastConfigMutex_.unlock();
}

void ObjectReader::increaseNbObjects()
{
  nbObjects_++; /// total object number
  nbLocalObjects_++;
}

toaster_msgs::Fact ObjectReader::DefaultFactMsg(string id, MovableObject* object, struct objectIn_t& objectIn)
{
  toaster_msgs::Fact fact_msg;

  //Fact message
  fact_msg.property = "IsInHand";
  fact_msg.propertyType = "position";
  fact_msg.subProperty = "object";
  fact_msg.subjectId = id;
  fact_msg.targetId = objectIn.Agent_[id];
  fact_msg.targetOwnerId = objectIn.Agent_[id];
  fact_msg.confidence = 1.0;
  fact_msg.factObservability = 0.8;
  fact_msg.time = object->getTime();
  fact_msg.valueType = 0;
  fact_msg.stringValue = "true";

  return fact_msg;
}

void ObjectReader::putInHand(struct objectIn_t& objectIn, string id, MovableObject* object, struct toasterList_t& list_msg)
{
  if (objectIn.Agent_.find(id) != objectIn.Agent_.end())
  {
    bool addFactHand = true;
    if (!putAtJointPosition(object, objectIn.Agent_[id], objectIn.Hand_[id],
                            list_msg.human_msg, true)) // try to put in human
      if (!putAtJointPosition(object, objectIn.Agent_[id], objectIn.Hand_[id],
                              list_msg.robot_msg, true)) // try to put in robot
      {
        ROS_INFO("[pdg][put_in_hand] couldn't find joint %s for agent %s \n",
                 objectIn.Hand_[id].c_str(), objectIn.Agent_[id].c_str());
        addFactHand = false;
      }

    if (addFactHand)
    {
      toaster_msgs::Fact fact_msg= DefaultFactMsg(id, object, objectIn);
      list_msg.fact_msg.factList.push_back(fact_msg);
    }
  }
}
