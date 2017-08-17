/*
 * File:   OM2MObjectReader.cpp
 * Author: Dorian Kurzaj
 *
 * Created on August, 2016
 */

#include "pdg/readers/OM2MObjectReader.h"

#include "pdg/utility/XmlUtility.h"

enum objectType_t
{
  sensor,
  other
};

struct preFact_t
{
  std::string type;
  std::string data;
  std::string param;
  enum objectType_t objectType;
};

OM2MObjectReader::OM2MObjectReader() : ObjectReader(){
    childs_.push_back(this);
}

void OM2MObjectReader::init(ros::NodeHandle* node, std::string topic, std::string param)
{
  std::cout << "[PDG] Initializing OM2MObjectReader" << std::endl;
  Reader<MovableObject>::init(node, param);
  // ******************************************
  // Starts listening to the joint_states
  sub_ = node_->subscribe(topic, 1, &OM2MObjectReader::newValueCallBack, this);
  std::cout << "Done\n";
}

std::string OM2MObjectReader::getSubpart(std::string data, std::string start, std::string stop)
{
  std::string part  = "";
  if(data.find(start) != string::npos)
  {
    size_t start_pos = data.find(start);
    size_t stop_pos = data.find(stop, start_pos);
    part = data.substr(start_pos+start.length(), stop_pos-start_pos-start.length());
  }

  return part;
}

vector<struct preFact_t> OM2MObjectReader::readPreFacts(MovableObject* object)
{
  vector<struct preFact_t> preFacts;
  size_t start_pos = 0;
  std::string content = object->getValue();
  while(content.find("\n", start_pos) != string::npos)
  {
    size_t stop_pos = content.find("\n", start_pos);
    std::string data = content.substr(start_pos, stop_pos-start_pos);
    if(data.find("=") != string::npos) // if valid data
    {
      size_t pos = data.find("=");
      std::string name = data.substr(0, pos);
      std::string type = getSubpart(data, "type=", " /");
      std::string value = getSubpart(data, "data=", " /");
      std::string unit = getSubpart(data, "unit=", " /");
      std::string color = getSubpart(data, "color=", " /");
      
      struct preFact_t preFact;
      if(name != "DATA")
        preFact.type = name;
      else if(type != "")
        preFact.type = type;
      else
        preFact.type = "";

      if(value == "true")
        preFact.data = "active";
      else if(value == "false")
        preFact.data = "inactive";
      else
        preFact.data = value;

      if((unit != "") && (data != "true") && (data != "false"))
        preFact.objectType = sensor;
      else
        preFact.objectType = other;

      if((color != "") && (preFact.type.find("LAMP") != string::npos))
        preFact.param = color;

      if(preFact.type != "")
        preFacts.push_back(preFact);
    }

    start_pos = stop_pos+1;
  }
  return preFacts;
}

void OM2MObjectReader::Publish(struct toasterList_t& list_msg)
{
  lastConfigMutex_.lock();
  for (std::map<std::string, MovableObject *>::iterator it_obj = globalLastConfig_.begin();
       it_obj != globalLastConfig_.end(); ++it_obj)
  {

    vector<struct preFact_t> preFacts = readPreFacts(it_obj->second);
    for(vector<struct preFact_t>::iterator it = preFacts.begin(); it != preFacts.end(); ++it)
    {
      toaster_msgs::Fact fact_msg;
      fact_msg.subjectId = it_obj->first;
      fact_msg.time = it_obj->second->getTime();

      if((it->data == "active") || (it->data == "inactive"))
        fact_msg.confidence = 1.0;
      else if(it->objectType == sensor)
        fact_msg.confidence = 1.0;
      else
        fact_msg.confidence = 0.7;

      if((it->data == "active") || (it->data == "inactive"))
        fact_msg.factObservability = 0.8;
      else if(it->objectType == sensor)
        fact_msg.factObservability = 0.3;
      else
        fact_msg.factObservability = 0.5;

      if((it->data == "active") || (it->data == "inactive"))
      {
        fact_msg.valueType = 0;
        fact_msg.stringValue = it->data;
      }
      else
      {
        fact_msg.valueType = 1;
        std::string::size_type sz;
        double doubleData = std::stod (it->data,&sz);
        fact_msg.doubleValue = doubleData;
      }

      if(it->objectType == sensor)
        fact_msg.subProperty = "physical_quantity";
      else
        fact_msg.subProperty = "object";

      if((it->data == "active") || (it->data == "inactive"))
        fact_msg.propertyType = "state";
      else
        fact_msg.propertyType = "measure";

      fact_msg.property = it->type;

      list_msg.fact_msg.factList.push_back(fact_msg);
    }
  }
  lastConfigMutex_.unlock();
}

void OM2MObjectReader::newValueCallBack(const toaster_msgs::IoTData::ConstPtr& msg) {
  if(activated_)
  {
    ros::Time now = ros::Time::now();
    // OM2M objects are by convention considered movable
    MovableObject* curObject;
    bool is_new = false;

    //create a new object with the same id and name as the message
    lastConfigMutex_.lock();
    if (globalLastConfig_.find(msg->data.key) == globalLastConfig_.end()) {
        curObject = new MovableObject(msg->data.key);
        curObject->setName(msg->data.key);
        increaseNbObjects();
        is_new = true;
    } else
        curObject = (MovableObject*)globalLastConfig_[msg->data.key];

    lastConfigMutex_.unlock();

    //set object position at default : 0,0,0
    bg::model::point<double, 3, bg::cs::cartesian> objectPosition;
    objectPosition.set<0>(0);
    objectPosition.set<1>(0);
    objectPosition.set<2>(0);

    //set the object orientation
    std::vector<double> objectOrientation;
    objectOrientation.push_back(0.0);
    objectOrientation.push_back(0.0);
    objectOrientation.push_back(0.0);


    //put the same orientation and positions as the previous time
    if(is_new)
    {
      curObject->setOrientation(objectOrientation);
      curObject->setPosition(objectPosition);
    }

    // set the new value of the OM2M object
    curObject->setValue(msg->data.value);

    // set the time
    unsigned long micro_sec = msg->header.stamp.sec * 1000000;
    curObject->setTime(micro_sec);

    lastConfigMutex_.lock();
    globalLastConfig_[msg->data.key]=curObject;
    lastConfigMutex_.unlock();
    lastConfig_[msg->data.key]=curObject;
  }
}
