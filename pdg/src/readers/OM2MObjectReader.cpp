/*
 * File:   OM2MObjectReader.cpp
 * Author: Dorian Kurzaj
 *
 * Created on August, 2016
 */

#include "pdg/readers/OM2MObjectReader.h"

#include "toaster-lib/MovableIoTObject.h"
#include "pdg/utility/XmlUtility.h"

OM2MObjectReader::OM2MObjectReader() {
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

void OM2MObjectReader::Publish(struct toasterList_t& list_msg)
{
  lastConfigMutex_.lock();
  for (std::map<std::string, MovableObject *>::iterator it_obj = globalLastConfig_.begin();
       it_obj != globalLastConfig_.end(); ++it_obj)
  {
      std::string value_name;
      std::string value_value;

      // The names of the facts related to the current object
      std::vector<std::string> factNames;
      factNames = loadPropertiesFromXml(it_obj->first);

      std::string ae_data = (static_cast<MovableIoTObject*>(it_obj->second))->getValue().c_str();

      // For each fact we determine its value according to the oBIX XML received from OM2M
      for (std::vector<std::string>::iterator it_fact = factNames.begin(); it_fact != factNames.end(); ++it_fact)
      {
          //Fact message
          toaster_msgs::Fact fact_msg;
          fact_msg.property = *it_fact;
          fact_msg.propertyType = "iot";
          fact_msg.subjectId = it_obj->first;
          fact_msg.confidence = 1.0;
          // We can not know if the human has seen the value of the iot object
          fact_msg.factObservability = 0.5;
          fact_msg.time = it_obj->second->getTime();
          fact_msg.valueType = 0;

          bool value_found = false;
          size_t ref_pos = 0;
          while((ae_data.find(" : ", ref_pos) != std::string::npos) && !value_found)
          {
            size_t begin_pos = ae_data.find(" : ", ref_pos);
            size_t middle_pos = ae_data.find("=", begin_pos);
            size_t end_pos = ae_data.find(" / ", middle_pos);
            value_name = ae_data.substr(begin_pos + 3, middle_pos - begin_pos - 3);
            value_value = ae_data.substr(middle_pos + 1, end_pos - middle_pos - 1);
            ref_pos = end_pos;

            fact_msg.stringValue = loadValueFromXmlAsString(it_obj->first, *it_fact, value_name, value_value);

            // When there is a match we stop parsing
            if (!fact_msg.stringValue.empty())
                value_found = true;
          }

          // We add the fact to the list
          list_msg.fact_msg.factList.push_back(fact_msg);

      }

      //Message for object
      toaster_msgs::Object object_msg;
      fillValue(static_cast<MovableIoTObject*>(it_obj->second), object_msg);
      fillEntity(it_obj->second, object_msg.meEntity);
      list_msg.object_msg.objectList.push_back(object_msg);
  }
  lastConfigMutex_.unlock();
}

void OM2MObjectReader::newValueCallBack(const toaster_msgs::IoTData::ConstPtr& msg) {
  if(activated_)
  {
    ros::Time now = ros::Time::now();
    // OM2M objects are by convention considered movable
    MovableIoTObject* curObject;

    //create a new object with the same id and name as the message
    lastConfigMutex_.lock();
    if (globalLastConfig_.find(msg->data.key) == globalLastConfig_.end()) {
        curObject = new MovableIoTObject(msg->data.key);
        curObject->setName(msg->data.key);
        increaseNbObjects();
    } else {
        curObject = (MovableIoTObject*)globalLastConfig_[msg->data.key];
    }
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
    curObject->setOrientation(objectOrientation);
    curObject->setPosition(objectPosition);

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
