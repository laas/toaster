#include "pdg/utility/EntityUtility.h"
#include "pdg/utility/XmlUtility.h"
#include "pdg/publishers/ObjectPublisher.h"

#include "toaster-lib/Object.h"

void PublishObject(ArObjectReader& arObjectRd, Entity& newPoseEnt_,
                     struct objectIn_t& objectIn,
                     struct toasterList_t& list_msg)
{
  for (std::map<std::string, MovableObject *>::iterator it = arObjectRd.lastConfig_.begin();
       it != arObjectRd.lastConfig_.end(); ++it) {
      if (newPoseEnt_.getId() == it->first) {
          updateEntity(newPoseEnt_, it->second);
          //Reset newPoseEnt_
          newPoseEnt_.setId("");
      }

      // If in hand, modify position:
      if (objectIn.Agent_.find(it->first) != objectIn.Agent_.end()) {
          bool addFactHand = true;
          if (!putAtJointPosition(it->second, objectIn.Agent_[it->first], objectIn.Hand_[it->first],
                                  list_msg.human_msg, true))
              if (!putAtJointPosition(it->second, objectIn.Agent_[it->first], objectIn.Hand_[it->first],
                                      list_msg.robot_msg, true)) {
                  ROS_INFO("[pdg][put_in_hand] couldn't find joint %s for agent %s \n",
                           objectIn.Hand_[it->first].c_str(), objectIn.Agent_[it->first].c_str());
                  addFactHand = false;
              }

          if (addFactHand) {
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


              list_msg.fact_msg.factList.push_back(fact_msg);
          }

      }
      //Message for object
      toaster_msgs::Object object_msg;
      fillEntity(it->second, object_msg.meEntity);
      list_msg.object_msg.objectList.push_back(object_msg);
  }
}

void PublishObject(OM2MObjectReader& om2mObjectRd, Entity& newPoseEnt_,
                  struct objectIn_t& objectIn,
                  struct toasterList_t& list_msg)
{
  for (std::map<std::string, MovableObject *>::iterator it_obj = om2mObjectRd.lastConfig_.begin();
       it_obj != om2mObjectRd.lastConfig_.end(); ++it_obj)
  {
      if (newPoseEnt_.getId() == it_obj->first)
      {
          updateEntity(newPoseEnt_, it_obj->second);
          //Reset newPoseEnt_
          newPoseEnt_.setId("");
      }

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
}

void PublishObject(GazeboObjectReader& gazeboRd, Entity& newPoseEnt_,
                    struct objectIn_t& objectIn,
                    struct toasterList_t& list_msg)
{
  for (std::map<std::string, MovableObject *>::iterator it = gazeboRd.lastConfig_.begin();
       it != gazeboRd.lastConfig_.end(); ++it) {
      if (newPoseEnt_.getId() == it->first) {
          updateEntity(newPoseEnt_, it->second);
          //Reset newPoseEnt_
          newPoseEnt_.setId("");
      }


      // If in hand, modify position:
      if (objectIn.Agent_.find(it->first) != objectIn.Agent_.end()) {
          bool addFactHand = true;
          if (!putAtJointPosition(it->second, objectIn.Agent_[it->first], objectIn.Hand_[it->first],
                                  list_msg.human_msg, true))
              if (!putAtJointPosition(it->second, objectIn.Agent_[it->first], objectIn.Hand_[it->first],
                                      list_msg.robot_msg, true)) {
                  ROS_INFO("[pdg][put_in_hand] couldn't find joint %s for agent %s \n",
                           objectIn.Hand_[it->first].c_str(), objectIn.Agent_[it->first].c_str());
                  addFactHand = false;
              }

          if (addFactHand) {

              //Fact message
              toaster_msgs::Fact fact_msg;
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


              list_msg.fact_msg.factList.push_back(fact_msg);
          }

      }
      //Message for object
      toaster_msgs::Object object_msg;
      fillEntity(it->second, object_msg.meEntity);
      list_msg.object_msg.objectList.push_back(object_msg);
  }
}

void PublishObject(ToasterSimuObjectReader& toasterSimuObjectRd,
                    Entity& newPoseEnt_,
                    struct objectIn_t& objectIn,
                    struct toasterList_t& list_msg)
{
  for (std::map<std::string, MovableObject *>::iterator it = toasterSimuObjectRd.lastConfig_.begin();
       it != toasterSimuObjectRd.lastConfig_.end(); ++it) {
      if (newPoseEnt_.getId() == it->first) {
          updateEntity(newPoseEnt_, it->second);
          //Reset newPoseEnt_
          newPoseEnt_.setId("");
          ROS_INFO("got true");
      }

      // If in hand, modify position:
      if (objectIn.Agent_.find(it->first) != objectIn.Agent_.end()) {
          bool addFactHand = true;
          if (!putAtJointPosition(it->second, objectIn.Agent_[it->first], objectIn.Hand_[it->first],
                                  list_msg.human_msg, true))
              if (!putAtJointPosition(it->second, objectIn.Agent_[it->first], objectIn.Hand_[it->first],
                                      list_msg.robot_msg, true)) {
                  ROS_INFO("[pdg][put_in_hand] couldn't find joint %s for agent %s \n",
                           objectIn.Hand_[it->first].c_str(), objectIn.Agent_[it->first].c_str());
                  addFactHand = false;
              }

          if (addFactHand) {

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


              list_msg.fact_msg.factList.push_back(fact_msg);
          }

      }

      //Message for object
      toaster_msgs::Object object_msg;
      fillEntity(it->second, object_msg.meEntity);
      list_msg.object_msg.objectList.push_back(object_msg);


      //printf("[PDG] Last time object %d: %lu\n", i, toasterSimuObjectRd.lastConfig_[toasterSimuObjectRd.objectIdOffset_ + i]->getTime());
      //printf("[PDG] object %d named %s is seen\n", toasterSimuObjectRd.objectIdOffset_ + i, toasterSimuObjectRd.lastConfig_[toasterSimuObjectRd.objectIdOffset_ + i]->getName().c_str());
      //}
  }
}
