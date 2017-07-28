#include "pdg/utility/EntityUtility.h"
#include "pdg/publishers/HumanPublisher.h"

#include "toaster-lib/Human.h"

#include <string>
#include <map>

void PublishHuman(MorseHumanReader& morseHumanRd, Entity& newPoseEnt_,
                       toaster_msgs::FactList& factList_msg,
                       toaster_msgs::HumanListStamped& humanList_msg)
{
  for (std::map<std::string, Human *>::iterator it = morseHumanRd.lastConfig_.begin(); it != morseHumanRd.lastConfig_.end(); ++it)
  {
      if (newPoseEnt_.getId() == it->first)
          updateEntity(newPoseEnt_, it->second);
      if (morseHumanRd.isPresent(it->first))
      {
          toaster_msgs::Fact fact_msg;

          //Fact
          fact_msg.property = "isPresent";
          fact_msg.subjectId = it->first;
          fact_msg.stringValue = "true";
          fact_msg.confidence = 0.90;
          fact_msg.factObservability = 1.0;
          fact_msg.time = it->second->getTime();
          fact_msg.valueType = 0;

          factList_msg.factList.push_back(fact_msg);


          //Human
          toaster_msgs::Human human_msg;
          fillEntity(it->second, human_msg.meAgent.meEntity);
          humanList_msg.humanList.push_back(human_msg);
      }
  }
}

void PublishHuman(MocapHumanReader& mocapHumanRd, Entity& newPoseEnt_,
                       toaster_msgs::FactList& factList_msg,
                       toaster_msgs::HumanListStamped& humanList_msg)
{
  for (std::map<std::string, Human*>::iterator it = mocapHumanRd.lastConfig_.begin(); it != mocapHumanRd.lastConfig_.end(); ++it) {
      if (newPoseEnt_.getId() == it->first)
          updateEntity(newPoseEnt_, it->second);
      if (mocapHumanRd.isPresent(it->first)) {
          toaster_msgs::Fact fact_msg;

          //Fact
          fact_msg.property = "isPresent";
          fact_msg.subjectId = it->first;
          fact_msg.stringValue = "true";
          fact_msg.confidence = 0.90;
          fact_msg.factObservability = 1.0;
          fact_msg.time = it->second->getTime();
          fact_msg.valueType = 0;

          factList_msg.factList.push_back(fact_msg);

          //Human
          toaster_msgs::Human human_msg;
          fillEntity(it->second, human_msg.meAgent.meEntity);
          humanList_msg.humanList.push_back(human_msg);

      }
  }
}

void PublishHuman(AdreamMocapHumanReader& adreamMocapHumanRd,
                      Entity& newPoseEnt_,
                      toaster_msgs::FactList& factList_msg,
                      toaster_msgs::HumanListStamped& humanList_msg)
{
  for (std::map<std::string, Human*>::iterator it = adreamMocapHumanRd.lastConfig_.begin(); it != adreamMocapHumanRd.lastConfig_.end(); ++it) {
      if (newPoseEnt_.getId() == it->first)
          updateEntity(newPoseEnt_, it->second);
      if (adreamMocapHumanRd.isPresent(it->first)) {
          toaster_msgs::Fact fact_msg;

          //Fact
          fact_msg.property = "isPresent";
          fact_msg.subjectId = it->first;
          fact_msg.stringValue = "true";
          fact_msg.confidence = 0.90;
          fact_msg.factObservability = 1.0;
          fact_msg.time = it->second->getTime();
          fact_msg.valueType = 0;

          factList_msg.factList.push_back(fact_msg);

          //Human
          toaster_msgs::Human human_msg;
          fillEntity(it->second, human_msg.meAgent.meEntity);

          //if (humanFullConfig_) {
          for (std::map<std::string, Joint*>::iterator itJoint = adreamMocapHumanRd.lastConfig_[it->first]->skeleton_.begin(); itJoint != adreamMocapHumanRd.lastConfig_[it->first]->skeleton_.end(); ++itJoint) {
              toaster_msgs::Joint joint_msg;
              human_msg.meAgent.skeletonNames.push_back(itJoint->first);
              fillEntity((itJoint->second), joint_msg.meEntity);
              joint_msg.jointOwner = it->first;

              human_msg.meAgent.skeletonJoint.push_back(joint_msg);

          }
          //}
          humanList_msg.humanList.push_back(human_msg);
      }
  }
}

void PublishHuman(GroupHumanReader& groupHumanRd, Entity& newPoseEnt_,
                      toaster_msgs::FactList& factList_msg,
                      toaster_msgs::HumanListStamped& humanList_msg)
{
  for (std::map<std::string, Human *>::iterator it = groupHumanRd.lastConfig_.begin();
       it != groupHumanRd.lastConfig_.end(); ++it) {
      if (newPoseEnt_.getId() == it->first)
          updateEntity(newPoseEnt_, it->second);
      if (groupHumanRd.isPresent(it->first)) {
          toaster_msgs::Fact fact_msg;

          //Fact
          fact_msg.property = "isPresent";
          fact_msg.subjectId = it->first;
          fact_msg.stringValue = true;
          fact_msg.confidence = 0.90;
          fact_msg.factObservability = 1.0;
          fact_msg.time = it->second->getTime();
          fact_msg.valueType = 0;

          factList_msg.factList.push_back(fact_msg);

          //Human
          toaster_msgs::Human human_msg;
          fillEntity(it->second, human_msg.meAgent.meEntity);
          humanList_msg.humanList.push_back(human_msg);
      }
  }
}

void PublishHuman(ToasterSimuHumanReader& toasterSimuHumanRd,
                      Entity& newPoseEnt_,
                      toaster_msgs::FactList& factList_msg,
                      toaster_msgs::HumanListStamped& humanList_msg)
{
  for (std::map<std::string, Human*>::iterator it = toasterSimuHumanRd.lastConfig_.begin(); it != toasterSimuHumanRd.lastConfig_.end(); ++it) {
      if (newPoseEnt_.getId() == it->first)
          updateEntity(newPoseEnt_, it->second);

      //Human
      toaster_msgs::Human human_msg;
      fillEntity(it->second, human_msg.meAgent.meEntity);

      human_msg.meAgent.skeletonJoint.clear();
      human_msg.meAgent.skeletonNames.clear();

      for (std::map<std::string, Joint*>::iterator itJoint = toasterSimuHumanRd.lastConfig_[it->first]->skeleton_.begin(); itJoint != toasterSimuHumanRd.lastConfig_[it->first]->skeleton_.end(); ++itJoint) {
          toaster_msgs::Joint joint_msg;
          human_msg.meAgent.skeletonNames.push_back(itJoint->first);
          fillEntity((itJoint->second), joint_msg.meEntity);
          joint_msg.jointOwner = it->first;

          human_msg.meAgent.skeletonJoint.push_back(joint_msg);
      }
      humanList_msg.humanList.push_back(human_msg);
  }
}
