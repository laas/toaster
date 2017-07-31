#include "pdg/utility/EntityUtility.h"
#include "pdg/publishers/RobotPublisher.h"

#include "toaster-lib/Robot.h"

#include <string>
#include <map>

void PublishRobot(Pr2RobotReader& pr2RobotRd, Entity& newPoseEnt_,
                     struct toasterList_t& list_msg,
                     bool FullConfig)
{
  for (std::map<std::string, Robot *>::iterator it = pr2RobotRd.lastConfig_.begin();
       it != pr2RobotRd.lastConfig_.end(); ++it) {
      if (newPoseEnt_.getId() == it->first)
          updateEntity(newPoseEnt_, it->second);
      //if (pr2RobotRd.isPresent(it->first)) {

      toaster_msgs::Fact fact_msg;

      //Fact
      fact_msg.property = "isPresent";
      fact_msg.subjectId = it->first;
      fact_msg.stringValue = "true";
      fact_msg.confidence = 0.90;
      fact_msg.factObservability = 1.0;
      fact_msg.time = it->second->getTime();
      fact_msg.valueType = 0;


      list_msg.fact_msg.factList.push_back(fact_msg);


      //Robot
      toaster_msgs::Robot robot_msg;
      robot_msg.meAgent.mobility = 0;
      fillEntity(pr2RobotRd.lastConfig_[it->first], robot_msg.meAgent.meEntity);

      if (FullConfig) {
          for (std::map<std::string, Joint *>::iterator itJoint = pr2RobotRd.lastConfig_[it->first]->skeleton_.begin();
               itJoint != pr2RobotRd.lastConfig_[it->first]->skeleton_.end(); ++itJoint) {
              toaster_msgs::Joint joint_msg;
              robot_msg.meAgent.skeletonNames.push_back(itJoint->first);
              fillEntity((itJoint->second), joint_msg.meEntity);

              joint_msg.jointOwner = it->first;
              joint_msg.position = itJoint->second->position;

              robot_msg.meAgent.skeletonJoint.push_back(joint_msg);
          }
      }
      list_msg.robot_msg.robotList.push_back(robot_msg);
      //}
  }
}

void PublishRobot(SpencerRobotReader& spencerRobotRd, Entity& newPoseEnt_,
                    struct toasterList_t& list_msg,
                    bool FullConfig)
{
  for (std::map<std::string, Robot*>::iterator it = spencerRobotRd.lastConfig_.begin(); it != spencerRobotRd.lastConfig_.end(); ++it) {
      if (newPoseEnt_.getId() == it->first)
          updateEntity(newPoseEnt_, it->second);
      //if (spencerRobotRd.isPresent(it->first)) {
      toaster_msgs::Fact fact_msg;

      //Fact
      fact_msg.property = "isPresent";
      fact_msg.subjectId = it->first;
      fact_msg.stringValue = true;
      fact_msg.confidence = 0.90;
      fact_msg.factObservability = 1.0;
      fact_msg.time = it->second->getTime();
      fact_msg.valueType = 0;


      list_msg.fact_msg.factList.push_back(fact_msg);

      //Robot
      toaster_msgs::Robot robot_msg;
      robot_msg.meAgent.mobility = 0;
      fillEntity(spencerRobotRd.lastConfig_[it->first], robot_msg.meAgent.meEntity);

      /*if (FullConfig) {
          unsigned int i = 0;
          for (std::map<std::string, Joint*>::iterator it = pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->skeleton_.begin(); it != pr2RobotRd.lastConfig_[pr2RobotRd.robotIdOffset_]->skeleton_.end(); ++it) {
              robot_msg.meAgent.skeletonNames[i] = it->first;
              fillEntity((it->second), joint_msg.meEntity);

              joint_msg.jointOwner = 1;

              robot_msg.meAgent.skeletonJoint.push_back(joint_msg);
              i++;
          }
      }*/
      list_msg.robot_msg.robotList.push_back(robot_msg);
      //}
  }
}

void PublishRobot(ToasterSimuRobotReader& toasterSimuRobotRd,
                    Entity& newPoseEnt_,
                    struct toasterList_t& list_msg,
                    bool FullConfig)
{
  for (std::map<std::string, Robot *>::iterator it = toasterSimuRobotRd.lastConfig_.begin();
       it != toasterSimuRobotRd.lastConfig_.end(); ++it) {
      if (newPoseEnt_.getId() == it->first)
          updateEntity(newPoseEnt_, it->second);
      //if (toasterSimuRobotRd.isPresent(it->first)) {
      toaster_msgs::Fact fact_msg;


      //Fact
      fact_msg.property = "isPresent";
      fact_msg.subjectId = it->first;
      fact_msg.stringValue = "true";
      fact_msg.confidence = 0.90;
      fact_msg.factObservability = 1.0;
      fact_msg.time = it->second->getTime();
      fact_msg.valueType = 0;


      list_msg.fact_msg.factList.push_back(fact_msg);


      //Robot
      toaster_msgs::Robot robot_msg;
      robot_msg.meAgent.mobility = 0;
      fillEntity(toasterSimuRobotRd.lastConfig_[it->first], robot_msg.meAgent.meEntity);

      if (FullConfig) {
          for (std::map<std::string, Joint *>::iterator itJoint = toasterSimuRobotRd.lastConfig_[it->first]->skeleton_.begin();
               itJoint != toasterSimuRobotRd.lastConfig_[it->first]->skeleton_.end(); ++itJoint) {
              toaster_msgs::Joint joint_msg;
              robot_msg.meAgent.skeletonNames.push_back(itJoint->first);
              fillEntity((itJoint->second), joint_msg.meEntity);

              joint_msg.jointOwner = it->first;
              joint_msg.position = itJoint->second->position;

              robot_msg.meAgent.skeletonJoint.push_back(joint_msg);
          }
      }
      list_msg.robot_msg.robotList.push_back(robot_msg);
      //}
  }
}
