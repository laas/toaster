#include "pdg/readers/RobotReader.h"

RobotReader::RobotReader()
{
  fullRobot_ = false;
}

RobotReader::~RobotReader()
{
  for(std::map<std::string, Robot*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it)
    delete it->second;
}

void RobotReader::Publish(struct toasterList_t& list_msg)
{
  if(activated_)
  {
    for (std::map<std::string, Robot *>::iterator it = lastConfig_.begin();
         it != lastConfig_.end(); ++it) {

        toaster_msgs::Fact fact_msg = DefaultFactMsg(it->first, it->second->getTime());
        list_msg.fact_msg.factList.push_back(fact_msg);

        //Robot
        toaster_msgs::Robot robot_msg;
        robot_msg.meAgent.mobility = 0;

        fillEntity(lastConfig_[it->first], robot_msg.meAgent.meEntity);

        if (fullRobot_)
        {
            for (std::map<std::string, Joint *>::iterator itJoint = lastConfig_[it->first]->skeleton_.begin();
                 itJoint != lastConfig_[it->first]->skeleton_.end(); ++itJoint)
            {
                toaster_msgs::Joint joint_msg;
                robot_msg.meAgent.skeletonNames.push_back(itJoint->first);
                fillEntity((itJoint->second), joint_msg.meEntity);

                joint_msg.jointOwner = it->first;
                joint_msg.position = itJoint->second->position;

                robot_msg.meAgent.skeletonJoint.push_back(joint_msg);
            }
        }
        list_msg.robot_msg.robotList.push_back(robot_msg);
    }
  }
}

bool RobotReader::isPresent(std::string id){
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

toaster_msgs::Fact RobotReader::DefaultFactMsg(std::string subjectId, uint64_t factTime)
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
