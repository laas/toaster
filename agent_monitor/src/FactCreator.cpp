#include "FactCreator.h"

using namespace std;

toaster_msgs::Fact FactCreator::setFactBase(Joint* joint)
{
  toaster_msgs::Fact fact_msg;
  fact_msg.subjectId = joint->getId();
  fact_msg.time = joint->getTime();
  fact_msg.subjectOwnerId = joint->getAgentId();

  return fact_msg;
}

toaster_msgs::Fact FactCreator::setFactBase(string agent, map<string, TRBuffer < Entity* > >& mapTRBEntity)
{
  toaster_msgs::Fact fact_msg;
  fact_msg.subjectId = agent;
  fact_msg.time = mapTRBEntity[agent].back()->getTime();
  fact_msg.subjectOwnerId = "";

  return fact_msg;
}

toaster_msgs::Fact FactCreator::setMotionFact(toaster_msgs::Fact baseFact, double speed, double confidence, string type)
{
  baseFact.property = "IsMoving";
  baseFact.propertyType = "motion";
  baseFact.subProperty = type;
  baseFact.stringValue = "true";
  baseFact.doubleValue = speed;
  baseFact.confidence = confidence;
  baseFact.targetOwnerId = "";

  return baseFact;
}

toaster_msgs::Fact FactCreator::setDirectionFact(toaster_msgs::Fact baseFact, string target, double confidence)
{
  baseFact.property = "IsMovingToward";
  baseFact.propertyType = "motion";
  baseFact.subProperty = "direction";
  baseFact.targetId = target;
  baseFact.confidence = confidence;
  baseFact.doubleValue = confidence;
  baseFact.targetOwnerId = "";

  return baseFact;
}

toaster_msgs::Fact FactCreator::setDistanceFact(toaster_msgs::Fact baseFact, string target, double confidence)
{
  baseFact.property = "IsMovingToward";
  baseFact.propertyType = "motion";
  baseFact.subProperty = "distance";
  baseFact.targetId = target;
  baseFact.confidence = confidence;
  baseFact.doubleValue = confidence;
  baseFact.targetOwnerId = "";

  return baseFact;
}
