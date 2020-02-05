#include "AgentMonitor.h"

#include "LookingFact.h"

void AgentMonitor::init(ros::NodeHandle* node)
{
  agentsManager_.init(node);
}

void AgentMonitor::updateMonitored()
{
  agentsMonitored_ = agentsManager_.getMonitoredAgents();
  mapAgentToJointsMonitored_ = agentsManager_.getMonitoredJoints();

  // If we start to monitor all humans, we add them to the agentsMonitored_ vector
  if (agentsManager_.startMonitorAllHumans())
      for (std::map<std::string, Human*>::iterator it = humansMap_.begin(); it != humansMap_.end(); ++it)
          if ((it->second->getId() != "") && (std::find(agentsMonitored_.begin(), agentsMonitored_.end(), it->second->getId()) == agentsMonitored_.end()))
              agentsManager_.addMonitoredAgent(it->first);

  // If we start to monitor all robots, we add them to the agentsMonitored_ vector
  if (agentsManager_.startMonitorAllRobots())
      for (std::map<std::string, Robot*>::iterator it = robotsMap_.begin(); it != robotsMap_.end(); ++it)
          if ((it->second->getId() != "") && (std::find(agentsMonitored_.begin(), agentsMonitored_.end(), it->second->getId()) == agentsMonitored_.end()))
              agentsManager_.addMonitoredAgent(it->first);

  // If we stop to monitor all humans, we add them to the agentsMonitored_ vector
  if (agentsManager_.stopMonitorAllHumans())
      for (std::map<std::string, Human*>::iterator it = humansMap_.begin(); it != humansMap_.end(); ++it)
          if ((it->second->getId() != "") && (std::find(agentsMonitored_.begin(), agentsMonitored_.end(), it->second->getId()) != agentsMonitored_.end()))
              agentsManager_.removeMonitoredAgent(it->first);

  // If we stop to monitor all robots, we add them to the agentsMonitored_ vector
  if (agentsManager_.stopMonitorAllRobots())
      for (std::map<std::string, Robot*>::iterator it = robotsMap_.begin(); it != robotsMap_.end(); ++it)
          if ((it->second->getId() != "") && (std::find(agentsMonitored_.begin(), agentsMonitored_.end(), it->second->getId()) != agentsMonitored_.end()))
              agentsManager_.removeMonitoredAgent(it->first);

  // Reload monitored agents
  agentsMonitored_ = agentsManager_.getMonitoredAgents();
}

void AgentMonitor::updateUnmonitoredEntitieTRBuffer()
{
  updateUnmonitoredEntitieTRBuffer(humansMap_);
  updateUnmonitoredEntitieTRBuffer(robotsMap_);
  updateUnmonitoredEntitieTRBuffer(objectsMap_);
}

bool AgentMonitor::updateAgentTRBuffer(Agent* agent)
{
  std::map<std::string, TRBuffer < Entity* > >::iterator itTRB = mapTRBEntity_.find(agent->getId());
  if (itTRB == mapTRBEntity_.end())
  {
    //1st time, we initialize variables
    TRBuffer<Entity*> buffAgnt;
    buffAgnt.push_back(agent->getTime(), agent);
    mapTRBEntity_[agent->getId()] = buffAgnt;

    // This module is made for temporal reasoning.
    // We need more data to make computation, so we will end the loop here.
  }
  else // Agent is present in agentsMonitor_.mapTRBEntity_
  {
    // If this is a new data we add it to the buffer
    if(agent->getId() != "")
    {
      if(mapTRBEntity_[agent->getId()].back()->getTime() < agent->getTime())
      {
        mapTRBEntity_[agent->getId()].push_back(agent->getTime(), agent);
        return true;
      }
    }
  }
  return false;
}

Agent* AgentMonitor::getMonitoredAgent(const std::string id)
{
  if (robotsMap_.find(id) != robotsMap_.end())
    return robotsMap_[id];
  else if (humansMap_.find(id) != humansMap_.end())
    return humansMap_[id];
  else
    return nullptr; // objects
}

void AgentMonitor::computeLookingFacts(Agent* agent, double lookTwdDeltaDist, double lookTwdAngularAperture, toaster_msgs::FactList& factList_msg)
{
  std::map<std::string, double> mapIdValue;
  mapIdValue = LookingFact::compute(mapTRBEntity_, agent->getId(), lookTwdDeltaDist, lookTwdAngularAperture);
  std::cout << "toward " << mapIdValue.size() << std::endl;
  LookingFact::createTowardFact(mapIdValue, factList_msg,
                                lookTwdAngularAperture,
                                agent->getId(),
                                mapTRBEntity_[agent->getId()].back());

  mapIdValue = LookingFact::compute(mapTRBEntity_, agent->getId(), lookTwdDeltaDist*0.5, lookTwdAngularAperture*0.25);
  std::cout << "at " << mapIdValue.size() << std::endl;
  LookingFact::createAtFact(mapIdValue, factList_msg,
                                lookTwdAngularAperture*0.25,
                                agent->getId(),
                                mapTRBEntity_[agent->getId()].back());
}
