#include "AgentManager.h"

using namespace std;

AgentManager::AgentManager() : monitorAllHumans_(false), monitorAllRobots_(false)
{

}

void AgentManager::init(ros::NodeHandle* node)
{
  node_ = node;
  serviceAdd_ = node_->advertiseService("agent_monitor/add_agent", &AgentManager::addAgent, this);
  ROS_INFO("[Request] Ready to add agent to monitor.");

  serviceRemove_ = node_->advertiseService("agent_monitor/remove_agent", &AgentManager::removeAgent, this);
  ROS_INFO("[Request] Ready to remove monitored agent.");

  serviceRemoves_ = node_->advertiseService("agent_monitor/remove_all_agents", &AgentManager::removeAllAgents, this);
  ROS_INFO("[Request] Ready to remove monitored agents.");

  serviceAddJoint_ = node_->advertiseService("agent_monitor/add_joint_to_agent", &AgentManager::addJointToAgent, this);
  ROS_INFO("[Request] Ready to add joint to monitor for agent.");

  serviceRemoveJt_ = node_->advertiseService("agent_monitor/remove_joint_to_agent", &AgentManager::removeJointToAgent, this);
  ROS_INFO("[Request] Ready to remove monitored joint to agent.");

  serviceRemoveJts_ = node_->advertiseService("agent_monitor/remove_all_joints_to_agent", &AgentManager::removeAllJointsToAgent, this);
  ROS_INFO("[Request] Ready to remove monitored joints to agent.");

  servicePrintMonitored_ = node_->advertiseService("agent_monitor/print_all_monitored_agents", &AgentManager::printAllMonitoredAgents, this);
  ROS_INFO("[Request] Ready to print monitored agents.");

  serviceMonitorAllAgents_ = node_->advertiseService("agent_monitor/monitor_all_agents", &AgentManager::monitorAllAgents, this);
  ROS_INFO("[Request] Ready to monitor all agents.");

  serviceMonitorAllHumans_ = node_->advertiseService("agent_monitor/monitor_all_humans", &AgentManager::monitorAllHumans, this);
  ROS_INFO("[Request] Ready to monitor all humans.");

  serviceMonitorAllRobots_ = node_->advertiseService("agent_monitor/monitor_all_robots", &AgentManager::monitorAllRobots, this);
  ROS_INFO("[Request] Ready to monitor all robots.");
}

bool AgentManager::addMonitoredAgent(string id)
{
    if (std::find(agentsMonitored_.begin(), agentsMonitored_.end(), id) == agentsMonitored_.end())
    {
      ROS_INFO("[agent_monitor][INFO] Agent %s now monitored", id.c_str());
      agentsMonitored_.push_back(id);
    }
    else
      ROS_INFO("[agent_monitor][INFO] Agent %s is already monitored", id.c_str());
}

bool AgentManager::addAgent(toaster_msgs::AddAgent::Request &req,
                            toaster_msgs::AddAgent::Response & res)
{
  if (req.id != "")
    res.answer = addMonitoredAgent(req.id);
  else
  {
    ROS_INFO("[agent_monitor][Request][WARNING] request to add agent with "
            "no id specified, sending back response: false");
    res.answer = false;
  }
  return true;
}

bool AgentManager::addJointToAgent(toaster_msgs::AddJointToAgent::Request &req,
                                  toaster_msgs::AddJointToAgent::Response & res)
{
  if(req.agentId == "")
  {
    ROS_INFO("[agent_monitor][Request][WARNING] request to addJoint with no id specified, sending back response: false");
    res.answer = false;
  }
  else if(std::find(agentsMonitored_.begin(), agentsMonitored_.end(), req.agentId) == agentsMonitored_.end())
  {
    ROS_INFO("[agent_monitor][Request][WARNING] Agent with id %s is not "
            "monitored, can't monitor his joint. Sending back response: false", req.agentId.c_str());
    res.answer = false;
  }
  else if(req.jointName == "")
  {
    ROS_INFO("[agent_monitor][Request][WARNING] request to add joint "
            "to agent with id %s with no joint name specified, sending back response: false", req.agentId.c_str());
    res.answer = false;
  }
  else if(mapAgentToJointsMonitored_.find(req.agentId) == mapAgentToJointsMonitored_.end())
  {
    ROS_INFO("[agent_monitor][Request][INFO] Now monitoring joint "
            "%s of agent with id %s , sending back response: true", req.jointName.c_str(), req.agentId.c_str());
    res.answer = true;
    vector<string> tmpVec(1, req.jointName);
    mapAgentToJointsMonitored_[req.agentId] = tmpVec;
  }
  else if(std::find(mapAgentToJointsMonitored_[req.agentId].begin(),
                    mapAgentToJointsMonitored_[req.agentId].end(), req.jointName)
          != mapAgentToJointsMonitored_[req.agentId].end())
  {
    ROS_INFO("[agent_monitor][Request][INFO] Joint %s of agent "
            "with id %s already monitored, sending back response: false", req.jointName.c_str(), req.agentId.c_str());
    res.answer = false;
  }
  else
  {
    ROS_INFO("[agent_monitor][Request][INFO] Now monitoring joint "
            "%s of agent with id %s , sending back response: true", req.jointName.c_str(), req.agentId.c_str());
    res.answer = true;
    mapAgentToJointsMonitored_[req.agentId].push_back(req.jointName);
  }
  return true;
}

bool AgentManager::removeMonitoredAgent(string id)
{
  if (std::find(agentsMonitored_.begin(), agentsMonitored_.end(), id) == agentsMonitored_.end())
  {
    ROS_INFO("[agent_monitor][INFO] Agent %s no more monitored", id.c_str());
    agentsMonitored_.erase(std::remove(agentsMonitored_.begin(), agentsMonitored_.end(), id), agentsMonitored_.end());
  }
  else
    ROS_INFO("[agent_monitor][INFO] Agent %s is already not monitored", id.c_str());
}

bool AgentManager::removeAgent(toaster_msgs::RemoveAgent::Request &req,
                              toaster_msgs::RemoveAgent::Response & res)
{
  if (req.id != "")
    res.answer = removeMonitoredAgent(req.id);
  else
  {
    ROS_INFO("[agent_monitor][Request][WARNING] request to remove agent with "
            "no id specified, sending back response: false");
    res.answer = false;
  }
  return true;
}

bool AgentManager::removeAllAgents(toaster_msgs::Empty::Request &req,
                                  toaster_msgs::Empty::Response & res)
{
  agentsMonitored_.clear();
  ROS_INFO("[agent_monitor][Request][WARNING] request to remove all agents");
  return true;
}

bool AgentManager::removeJointToAgent(toaster_msgs::RemoveJointToAgent::Request &req,
                                      toaster_msgs::RemoveJointToAgent::Response & res)
{
  if(req.agentId == "")
  {
    ROS_INFO("[agent_monitor][Request][WARNING] request to remove agent's joint "
            "with no id  specified, sending back response: false");
    res.answer = false;
  }
  else if(std::find(agentsMonitored_.begin(), agentsMonitored_.end(), req.agentId) == agentsMonitored_.end())
  {
    ROS_INFO("[agent_monitor][Request][WARNING] Agent with id %s is not "
            "monitored, can't stop monitoring his joint. Sending back response: false", req.agentId.c_str());
    res.answer = false;
  }
  else if(req.jointName == "")
  {
    ROS_INFO("[agent_monitor][Request][WARNING] request to remove joint "
            "to agent with id %s with no name specified, sending back response: false", req.agentId.c_str());
    res.answer = false;
  }
  else if(std::find(mapAgentToJointsMonitored_[req.agentId].begin(),
          mapAgentToJointsMonitored_[req.agentId].end(), req.jointName) == mapAgentToJointsMonitored_[req.agentId].end())
  {
    ROS_INFO("[agent_monitor][Request][INFO] Joint %s of agent with "
            "id %s already not monitored, sending back response: false", req.jointName.c_str(), req.agentId.c_str());
    res.answer = false;
  }
  else
  {
    ROS_INFO("[agent_monitor][Request][INFO] Joint %s of agent with id %s"
            " not monitored anymore, sending back response: true", req.jointName.c_str(), req.agentId.c_str());
    res.answer = true;
    mapAgentToJointsMonitored_[req.agentId].erase(std::remove(mapAgentToJointsMonitored_[req.agentId].begin(),
            mapAgentToJointsMonitored_[req.agentId].end(), req.jointName), mapAgentToJointsMonitored_[req.agentId].end());
  }
  return true;
}

bool AgentManager::removeAllJointsToAgent(toaster_msgs::RemoveAllJointsToAgent::Request &req,
                                          toaster_msgs::RemoveAllJointsToAgent::Response & res)
{
  if (req.agentId == "")
  {
    ROS_INFO("[agent_monitor][Request][WARNING] request to remove agent joints' "
            "with no id and no name specified, sending back response: false");
    res.answer = false;
  }
  else if(find(agentsMonitored_.begin(), agentsMonitored_.end(), req.agentId) == agentsMonitored_.end())
  {
    ROS_INFO("[agent_monitor][Request][INFO] Agent with id %s is already "
            "not monitored, sending back response: false", req.agentId.c_str());
    res.answer = false;
  }
  else
  {
    ROS_INFO("[agent_monitor][Request][INFO] Agent with id %s joint's not "
            "monitored anymore, sending back response: true", req.agentId.c_str());
    mapAgentToJointsMonitored_[req.agentId].clear();
    res.answer = true;
  }
  return true;
}

bool AgentManager::printAllMonitoredAgents(toaster_msgs::Empty::Request &req,
                                          toaster_msgs::Empty::Response & res)
{
    ROS_INFO("[agent_monitor][Request][PRINT] ****** Agents Monitored *******");
    for (std::vector<std::string>::iterator it = agentsMonitored_.begin(); it != agentsMonitored_.end(); ++it)
    {
        ROS_INFO("[agent_monitor][Request][PRINT] Agent id: %s", (*it).c_str());
        for (vector<string>::iterator itJoint = mapAgentToJointsMonitored_[*it].begin();
                itJoint != mapAgentToJointsMonitored_[*it].end(); ++itJoint) {
            ROS_INFO("[agent_monitor][Request][PRINT] Joint Monitored name: %s", (*itJoint).c_str());
        }
    }
    return true;
}

bool AgentManager::monitorAllAgents(toaster_msgs::MonitorAll::Request &req,
                                    toaster_msgs::MonitorAll::Response & res)
{
    monitorAllHumans_ = req.monitorAll;
    monitorAllRobots_ = req.monitorAll;
    if (req.monitorAll)
        ROS_INFO("[agent_monitor][REQUEST] Start monitoring all agents");
    else
        ROS_INFO("[agent_monitor][REQUEST] Stop monitoring all agents");
    return true;
}

bool AgentManager::monitorAllHumans(toaster_msgs::MonitorAll::Request &req,
                                    toaster_msgs::MonitorAll::Response & res)
{
    monitorAllHumans_ = req.monitorAll;
    if (req.monitorAll)
        ROS_INFO("[agent_monitor][REQUEST] Start monitoring all humans");
    else
        ROS_INFO("[agent_monitor][REQUEST] Stop monitoring all humans");
    return true;
}

bool AgentManager::monitorAllRobots(toaster_msgs::MonitorAll::Request &req,
                                    toaster_msgs::MonitorAll::Response & res)
{
    monitorAllRobots_ = req.monitorAll;
    if (req.monitorAll)
        ROS_INFO("[agent_monitor][REQUEST] Start monitoring all robots");
    else
        ROS_INFO("[agent_monitor][REQUEST] Stop monitoring all robots");
    return true;
}
