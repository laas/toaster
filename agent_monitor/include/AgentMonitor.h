#ifndef AGENTMONITOR_H
#define AGENTMONITOR_H

#include <map>
#include <string>

#include "toaster-lib/TRBuffer.h"
#include "toaster-lib/Entity.h"
#include "toaster-lib/Human.h"
#include "toaster-lib/Robot.h"
#include "toaster-lib/Object.h"

#include "toaster_msgs/FactList.h"

#include "AgentManager.h"

class AgentMonitor
{
public:
  void init(ros::NodeHandle* node);

  void setHumanMap(const std::map<std::string, Human*>& map) { humansMap_ = map; }
  void setRobotMap(const std::map<std::string, Robot*>& map) { robotsMap_ = map; }
  void setObjectMap(const std::map<std::string, Object*>& map) { objectsMap_ = map; }

public:

  // Map of Timed Ring Buffer Entities
  std::map<std::string, TRBuffer < Entity* > > mapTRBEntity_;

  std::map<std::string, Human*> humansMap_;
  std::map<std::string, Robot*> robotsMap_;
  std::map<std::string, Object*> objectsMap_;

  AgentManager agentsManager_;
  std::vector<std::string> agentsMonitored_;
  std::map<std::string, std::vector<std::string> > mapAgentToJointsMonitored_;

  void updateMonitored();
  void updateUnmonitoredEntitieTRBuffer();
  bool updateAgentTRBuffer(Agent* agent);

  Agent* getMonitoredAgent(const std::string id);

  void computeLookingFacts(Agent* agent, double lookTwdDeltaDist_, double lookTwdAngularAperture_, toaster_msgs::FactList& factList_msg);

  template<typename T>
  void updateUnmonitoredEntitieTRBuffer(const std::map<std::string, T*>& entity_map)
  {
    for (auto it = entity_map.begin(); it != entity_map.end(); ++it)
    {
      if (std::find(agentsMonitored_.begin(), agentsMonitored_.end(), it->first) == agentsMonitored_.end())
      {
        std::map<std::string, TRBuffer < Entity* > >::iterator itTRB = mapTRBEntity_.find(it->first);
        T* ent = it->second;

        if(itTRB == mapTRBEntity_.end()) // If 1st data
        {
          TRBuffer<Entity*> buff;
          buff.push_back(ent->getTime(), ent);
          mapTRBEntity_[it->first] = buff;
        }
        else if(mapTRBEntity_[it->first].back()->getTime() < it->second->getTime())
          mapTRBEntity_[it->first].push_back(ent->getTime(), ent);
      }
    }
  }
private:
};

#endif // AGENTMONITOR_H
