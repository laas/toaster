#ifndef AGENTMONITOR_H
#define AGENTMONITOR_H

#include <map>
#include <string>

#include "toaster-lib/TRBuffer.h"
#include "toaster-lib/Entity.h"

class AgentMonitor
{
public:

  // Map of Timed Ring Buffer Entities
  std::map<std::string, TRBuffer < Entity* > > mapTRBEntity_;

  template<typename T>
  void updateUnmonitoredEntitieTRBuffer(const std::map<std::string, T*>& entity_map, const std::vector<std::string>& agents_monitored)
  {
    for (auto it = entity_map.begin(); it != entity_map.end(); ++it)
    {
      if (std::find(agents_monitored.begin(), agents_monitored.end(), it->first) == agents_monitored.end())
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
