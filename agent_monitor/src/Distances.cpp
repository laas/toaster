#include "Distances.h"

#include "toaster-lib/MathFunctions.h"
#include "toaster-lib/Agent.h"

using namespace std;

map<string, double> Distances::computeDeltaDist(map<string, TRBuffer < Entity* > > mapEnts,
                                              string agentMonitored, unsigned long timelapse)
{
  map<string, double> deltaDistMap;

  //For each entities in the same room
  for (map<string, TRBuffer < Entity*> >::iterator it = mapEnts.begin(); it != mapEnts.end(); ++it)
  {
    if (it->first != agentMonitored)
    {
      // We compute the current distance
      Entity* entCur = it->second.back();
      Entity* entMonitoredCur = mapEnts[agentMonitored].back();

      //Put this in a function
      double curDist = bg::distance(MathFunctions::convert3dTo2d(entCur->getPosition()),
                                    MathFunctions::convert3dTo2d(entMonitoredCur->getPosition()));

      // We compute the distance at now - timelapse
      unsigned long timeCur = entMonitoredCur->getTime();
      unsigned long timePrev = timeCur - timelapse;

      Entity* entMonitoredPrev = mapEnts[agentMonitored].getDataFromIndex(mapEnts[agentMonitored].getIndexAfter(timePrev));

      double prevDist = bg::distance(MathFunctions::convert3dTo2d(entCur->getPosition()),
                                    MathFunctions::convert3dTo2d(entMonitoredPrev->getPosition()));

      //We compute Deltadist
      double deltaDist = prevDist - curDist;

      // We fill towardConfidence
      deltaDistMap[it->first] = deltaDist;
    }
  }
  return deltaDistMap;
}

map<string, double> Distances::computeJointDeltaDist(map<string, TRBuffer < Entity* > > mapEnts,
                                                    string agentMonitored, string jointName,
                                                    unsigned long timelapse)
{
  map<string, double> deltaDistMap;

  //For each entities in the same room
  for (map<string, TRBuffer < Entity*> >::iterator it = mapEnts.begin(); it != mapEnts.end(); ++it)
  {
    if (it->first != agentMonitored)
    {
      // We compute the current distance
      Entity* entCur = it->second.back();
      Entity* entMonitoredCur = ((Agent*) mapEnts[agentMonitored].back())->skeleton_[jointName];

      //Put this in a function
      double curDist = bg::distance(MathFunctions::convert3dTo2d(entCur->getPosition()),
                                    MathFunctions::convert3dTo2d(entMonitoredCur->getPosition()));

      // We compute the distance at now - timelapse
      unsigned long timeCur = entMonitoredCur->getTime();
      unsigned long timePrev = timeCur - timelapse;

      Entity* entMonitoredPrev = ((Agent*) mapEnts[agentMonitored].getDataFromIndex(
              mapEnts[agentMonitored].getIndexAfter(timePrev)))->skeleton_[jointName];

      double prevDist = bg::distance(MathFunctions::convert3dTo2d(entCur->getPosition()),
                                    MathFunctions::convert3dTo2d(entMonitoredPrev->getPosition()));

      //We compute Deltadist
      double deltaDist = curDist - prevDist;

      // We fill towardConfidence
      deltaDistMap[it->first] = deltaDist;
    }
  }
  return deltaDistMap;
}
