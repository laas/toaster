#include "Motion2D.h"

#include "toaster-lib/MathFunctions.h"
#include "toaster-lib/Agent.h"

using namespace std;

map<string, double> Motion2D::computeToward(map<string, TRBuffer < Entity* > > mapEnts,
                                            string agentMonitored,
                                            double towardAngle, double angleThreshold,
                                            string jointName)
{
  map<string, double> towardConfidence;

  //For each entities in the same room
  for (map<string, TRBuffer < Entity*> >::iterator it = mapEnts.begin(); it != mapEnts.end(); ++it)
  {
    if (it->first != agentMonitored)
    {
      double unused = 0;
      double curConf = MathFunctions::isInAngle(getEntityOrJoint(mapEnts[agentMonitored].back(), jointName),
                                                it->second.back(), towardAngle, angleThreshold, unused);
      if (curConf > 0.0)
        towardConfidence[it->first] = curConf;
    }
  }
  return towardConfidence;
}

double Motion2D::computeDirection(TRBuffer< Entity* > confBuffer,
                                  unsigned long timelapse,
                                  string jointName)
{
    long timeNew = confBuffer.getTimeFromIndex(confBuffer.size() - 1);
    long timeOld = timeNew - timelapse;

    Entity* entNew = getEntityOrJoint(confBuffer.getDataFromIndex(confBuffer.size() - 1), jointName);

    int index = confBuffer.getIndexAfter(timeOld);

    Entity* entOld = getEntityOrJoint(confBuffer.getDataFromIndex(index), jointName);
    double towardAngle = acos(fabs(entOld->getPosition().get<0>() - entNew->getPosition().get<0>())
            / bg::distance(MathFunctions::convert3dTo2d(entOld->getPosition()),
            MathFunctions::convert3dTo2d(entNew->getPosition())));

    // Trigonometric adjustment
    if (entNew->getPosition().get<0>() < entOld->getPosition().get<0>())
        towardAngle = 3.1416 - towardAngle;

    if (entNew->getPosition().get<1>() < entOld->getPosition().get<1>())
        towardAngle = -towardAngle;

    return towardAngle;
}

double Motion2D::compute(TRBuffer< Entity* > confBuffer,
                              unsigned long timelapse, string jointName)
{
    long timeNew = confBuffer.getTimeFromIndex(confBuffer.size() - 1);
    long timeOld = timeNew - timelapse;

    Entity* entNew = getEntityOrJoint(confBuffer.back(), jointName);

    int index = confBuffer.getIndexAfter(timeOld);
    if (index == -1) // In case we don't have the index, we will just put isMoving to false
        return false;

    long actualTimelapse = timeNew - confBuffer.getTimeFromIndex(index); // Actual timelapse

    Entity* entOld = getEntityOrJoint(confBuffer.getDataFromIndex(index), jointName);

    double dist = bg::distance(MathFunctions::convert3dTo2d(entNew->getPosition()),
                              MathFunctions::convert3dTo2d(entOld->getPosition()));

    unsigned long oneSecond = pow(10, 9);
    return dist * oneSecond / actualTimelapse;
}

bool Motion2D::computeIsMoving(TRBuffer< Entity* > confBuffer,
                              unsigned long timelapse, double distanceThreshold,
                              string jointName)
{
    long timeNew = confBuffer.getTimeFromIndex(confBuffer.size() - 1);
    long timeOld = timeNew - timelapse;

    Entity* entNew = getEntityOrJoint(confBuffer.back(), jointName);

    int index = confBuffer.getIndexAfter(timeOld);
    if (index == -1) // In case we don't have the index, we will just put isMoving to false
        return false;

    long actualTimelapse = timeNew - confBuffer.getTimeFromIndex(index); // Actual timelapse
    Entity* entOld = getEntityOrJoint(confBuffer.getDataFromIndex(index), jointName);

    double dist = bg::distance(MathFunctions::convert3dTo2d(entNew->getPosition()),
                              MathFunctions::convert3dTo2d(entOld->getPosition()));

    if (dist < distanceThreshold * actualTimelapse / timelapse)
        return false;
    else
        return true;
}

Entity* Motion2D::getEntityOrJoint(Entity* entity, string jointName)
{
  Entity* returnEntity = nullptr;
  if(jointName == "")
    returnEntity = entity;
  else
    returnEntity = ((Agent*) entity)->skeleton_[jointName];
  return returnEntity;
}
