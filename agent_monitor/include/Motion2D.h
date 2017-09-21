#include <string>
#include <map>

#include "toaster-lib/TRBuffer.h"
#include "toaster-lib/Entity.h"

using namespace std;

class Motion2D
{
public:
  static map<string, double> computeToward(map<string, TRBuffer < Entity* > > mapEnts,
                                          string agentMonitored,
                                          double towardAngle, double angleThreshold,
                                          string jointName = "");

  /*Compute Motion angle for agents (jointName == "") or joints (jointName != "")*/
  static double computeDirection(TRBuffer< Entity* > confBuffer, unsigned long timelapse,
                                string jointName = "");

  /*Compute Motion speed for agents (jointName == "") or joints (jointName != "")*/
  static double compute(TRBuffer< Entity* > confBuffer, unsigned long timelapse,
                        string jointName = "");

  /*Compute if is moving for agents (jointName == "") or joints (jointName != "")*/
  static bool computeIsMoving(TRBuffer< Entity* > confBuffer, unsigned long timelapse,
                              double distanceThreshold, string jointName = "");

private:
  static Entity* getEntityOrJoint(Entity* entity, string jointName);
};
