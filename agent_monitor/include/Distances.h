#include <string>
#include <map>

#include "toaster-lib/TRBuffer.h"
#include "toaster-lib/Entity.h"

using namespace std;

class Distances
{
public:
  static map<string, double> computeDeltaDist(map<string, TRBuffer < Entity* > > mapEnts,
                                              string agentMonitored, unsigned long timelapse);

  static map<string, double> computeJointDeltaDist(map<string, TRBuffer < Entity* > > mapEnts,
                                                  string agentMonitored, string jointName,
                                                  unsigned long timelapse);
};
