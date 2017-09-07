#include "toaster_msgs/Fact.h"
#include "toaster-lib/Joint.h"
#include "toaster-lib/Entity.h"
#include "toaster-lib/TRBuffer.h"

#include <map>
#include <vector>
#include <string>

using namespace std;

class FactCreator
{
public:
  static toaster_msgs::Fact setFactBase(Joint* joint);
  static toaster_msgs::Fact setFactBase(string agent, map<string, TRBuffer < Entity* > >& mapTRBEntity);

  static toaster_msgs::Fact setMotionFact(toaster_msgs::Fact baseFact, double speed, double confidence, string type = "agent");
  static toaster_msgs::Fact setDirectionFact(toaster_msgs::Fact baseFact, string target, double confidence);
  static toaster_msgs::Fact setDistanceFact(toaster_msgs::Fact baseFact, string target, double confidence);
};
