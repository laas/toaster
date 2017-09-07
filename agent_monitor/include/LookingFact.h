#include <map>
#include <string>

#include "toaster-lib/TRBuffer.h"
#include "toaster-lib/Entity.h"
#include "toaster_msgs/FactList.h"

using namespace std;

class LookingFact
{
public:
  /**
   * @brief This function compute the map required by the fact "IsLookingToward"
   *        by testing if an entity is lying in the 3D cone of agent visual attention
   * @param Entity map containing all entity involved in the joint action & their
   *        respective ids
   * @param Monitored agent id
   * @param Distance between center of basement circle and agent head position
   * @param Angular aperture of the cone in radians
   * @return Map required by the fact "IsLookingToward" containing all entities
   *         lying in the cone and all normalized angles beetween entities and cone axis
   */
  static map<string, double> compute(map<string, TRBuffer < Entity* > > mapEnts,
                                      string agentMonitored, double deltaDist,
                                      double angularAperture);

  static void createTowardFact(map<string, double> mapIdValue, toaster_msgs::FactList &factList_msg,
                                double angle, string subjectId, Entity* entity);

  static void createAtFact(map<string, double> mapIdValue, toaster_msgs::FactList &factList_msg,
                          double angle, string subjectId, Entity* entity);

  static void createFact(map<string, double> mapIdValue, toaster_msgs::FactList &factList_msg,
                        double angle, string subjectId, Entity* entity, string property);
};
