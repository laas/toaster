#ifndef OBJECTPUBLISHER_H
#define OBJECTPUBLISHER_H

#include <toaster_msgs/SetEntityPose.h>
#include <toaster_msgs/FactList.h>
#include "toaster_msgs/ToasterObjectReader.h"

#include "pdg/readers/OM2MObjectReader.h"
#include "pdg/readers/ArObjectReader.h"
#include "pdg/readers/GazeboObjectReader.h"
#include "pdg/readers/ToasterSimuObjectReader.h"

#include "pdg/types.h"

using namespace std;

void PublishObject(ArObjectReader& arObjectRd,
                   struct objectIn_t& objectIn,
                   struct toasterList_t& list_msg);

void PublishObject(OM2MObjectReader& om2mObjectRd,
                  struct objectIn_t& objectIn,
                  struct toasterList_t& list_msg);

void PublishObject(GazeboObjectReader& gazeboRd,
                  struct objectIn_t& objectIn,
                  struct toasterList_t& list_msg);

void PublishObject(ToasterSimuObjectReader& toasterSimuObjectRd,
                    struct objectIn_t& objectIn,
                    struct toasterList_t& list_msg);

#endif
