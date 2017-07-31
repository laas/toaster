#ifndef HUMANPUBLISHER_H
#define HUMANPUBLISHER_H

#include <toaster_msgs/SetEntityPose.h>
#include <toaster_msgs/FactList.h>
#include <toaster_msgs/HumanListStamped.h>

#include "pdg/readers/MorseHumanReader.h"
#include "pdg/readers/MocapHumanReader.h"
#include "pdg/readers/AdreamMocapHumanReader.h"
#include "pdg/readers/GroupHumanReader.h"
#include "pdg/readers/ToasterSimuHumanReader.h"

#include "pdg/types.h"

void PublishHuman(MorseHumanReader& morseHumanRd, Entity& newPoseEnt_,
                       struct toasterList_t& list_msg);

void PublishHuman(MocapHumanReader& mocapHumanRd, Entity& newPoseEnt_,
                      struct toasterList_t& list_msg);

void PublishHuman(AdreamMocapHumanReader& adreamMocapHumanRd,
                      Entity& newPoseEnt_,
                      struct toasterList_t& list_msg);

void PublishHuman(GroupHumanReader& groupHumanRd, Entity& newPoseEnt_,
                      struct toasterList_t& list_msg);

void PublishHuman(ToasterSimuHumanReader& toasterSimuHumanRd,
                      Entity& newPoseEnt_,
                      struct toasterList_t& list_msg);

#endif
