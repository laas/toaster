#ifndef ROBOTPUBLISHER_H
#define ROBOTPUBLISHER_H

#include <toaster_msgs/SetEntityPose.h>
#include <toaster_msgs/FactList.h>
#include <toaster_msgs/RobotListStamped.h>

#include "pdg/readers/Pr2RobotReader.h"
#include "pdg/readers/SpencerRobotReader.h"
#include "pdg/readers/ToasterSimuRobotReader.h"

#include "pdg/types.h"

void PublishRobot(Pr2RobotReader& pr2RobotRd,
                     struct toasterList_t& list_msg,
                     bool FullConfig = false);

void PublishRobot(SpencerRobotReader& spencerRobotRd,
                    struct toasterList_t& list_msg,
                    bool FullConfig = false);

void PublishRobot(ToasterSimuRobotReader& toasterSimuRobotRd,
                    struct toasterList_t& list_msg,
                    bool FullConfig = false);

#endif
