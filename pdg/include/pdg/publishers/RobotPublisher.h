#ifndef ROBOTPUBLISHER_H
#define ROBOTPUBLISHER_H

#include <toaster_msgs/SetEntityPose.h>
#include <toaster_msgs/FactList.h>
#include <toaster_msgs/RobotListStamped.h>

#include "pdg/readers/Pr2RobotReader.h"
#include "pdg/readers/SpencerRobotReader.h"
#include "pdg/readers/ToasterSimuRobotReader.h"

void PublishRobot(Pr2RobotReader& pr2RobotRd, Entity& newPoseEnt_,
                     toaster_msgs::FactList& factList_msg,
                     toaster_msgs::RobotListStamped& robotList_msg,
                     bool FullConfig = false);

void PublishRobot(SpencerRobotReader& spencerRobotRd, Entity& newPoseEnt_,
                    toaster_msgs::FactList& factList_msg,
                    toaster_msgs::RobotListStamped& robotList_msg,
                    bool FullConfig = false);

void PublishRobot(ToasterSimuRobotReader& toasterSimuRobotRd,
                    Entity& newPoseEnt_,
                    toaster_msgs::FactList& factList_msg,
                    toaster_msgs::RobotListStamped& robotList_msg,
                    bool FullConfig = false);

#endif
