// This main will compute the facts for the requested agent.

#include "SPAR/PDGHumanReader.h"
#include "SPAR/PDGRobotReader.h"
//#include "SPAR/PDGObjectReader.h"
#include "toaster-lib/TRBuffer.h"
#include "toaster-lib/MathFunctions.h"

bool computeMotion2D(TRBuffer< Entity* >& confBuffer, unsigned long timelapse, unsigned int distanceThreshold) {
    int index;
    double dist = 0.0;
    long actualTimelapse = 0;
    long timeNew = confBuffer.getTimeFromIndex(confBuffer.size() - 1);
    long timeOld = timeNew - timelapse;
    Entity* entNew = confBuffer.getDataFromIndex(confBuffer.size() - 1);

    index = confBuffer.getIndexAfter(timeOld);
    actualTimelapse = timeNew - confBuffer.getTimeFromIndex(index); // Actual timelapse
    Entity* entOld = confBuffer.getDataFromIndex(index);

    dist = bg::distance(MathFunctions::convert3dTo2d(entNew->getPosition()), MathFunctions::convert3dTo2d(entOld->getPosition()));

    // std::cout << "Distance is " << dist << std::endl;
    //		std::cout << "ds*actualTimeLapse / timelapse " << distanceThreshold * actualTimelapse / timelapse << std::endl;
    //	std::cout << "actual timelapse "<< actualTimelapse << std::endl;
    if (dist < distanceThreshold * actualTimelapse / timelapse) {
        return false;
    } else {
        return true;
    }
}

double computeMotion2DDirection(TRBuffer< Entity* >& confBuffer, unsigned long timelapse) {
    double towardAngle;
    int index;
    //long actualTimelapse = 0;
    long timeNew = confBuffer.getTimeFromIndex(confBuffer.size() - 1);
    long timeOld = timeNew - timelapse;
    Entity* entNew = confBuffer.getDataFromIndex(confBuffer.size() - 1);

    index = confBuffer.getIndexAfter(timeOld);
    //actualTimelapse = timeNew - confBuffer.getTimeFromIndex(index);   // Actual timelapse
    Entity* entOld = confBuffer.getDataFromIndex(index);
    towardAngle = acos(fabs(entOld->getPosition().get<0>() - entNew->getPosition().get<0>())
            / bg::distance(MathFunctions::convert3dTo2d(entOld->getPosition()), MathFunctions::convert3dTo2d(entNew->getPosition())));

    // Trigonometric adjustment
    if (entNew->getPosition().get<0>() < entOld->getPosition().get<0>())
        towardAngle = 3.1416 - towardAngle;

    if (entNew->getPosition().get<1>() < entOld->getPosition().get<1>())
        towardAngle = -towardAngle;

    return towardAngle;
}

std::map<unsigned int, double> motion2DToward(std::map<unsigned int, TRBuffer < Entity* > >& mapEnts,
        unsigned int agentMonitored, double towardAngle, double angleThreshold) {

    std::map<unsigned int, double> towardConfidence;

    //For each entities in the same room
    for (std::map<unsigned int, TRBuffer < Entity*> >::iterator it = mapEnts.begin(); it != mapEnts.end(); ++it) {
        if (it->second.back()->getId() != agentMonitored)
            towardConfidence[it->second.back()->getId()] = MathFunctions::isInAngle(mapEnts[agentMonitored].back(), it->second.back(), towardAngle, angleThreshold);
    }
    return towardConfidence;
}

void initTRBuffer(unsigned int agentMonitored, TRBuffer<Entity*>& TRBEntity, unsigned int historyLength) {
    //We need to initiate the ringbuffer

    TRBuffer<Entity*> mybuffer(historyLength);
    TRBEntity = mybuffer;
}

int main(int argc, char** argv) {
    // Set this in a ros service
    const bool AGENT_FULL_CONFIG = false; //If false we will use only position and orientation
    unsigned int roomOfInterest = 0;

    // Set this in a ros service
    // TODO: Make it a vector?
    unsigned int agentMonitored = 101;
    bool humanMonitored = agentMonitored - 100;

    // Map of Timed Ring Buffer Entities
    std::map<unsigned int, TRBuffer < Entity* > > mapTRBEntity;


    ros::init(argc, argv, "AGENT_MONITOR");
    ros::NodeHandle node;

    //Data reading
    PDGHumanReader humanRd(node, AGENT_FULL_CONFIG);
    PDGRobotReader robotRd(node, AGENT_FULL_CONFIG);
    //PDGObjectReader objectRd(node);

    // Set this in a ros service?
    ros::Rate loop_rate(30);


    /************************/
    /* Start of the Ros loop*/
    /************************/

    while (node.ok()) {
        // We received agentMonitored

        //////////////////////////////////////
        //           Updating data          //
        //////////////////////////////////////

        if ((!humanMonitored && (robotRd.lastConfig_[agentMonitored] != NULL))
                || (humanMonitored && (humanRd.lastConfig_[agentMonitored] != NULL))) {

            // We add the agent to the mapTRBEntity and update roomOfInterest
            if (humanMonitored) {
                // If this is a new data we add it
                if ((mapTRBEntity[agentMonitored].empty()) || (mapTRBEntity[agentMonitored].back()->getTime() < humanRd.lastConfig_[agentMonitored]->getTime()))
                    mapTRBEntity[agentMonitored].push_back(humanRd.lastConfig_[agentMonitored]->getTime(), humanRd.lastConfig_[agentMonitored]);
                roomOfInterest = humanRd.lastConfig_[agentMonitored]->getRoomId();
            } else {
                // If this is a new data we add it
                if ((mapTRBEntity[agentMonitored].empty()) || (mapTRBEntity[agentMonitored].back()->getTime() < robotRd.lastConfig_[agentMonitored]->getTime()))
                    mapTRBEntity[agentMonitored].push_back(robotRd.lastConfig_[agentMonitored]->getTime(), robotRd.lastConfig_[agentMonitored]);
                roomOfInterest = robotRd.lastConfig_[agentMonitored]->getRoomId();
            }

            // for each entity
            //Put the following in a function?

            // For humans
            for (std::map<unsigned int, Human*>::iterator it = humanRd.lastConfig_.begin(); it != humanRd.lastConfig_.end(); ++it) {
                // if in same room as monitored agent and not monitored agent
                if (roomOfInterest == it->second->getRoomId() && it->second->getId() != agentMonitored) {
                    // If this is a new data we add it
                    if ((mapTRBEntity[it->second->getId()].empty()) || mapTRBEntity[it->second->getId()].back()->getTime() < it->second->getTime())
                        mapTRBEntity[it->second->getId()].push_back(it->second->getTime(), it->second);
                } // TODO: else remove

            }

            // For robots

            for (std::map<unsigned int, Robot*>::iterator it = robotRd.lastConfig_.begin(); it != robotRd.lastConfig_.end(); ++it) {
                // if in same room as monitored agent and not monitored agent
                if ((roomOfInterest == it->second->getRoomId()) && (it->second->getId() != agentMonitored)) {
                    // If this is a new data we add it
                    if ((mapTRBEntity[it->second->getId()].empty()) || mapTRBEntity[it->second->getId()].back()->getTime() < it->second->getTime())
                        mapTRBEntity[it->second->getId()].push_back(it->second->getTime(), it->second);
                } // TODO: else remove

            }

            //  For Objects

            //////////////////////////////////////////////
            // Compute facts concerning monitored agent //
            //////////////////////////////////////////////




        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
