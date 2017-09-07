// This main will compute the facts for the requested agent.

#include "toaster_msgs/ToasterHumanReader.h"
#include "toaster_msgs/ToasterRobotReader.h"
#include "toaster_msgs/ToasterObjectReader.h"
#include "toaster_msgs/Empty.h"
#include "toaster_msgs/FactList.h"
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/PointingTime.h"
#include "toaster_msgs/Pointing.h"

#include "toaster-lib/TRBuffer.h"
#include "toaster-lib/MathFunctions.h"

#include <dynamic_reconfigure/server.h>
#include <agent_monitor/agent_monitorConfig.h>

#include "AgentManager.h"
#include "LookingFact.h"
#include "Distances.h"
#include "Motion2D.h"
#include "FactCreator.h"

AgentManager agentsManager_;

//For convinience
typedef dynamic_reconfigure::Server<agent_monitor::agent_monitorConfig> ParamServer_t;

std::map<std::string, std::vector<toaster_msgs::Fact> > previousAgentsFactList_;

// Compute motion:
unsigned long oneSecond_ = pow(10, 9);

// Map of Timed Ring Buffer Entities
static std::map<std::string, TRBuffer < Entity* > > mapTRBEntity_;

//Dyn config params def values
double lookTwdDeltaDist_ = 2.0;
double lookTwdAngularAperture_ = 2 * PI / 3;

//We consider motion when it moves more than 3 cm during 1/4 second, so when higher than 0.12 m/s
unsigned long motion2DBodyTime_ = oneSecond_ / 4;
double motion2DBodySpeedThreshold_ = 0.12; // this are m/s

double motion2DBodyDirTime_ = oneSecond_ / 2;
double motionTwd2DBodyAngleThresold_ = 1.0;

// We consider motion toward when it moves more than 3 cm during 1/4 second toward an item, so when higher than 0.12 m/s
unsigned long motionTwdBodyDeltaDistTime_ = oneSecond_ / 4;
double movingTwdBodyDeltaDistThreshold_ = 0.03;

//We consider motion when it moves more than 3 cm during 1/4 second, so when higher than 0.12 m/s
unsigned long motion2DJointTime_ = oneSecond_ / 4;
double motion2DJointSpeedThreshold_ = 0.12; // this are m/s

double motion2DJointDirTime_ = oneSecond_;
double motionTwd2DJointAngleThresold_ = 1.0;

// We consider motion toward when it moves more than 3 cm during 1/4 second toward an item, so when higher than 0.12 m/s
unsigned long motionTwdJointDeltaDistTime_ = oneSecond_ / 4;
double movingTwdJointDeltaDistThreshold_ = 0.03;


// Distance
double distReach_ = 0.05;
double distClose_ = 0.2;
double distMedium_ = 1.5;
double distFar_ = 8.0;

// Move this to a library?
// create a fact

/*toaster_msgs::Fact createFact(std::string property, std::string propertyType, std::string subProperty, ) {
    toaster_msgs::Fact fact_msg;
    fact_msg.property = property;
    fact_msg.subProperty = subProperty;
    fact_msg.subjectId = subjectId;
    fact_msg.subjectName = subjectName;
    fact_msg.targetId = targetId;
    fact_msg.targetName = targetName;
    fact_msg.confidence = confidence;
    fact_msg.time = time;
}*/

bool isPointing(Agent* agent, std::string pointingJoint, double pointingDistThreshold) {
    // if distance from body > threshold
    // TODO: check if agent got this joint
    double distBodyJoint = bg::distance(MathFunctions::convert3dTo2d(agent->getPosition()),
            MathFunctions::convert3dTo2d(agent->skeleton_[pointingJoint]->getPosition()));
    if (distBodyJoint > pointingDistThreshold)
        return true;
    else
        return false;
}

double computePointingAngle(Agent* agent, std::string pointingJoint) {
    // If joint has orientation, use it
    double orientation = agent->skeleton_[pointingJoint]->orientation_[2];
    if (orientation != 0.0)
        return orientation;
    else
        return acos(fabs(agent->getPosition().get<0>() - agent->skeleton_[pointingJoint]->getPosition().get<0>())
            / bg::distance(MathFunctions::convert3dTo2d(agent->getPosition()),
            MathFunctions::convert3dTo2d(agent->skeleton_[pointingJoint]->getPosition())));
}

std::map<std::string, double> computePointingToward(std::map<std::string, TRBuffer < Entity* > > mapEnts,
        std::string pointingAgent, std::string pointingJoint, unsigned long timePointing,
        double towardAngle, double angleThreshold) {
    std::map<std::string, double> towardConfidence;

    // This parameter won't be used here...
    double angleResult = 0.0;


    Agent* agent;
    int index = mapEnts[pointingAgent].getIndexAfter(timePointing);
    if (index != -1) {
        agent = (Agent*) mapEnts[pointingAgent].getDataFromIndex(index);

        //For each entities in the same room
        for (std::map<std::string, TRBuffer < Entity*> >::iterator it = mapEnts.begin(); it != mapEnts.end(); ++it) {
            Entity* curEnt;
            int index = mapEnts[it->first].getIndexAfter(timePointing);
            if (index != -1) {
                curEnt = mapEnts[it->first].getDataFromIndex(index);
                // Can the agent point himself?
                //if (it->first != agentMonitored)
                double curConf = MathFunctions::isInAngle(agent->skeleton_[pointingJoint],
                        curEnt, towardAngle, angleThreshold, angleResult);
                if (curConf > 0.0)
                    towardConfidence[it->first] = curConf;
            }
        }

    } else
        std::cout << "WARNING, no data to compute agent " << pointingAgent << " pointing" << std::endl;

    return towardConfidence;
}

std::map<std::string, double> computePointingToward(std::map<std::string, TRBuffer < Entity* > > mapEnts,
        std::string pointingAgent, std::string pointingJoint, double towardAngle, double angleThreshold) {
    std::map<std::string, double> towardConfidence;

    // This parameter won't be used here...
    double angleResult = 0.0;

    Agent* agent = (Agent*) mapEnts[pointingAgent].back();

    //For each entities in the same room
    for (std::map<std::string, TRBuffer < Entity*> >::iterator it = mapEnts.begin(); it != mapEnts.end(); ++it) {
        Entity* curEnt = mapEnts[it->first].back();
        // Can the agent point himself?
        //if (it->first != agentMonitored)
        double curConf = MathFunctions::isInAngle(agent->skeleton_[pointingJoint], curEnt,
                towardAngle, angleThreshold, angleResult);
        if (curConf > 0.0)
            towardConfidence[it->first] = curConf;
    }

    return towardConfidence;
}

/*void initTRBuffer(unsigned int agentMonitored, TRBuffer<Entity*>& TRBEntity, unsigned int historyLength) {
    //We need to initiate the ringbuffer... or not

    TRBuffer<Entity*> mybuffer(historyLength);
    TRBEntity = mybuffer;
}*/

///////////////////////////
//   Service functions   //
///////////////////////////

bool pointingTowardTimeRequest(toaster_msgs::PointingTime::Request &req,
        toaster_msgs::PointingTime::Response & res) {

    if (req.pointingAgentId == "") {
        ROS_INFO("[agent_monitor][Request][WARNING] request to get pointing agent with no id specified, sending back response: false");
        res.answer = false;
        return false;
    }

    // If we found the agent and his id
    if (req.pointingJoint != "") {
        res.answer = true;

        Agent* agent;
        int index = mapTRBEntity_[req.pointingAgentId].getIndexAfter(req.timePointing);
        if (index != -1) {
            agent = (Agent*) mapTRBEntity_[req.pointingAgentId].getDataFromIndex(index);
            if (isPointing(agent, req.pointingJoint, req.pointingJointDistThreshold)) {
                double towardAngle = 0.0;
                std::map < std::string, double> towardEnts;
                towardAngle = computePointingAngle(agent, req.pointingJoint);
                towardEnts = computePointingToward(mapTRBEntity_, req.pointingAgentId, req.pointingJoint, req.timePointing, towardAngle, req.angleThreshold);

                // Export result
                for (std::map < std::string, double>::iterator it = towardEnts.begin(); it != towardEnts.end(); ++it) {
                    res.pointedId.push_back(it->first);
                    res.confidence.push_back(it->second);
                }
            }
        }
        return true;
    }
}

bool pointingTowardRequest(toaster_msgs::Pointing::Request &req,
        toaster_msgs::Pointing::Response & res) {

    if (req.pointingAgentId == "") {
        ROS_INFO("[agent_monitor][Request][WARNING] request to get pointing agent with no id and no name specified, sending back response: false");
        res.answer = false;
        return false;
    }

    // If we found the agent and his id
    if (req.pointingJoint != "") {
        res.answer = true;

        // TODO: check if agent is tracked
        Agent* agent = (Agent*) mapTRBEntity_[req.pointingAgentId].back();
        if (isPointing(agent, req.pointingJoint, req.pointingJointDistThreshold)) {
            double towardAngle = 0.0;
            std::map < std::string, double> towardEnts;
            towardAngle = computePointingAngle(agent, req.pointingJoint);
            towardEnts = computePointingToward(mapTRBEntity_, req.pointingAgentId, req.pointingJoint, towardAngle, req.angleThreshold);

            // Export result
            for (std::map < std::string, double>::iterator it = towardEnts.begin(); it != towardEnts.end(); ++it) {
                res.pointedId.push_back(it->first);
                res.confidence.push_back(it->second);
            }
        }
        return true;
    }
}

/****************************************************
 * @brief : Update reactive parameters
 ****************************************************/
void dynParamCallback(agent_monitor::agent_monitorConfig &config, uint32_t level) {
    lookTwdDeltaDist_ = config.lookTwdDeltaDist;
    lookTwdAngularAperture_ = config.lookTwdAngularAperture;

    motion2DBodyTime_ = (unsigned long) (config.motion2DBodyTime * oneSecond_);
    motion2DBodySpeedThreshold_ = config.motion2DBodySpeedThreshold; // this is in m/s

    motion2DBodyDirTime_ = (unsigned long) (config.motion2DBodyDirTime * oneSecond_);
    motionTwd2DBodyAngleThresold_ = config.motionTwd2DBodyAngleThresold;

    motionTwdBodyDeltaDistTime_ = (unsigned long) (config.motionTwdBodyDeltaDistTime * oneSecond_);
    movingTwdBodyDeltaDistThreshold_ = config.movingTwdBodyDeltaDistThreshold * movingTwdBodyDeltaDistThreshold_ / oneSecond_; // this is in m

    motion2DJointTime_ = (unsigned long) (config.motion2DJointTime * oneSecond_);
    motion2DJointSpeedThreshold_ = config.motion2DJointSpeedThreshold; // this is in m

    motion2DJointDirTime_ = (unsigned long) (config.motion2DJointDirTime * oneSecond_);
    motionTwd2DJointAngleThresold_ = config.motionTwd2DJointAngleThresold;

    motionTwdJointDeltaDistTime_ = (unsigned long) (config.motionTwdJointDeltaDistTime * oneSecond_);
    movingTwdJointDeltaDistThreshold_ = config.movingTwdJointDeltaDistThreshold * motionTwdJointDeltaDistTime_ / oneSecond_; // this is in m

    distReach_ = config.distReach;
    distClose_ = config.distClose;
    distMedium_ = config.distMedium;
    distFar_ = config.distFar;
}

/////////////////////
/////// Main ////////
/////////////////////

int main(int argc, char** argv) {
    // Set this in a ros param
    const bool HUMAN_FULL_CONFIG = true; //If false we will use only position and orientation
    const bool ROBOT_FULL_CONFIG = true;

    // Make this a vector? Not really relevant if several monitored agents...
    //unsigned int roomOfInterest = 0;

    // Set this in a ros service
    // TODO: Make them vectors?
    //unsigned int agentMonitored = 101;
    //std::vector < std::string > jointsMonitoredName(1, "rWristX");
    //std::vector < unsigned int > jointsMonitoredId(1, 0);
    //bool humanMonitored = agentMonitored - 100;

    ros::init(argc, argv, "agent_monitor");
    ros::NodeHandle node;

    // TODO: add area_manager data reading to get the room of entities.
    //Data reading
    ToasterHumanReader humanRd(node, HUMAN_FULL_CONFIG);
    std::map<std::string, Human*> humansMap;
    ToasterRobotReader robotRd(node, ROBOT_FULL_CONFIG);
    std::map<std::string, Robot*> robotsMap;
    ToasterObjectReader objectRd(node);
    std::map<std::string, Object*> objectsMap;

    ParamServer_t monitoring_dyn_param_srv;
    monitoring_dyn_param_srv.setCallback(boost::bind(&dynParamCallback, _1, _2));

    //Services
    ros::ServiceServer servicePointingTime = node.advertiseService("agent_monitor/pointing_time", pointingTowardTimeRequest);
    ROS_INFO("[Request] Ready to receive timed request for pointing.");

    ros::ServiceServer servicePointing = node.advertiseService("agent_monitor/pointing", pointingTowardRequest);
    ROS_INFO("[Request] Ready to receive request for pointing.");


    agentsManager_.init(&node);

    ros::Publisher fact_pub = node.advertise<toaster_msgs::FactList>("agent_monitor/factList", 1000);

    // Set this in a ros service?
    ros::Rate loop_rate(30);

    /************************/
    /* Start of the Ros loop*/
    /************************/

    while (node.ok()) {
        toaster_msgs::FactList factList_msg;
        toaster_msgs::FactList agentFactList_msg;
        toaster_msgs::Fact fact_msg;
        // We received agentMonitored

        std::vector<std::string> agentsMonitored = agentsManager_.getMonitoredAgents();
        std::map<std::string, std::vector<std::string> > mapAgentToJointsMonitored = agentsManager_.getMonitoredJoints();

        //////////////////////////////////////
        //           Updating data          //
        //////////////////////////////////////
        humansMap = humanRd.lastConfig_;
        humanRd.clear();
        robotsMap = robotRd.lastConfig_;
        robotRd.clear();
        objectsMap = objectRd.lastConfig_;
        objectRd.clear();

        // If we monitor all humans, we add them to the agentsMonitored vector
        if (agentsManager_.monitorAllHumans())
            for (std::map<std::string, Human*>::iterator it = humansMap.begin(); it != humansMap.end(); ++it) {
                // If the pointer was updated and is not the swapped one
                if ((it->second->getId() != "") && (std::find(agentsMonitored.begin(), agentsMonitored.end(), it->second->getId()) == agentsMonitored.end())) {
                    agentsMonitored.push_back(it->first);
                }
            }

        // If we monitor all robots, we add them to the agentsMonitored vector
        if (agentsManager_.monitorAllRobots())
            for (std::map<std::string, Robot*>::iterator it = robotsMap.begin(); it != robotsMap.end(); ++it)
            { // If the pointer was updated and is not the swapped one
                if ((it->second->getId() != "") && (std::find(agentsMonitored.begin(), agentsMonitored.end(), it->second->getId()) == agentsMonitored.end())) {
                    agentsMonitored.push_back(it->first);
                }
            }

      /////////////////////////////////////
      // Update TRBuffer for each entity //
      /////////////////////////////////////
      // for each entity
      //Put the following in a function?

      // For humans
      for (std::map<std::string, Human*>::iterator it = humansMap.begin(); it != humansMap.end(); ++it)
      {
        // if not monitored agent
        if (std::find(agentsMonitored.begin(), agentsMonitored.end(), it->first) == agentsMonitored.end())
        {
          std::map<std::string, TRBuffer < Entity* > >::iterator itTRBHuman = mapTRBEntity_.find(it->first);
          Human * hum = humansMap[it->first];

          if (itTRBHuman == mapTRBEntity_.end()) // If 1st data
          {
          	TRBuffer<Entity*> buff;
            buff.push_back(hum->getTime(), hum);
            mapTRBEntity_[it->first] = buff;
          }
          else if (mapTRBEntity_[it->first].back()->getTime() < it->second->getTime())
            mapTRBEntity_[it->first].push_back(hum->getTime(), hum);
        }
      }

      // For robots
      for (std::map<std::string, Robot*>::iterator it = robotsMap.begin(); it != robotsMap.end(); ++it)
      {
        // if not monitored agent
        if (std::find(agentsMonitored.begin(), agentsMonitored.end(), it->first) == agentsMonitored.end())
        {
          std::map<std::string, TRBuffer < Entity* > >::iterator itTRBRobot = mapTRBEntity_.find(it->first);
          Robot* rob = robotsMap[it->first];

          if (itTRBRobot == mapTRBEntity_.end())
          {
            TRBuffer<Entity*> buff;
            buff.push_back(rob->getTime(), rob);
            mapTRBEntity_[it->first] = buff;
          }
          else if (mapTRBEntity_[it->first].back()->getTime() < it->second->getTime())
            mapTRBEntity_[it->first].push_back(rob->getTime(), rob);
        }
      }

      //  For Objects
      for (std::map<std::string, Object*>::iterator it = objectsMap.begin(); it != objectsMap.end(); ++it)
      {
        std::map<std::string, TRBuffer < Entity* > >::iterator itTRBObject = mapTRBEntity_.find(it->first);
        Object* obj = objectsMap[it->first];

        if (itTRBObject == mapTRBEntity_.end())
        {
          TRBuffer<Entity*> buff;
          buff.push_back(obj->getTime(), obj);
          mapTRBEntity_[it->first] = buff;
        }
        else if (mapTRBEntity_[it->first].back()->getTime() < it->second->getTime())
          mapTRBEntity_[it->first].push_back(obj->getTime(), obj);
      }

        // All the following computation are done for each monitored agents!
        for (std::vector<std::string>::iterator itAgnt = agentsMonitored.begin(); itAgnt != agentsMonitored.end(); ++itAgnt)
        {
            Agent* agentMonitored;

            bool isHuman = true;

            // Let's find back the monitored agent:
            if (robotsMap.find((*itAgnt)) != robotsMap.end()) {
              agentMonitored = robotsMap[(*itAgnt)];
              isHuman = false;
            }
            else if (humansMap.find((*itAgnt)) != humansMap.end())
                agentMonitored = humansMap[(*itAgnt)];
            else
                continue; // objects

            // We verify if the buffer is already there...
            std::map<std::string, TRBuffer < Entity* > >::iterator itTRB = mapTRBEntity_.find((*itAgnt));
            if (itTRB == mapTRBEntity_.end())
            {
              //1st time, we initialize variables
              TRBuffer<Entity*> buffAgnt, buffJnt;
              buffAgnt.push_back(agentMonitored->getTime(), agentMonitored);
              mapTRBEntity_[agentMonitored->getId()] = buffAgnt;

              // We will use directly the joint from the agent buffer
              /*// adding monitored joints to the entity.
              for (std::vector<std::string>::iterator itJnt = mapAgentToJointsMonitored[ agntCur->getId() ].begin(); itJnt != mapAgentToJointsMonitored[ agntCur->getId() ].end(); ++itJnt) {
                  jntCur = new Joint(agntCur->skeleton_[(*itJnt)]->getId(), agntCur->getId());
                  memcpy(jntCur, humanRd.lastConfig_[agentMonitored]->skeleton_[jointsMonitoredName[i]], sizeof (Joint));

                  buffJnt.push_back(jntCur->getTime(), jntCur);

                  mapTRBEntity_[jntCur->getId()] = buffJnt;
                  // printf("adding joint named: reader %d %s, tmp %s, in buffer: %s\n", agntCur->skeleton_[jointMonitoredName]->getId(), agntCur->skeleton_[jointMonitoredName]->getName().c_str(), jntCur->getName().c_str(), mapTRBEntity_[jntCur->getId()].back()->getName().c_str());
                  if (jointsMonitoredId.size() < i + 1)
                      jointsMonitoredId.push_back(jntCur->getId());
              }*/

              // This module is made for temporal reasoning.
              // We need more data to make computation, so we will end the loop here.
              continue;
            }
            else // Agent is present in mapTRBEntity_
            {
              // If this is a new data we add it to the buffer
              bool newData = false;

              if(agentMonitored->getId() != "")
              {
                if(isHuman)
                  newData = (mapTRBEntity_[(*itAgnt)].back()->getTime() < agentMonitored->getTime());
                else
                  newData = (mapTRBEntity_[(*itAgnt)].back()->getTime() < agentMonitored->getTime());
              }

              if (newData)
              {
                mapTRBEntity_[agentMonitored->getId()].push_back(agentMonitored->getTime(), agentMonitored);

                /*
                // adding monitored joint to the entities.
                for (unsigned int i = 0; i < jointsMonitoredName.size(); i++) {
                    jntCur = new Joint(humCur->skeleton_[jointsMonitoredName[i]]->getId(), agentMonitored);
                    memcpy(jntCur, humanRd.lastConfig_[agentMonitored]->skeleton_[jointsMonitoredName[i]], sizeof (Joint));

                    mapTRBEntity_[jntCur->getId()].push_back(jntCur->getTime(), jntCur);
                }*/
              }
              else  // If we don't have new data
              {     // Do we need this? Or should we update other entities?
                factList_msg.factList.insert(factList_msg.factList.end(), previousAgentsFactList_[*itAgnt].begin(), previousAgentsFactList_[*itAgnt].end());
                continue;
              }
            }

            //////////////////////////////////////////////
            // Compute facts concerning monitored agent //
            //////////////////////////////////////////////

            ROS_DEBUG("[agent_monitor] computing facts for agent %s\n", (*itAgnt).c_str());

            //looking facts
            std::map<std::string, double> mapIdValue;
            mapIdValue = LookingFact::compute(mapTRBEntity_, (*itAgnt), lookTwdDeltaDist_, lookTwdAngularAperture_);
            LookingFact::createTowardFact(mapIdValue, factList_msg,
                                          lookTwdAngularAperture_,
                                          (*itAgnt),
                                          mapTRBEntity_[(*itAgnt)].back());

		mapIdValue.clear();
            mapIdValue = LookingFact::compute(mapTRBEntity_, (*itAgnt), lookTwdDeltaDist_*0.5, lookTwdAngularAperture_*0.25);
            LookingFact::createAtFact(mapIdValue, factList_msg,
                                          lookTwdAngularAperture_*0.25,
                                          (*itAgnt),
                                          mapTRBEntity_[(*itAgnt)].back());
            // If the agent is moving
            double speed = Motion2D::compute(mapTRBEntity_[(*itAgnt)], motion2DBodyTime_);

            if (speed > (motion2DBodySpeedThreshold_)) //if in movement
            {
                double confidence = speed * 3.6 / 5.0; // Confidence is 1 if speed is 5 km/h or above
                confidence = (confidence > 1.0) ? 1.0 : confidence;
                //Fact moving
                fact_msg = FactCreator::setFactBase((*itAgnt), mapTRBEntity_);
                fact_msg = FactCreator::setMotionFact(fact_msg, speed, confidence);
                agentFactList_msg.factList.push_back(fact_msg);

                // We compute the direction toward fact:
                double angleDirection = Motion2D::computeDirection(mapTRBEntity_[(*itAgnt)], motion2DBodyDirTime_);
                mapIdValue = Motion2D::computeToward(mapTRBEntity_, (*itAgnt), angleDirection, motionTwd2DBodyAngleThresold_);

                for (std::map<std::string, double>::iterator it = mapIdValue.begin(); it != mapIdValue.end(); ++it)
                {
                  if (it->second > movingTwdBodyDeltaDistThreshold_)
                  {
                    //Fact moving toward
                    fact_msg = FactCreator::setFactBase((*itAgnt), mapTRBEntity_);
                    fact_msg = FactCreator::setDirectionFact(fact_msg, it->first, it->second);
                    agentFactList_msg.factList.push_back(fact_msg);
                  }
                }

                // We compute /_\distance toward entities
                mapIdValue = Distances::computeDeltaDist(mapTRBEntity_, (*itAgnt), motionTwdBodyDeltaDistTime_);
                for (std::map<std::string, double>::iterator it = mapIdValue.begin(); it != mapIdValue.end(); ++it)
                {
                  if (it->second > movingTwdBodyDeltaDistThreshold_)
                  {
                    //Fact moving toward
                    fact_msg = FactCreator::setFactBase((*itAgnt), mapTRBEntity_);
                    fact_msg = FactCreator::setDistanceFact(fact_msg, it->first, it->second);
                    agentFactList_msg.factList.push_back(fact_msg);
                  }
                }
            }
            else // If agent is not moving, we compute his joint motion
            {
                double dist3D;
                std::string dist3DString;

                // What is the distance between joints and objects?
                for (std::vector<std::string>::iterator itJnt = mapAgentToJointsMonitored[(*itAgnt)].begin(); itJnt != mapAgentToJointsMonitored[(*itAgnt)].end(); ++itJnt) {

                    Joint* curMonitoredJnt = ((Agent*) mapTRBEntity_[(*itAgnt)].back())->skeleton_[(*itJnt)];
                    for (std::map<std::string, TRBuffer < Entity*> >::iterator itEnt = mapTRBEntity_.begin(); itEnt != mapTRBEntity_.end(); ++itEnt) {
                        // if in same room as monitored agent and not monitored joint
                        //if ((roomOfInterest == it->second.back()->getRoomId()) && (it->first != jointsMonitoredId[i])) {
                        dist3D = bg::distance(curMonitoredJnt->getPosition(), itEnt->second.back()->getPosition());

                        if (dist3D < distReach_)
                            dist3DString = "reach";
                        else if (dist3D < distClose_)
                            dist3DString = "close";
                        else if (dist3D < distMedium_)
                            dist3DString = "medium";
                        else if (dist3D < distFar_)
                            dist3DString = "far";
                        else
                            dist3DString = "out";

                        //Fact distance
                        fact_msg = FactCreator::setFactBase(curMonitoredJnt);
                        fact_msg.property = "Distance";
                        fact_msg.propertyType = "position";
                        fact_msg.subProperty = "3D";
                        fact_msg.targetId = itEnt->first;

                        fact_msg.valueType = 0;
                        fact_msg.stringValue = dist3DString;
                        fact_msg.doubleValue = dist3D;
                        fact_msg.confidence = 0.90;

                        agentFactList_msg.factList.push_back(fact_msg);
                        //}
                    }
                    // Is the joint moving?
                    speed = Motion2D::compute(mapTRBEntity_[(*itAgnt)], motion2DJointTime_, (*itJnt));

                    //We consider motion when it moves more than 3 cm during 1/4 second, so when higher than 0.12 m/s
                    if (speed > (motion2DJointSpeedThreshold_))
                    {
                        double confidence = speed * 3.6 / 20.0; // Confidence is 1 if speed is 20 km/h or above
                        confidence = (confidence > 1.0) ? 1.0 : confidence;

                        //Fact moving
                        fact_msg = FactCreator::setFactBase(curMonitoredJnt);
                        fact_msg = FactCreator::setMotionFact(fact_msg, speed, confidence, "joint");
                        agentFactList_msg.factList.push_back(fact_msg);

                        // We compute the direction toward fact:
                        double angleDirection = Motion2D::computeDirection(mapTRBEntity_[(*itAgnt)], motion2DJointDirTime_, (*itJnt));
                        mapIdValue = Motion2D::computeToward(mapTRBEntity_, (*itAgnt), angleDirection, motionTwd2DJointAngleThresold_, (*itJnt));
                        for (std::map<std::string, double>::iterator it = mapIdValue.begin(); it != mapIdValue.end(); ++it)
                        {
                          fact_msg = FactCreator::setFactBase(curMonitoredJnt);
                          fact_msg = FactCreator::setDirectionFact(fact_msg, it->first, it->second);
                          agentFactList_msg.factList.push_back(fact_msg);
                        }

                        // Then we compute /_\distance
                        mapIdValue = Distances::computeJointDeltaDist(mapTRBEntity_, (*itAgnt), (*itJnt), motionTwdJointDeltaDistTime_);
                        for (std::map<std::string, double>::iterator it = mapIdValue.begin(); it != mapIdValue.end(); ++it)
                        {
                          if (it->second > movingTwdJointDeltaDistThreshold_)
                          {
                            fact_msg = FactCreator::setFactBase(curMonitoredJnt);
                            fact_msg = FactCreator::setDistanceFact(fact_msg, it->first, it->second);
                            agentFactList_msg.factList.push_back(fact_msg);
                          }
                        }
                    } // Joint moving
                } // All monitored joints
            } // Joints or full agent?

            previousAgentsFactList_[*itAgnt] = agentFactList_msg.factList;
            factList_msg.factList.insert(factList_msg.factList.end(), agentFactList_msg.factList.begin(), agentFactList_msg.factList.end());
            agentFactList_msg.factList.clear();
        } // each monitored agents

        fact_pub.publish(factList_msg);

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
