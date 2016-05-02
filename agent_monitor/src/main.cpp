// This main will compute the facts for the requested agent.

#include "toaster_msgs/ToasterHumanReader.h"
#include "toaster_msgs/ToasterRobotReader.h"
#include "toaster_msgs/ToasterObjectReader.h"
#include "toaster_msgs/AddAgent.h"
#include "toaster_msgs/AddJointToAgent.h"
#include "toaster_msgs/RemoveAgent.h"
#include "toaster_msgs/RemoveJointToAgent.h"
#include "toaster_msgs/RemoveAllJointsToAgent.h"
#include "toaster_msgs/Empty.h"
#include "toaster_msgs/MonitorAll.h"
#include "toaster_msgs/FactList.h"
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/PointingTime.h"
#include "toaster_msgs/Pointing.h"

#include "toaster-lib/TRBuffer.h"
#include "toaster-lib/MathFunctions.h"
#define PI 3.141592


std::vector<std::string> agentsMonitored_;
std::map<std::string, std::vector<std::string> > mapAgentToJointsMonitored_;
bool monitorAllHumans_ = false;
bool monitorAllRobots_ = false;


// Compute motion:
unsigned long oneSecond = pow(10, 9);

// Map of Timed Ring Buffer Entities
static std::map<std::string, TRBuffer < Entity* > > mapTRBEntity_;
std::map<std::string, TRBuffer < Entity* > >::iterator itTRB_;

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
    double curConf;

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
                curConf = MathFunctions::isInAngle(agent->skeleton_[pointingJoint],
                        curEnt, towardAngle, angleThreshold, angleResult);
                if (curConf > 0.0)
                    towardConfidence[it->first] = curConf;
            }
        }

    } else {
        printf("WARNING, no data to compute agent %s pointing", pointingAgent.c_str());
    }
    return towardConfidence;
}

std::map<std::string, double> computePointingToward(std::map<std::string, TRBuffer < Entity* > > mapEnts,
        std::string pointingAgent, std::string pointingJoint, double towardAngle, double angleThreshold) {
    std::map<std::string, double> towardConfidence;
    double curConf;

    // This parameter won't be used here...
    double angleResult = 0.0;


    Agent* agent = (Agent*) mapEnts[pointingAgent].back();

    //For each entities in the same room
    for (std::map<std::string, TRBuffer < Entity*> >::iterator it = mapEnts.begin(); it != mapEnts.end(); ++it) {
        Entity* curEnt = mapEnts[it->first].back();
        // Can the agent point himself?
        //if (it->first != agentMonitored)
        curConf = MathFunctions::isInAngle(agent->skeleton_[pointingJoint], curEnt,
                towardAngle, angleThreshold, angleResult);
        if (curConf > 0.0)
            towardConfidence[it->first] = curConf;
    }

    return towardConfidence;
}

bool computeIsMoving2D(TRBuffer< Entity* > confBuffer, unsigned long timelapse, double distanceThreshold) {
    int index;
    double dist = 0.0;
    long actualTimelapse = 0;
    long timeNew = confBuffer.back()->getTime();
    long timeOld = timeNew - timelapse;
    Entity* entNew = confBuffer.back();

    index = confBuffer.getIndexAfter(timeOld);
    // In case we don't have the index, we will just put isMoving to false
    if (index == -1)
        return false;
    actualTimelapse = timeNew - confBuffer.getTimeFromIndex(index); // Actual timelapse
    Entity* entOld = confBuffer.getDataFromIndex(index);

    dist = bg::distance(MathFunctions::convert3dTo2d(entNew->getPosition()),
            MathFunctions::convert3dTo2d(entOld->getPosition()));

    /*std::cout << "Distance is " << dist << std::endl;
    std::cout << "ds*actualTimeLapse / timelapse " << distanceThreshold * actualTimelapse / timelapse << std::endl;
    std::cout << "ds " << distanceThreshold << std::endl;
    std::cout << "timelapse " << timelapse << std::endl;
    std::cout << "actual timelapse " << actualTimelapse << std::endl;*/
    if (dist < distanceThreshold * actualTimelapse / timelapse) {
        return false;
    } else {
        return true;
    }
}

double computeMotion2D(TRBuffer< Entity* > confBuffer, unsigned long timelapse) {
    int index;
    double dist = 0.0;
    long actualTimelapse = 0;
    long timeNew = confBuffer.back()->getTime();
    long timeOld = timeNew - timelapse;
    Entity* entNew = confBuffer.back();

    index = confBuffer.getIndexAfter(timeOld);
    // In case we don't have the index, we will just put isMoving to false
    if (index == -1)
        return false;
    actualTimelapse = timeNew - confBuffer.getTimeFromIndex(index); // Actual timelapse
    Entity* entOld = confBuffer.getDataFromIndex(index);

    dist = bg::distance(MathFunctions::convert3dTo2d(entNew->getPosition()),
            MathFunctions::convert3dTo2d(entOld->getPosition()));

    /*std::cout << "Distance is " << dist << std::endl;
    std::cout << "ds*actualTimeLapse / timelapse " << distanceThreshold * actualTimelapse / timelapse << std::endl;
    std::cout << "ds " << distanceThreshold << std::endl;
    std::cout << "timelapse " << timelapse << std::endl;
    std::cout << "actual timelapse " << actualTimelapse << std::endl;*/

    return dist * oneSecond / actualTimelapse;
}

bool computeJointIsMoving2D(TRBuffer< Entity* > confBuffer, std::string jointName,
        unsigned long timelapse, double distanceThreshold) {
    int index;
    double dist = 0.0;
    long actualTimelapse = 0;
    long timeNew = confBuffer.getTimeFromIndex(confBuffer.size() - 1);
    long timeOld = timeNew - timelapse;
    Entity* entNew = ((Agent*) confBuffer.back())->skeleton_[jointName];

    index = confBuffer.getIndexAfter(timeOld);
    // In case we don't have the index, we will just put isMoving to false
    if (index == -1)
        return false;
    actualTimelapse = timeNew - confBuffer.getTimeFromIndex(index); // Actual timelapse
    Entity* entOld = ((Agent*) confBuffer.getDataFromIndex(index))->skeleton_[jointName];

    dist = bg::distance(MathFunctions::convert3dTo2d(entNew->getPosition()),
            MathFunctions::convert3dTo2d(entOld->getPosition()));

    /*std::cout << "Distance is " << dist << std::endl;
    std::cout << "ds*actualTimeLapse / timelapse " << distanceThreshold * actualTimelapse / timelapse << std::endl;
    std::cout << "ds " << distanceThreshold << std::endl;
    std::cout << "timelapse " << timelapse << std::endl;
    std::cout << "actual timelapse " << actualTimelapse << std::endl;*/
    if (dist < distanceThreshold * actualTimelapse / timelapse) {
        return false;
    } else {
        return true;
    }
}

double computeJointMotion2D(TRBuffer< Entity* > confBuffer, std::string jointName,
        unsigned long timelapse) {
    int index;
    double dist = 0.0;
    long actualTimelapse = 0;
    long timeNew = confBuffer.getTimeFromIndex(confBuffer.size() - 1);
    long timeOld = timeNew - timelapse;
    Entity* entNew = ((Agent*) confBuffer.back())->skeleton_[jointName];

    index = confBuffer.getIndexAfter(timeOld);
    // In case we don't have the index, we will just put isMoving to false
    if (index == -1)
        return false;
    actualTimelapse = timeNew - confBuffer.getTimeFromIndex(index); // Actual timelapse
    Entity* entOld = ((Agent*) confBuffer.getDataFromIndex(index))->skeleton_[jointName];

    dist = bg::distance(MathFunctions::convert3dTo2d(entNew->getPosition()),
            MathFunctions::convert3dTo2d(entOld->getPosition()));

    /*std::cout << "Distance is " << dist << std::endl;
    std::cout << "ds*actualTimeLapse / timelapse " << distanceThreshold * actualTimelapse / timelapse << std::endl;
    std::cout << "ds " << distanceThreshold << std::endl;
    std::cout << "timelapse " << timelapse << std::endl;
    std::cout << "actual timelapse " << actualTimelapse << std::endl;*/
    return dist * oneSecond / actualTimelapse;
}

double computeMotion2DDirection(TRBuffer< Entity* > confBuffer, unsigned long timelapse) {
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
            / bg::distance(MathFunctions::convert3dTo2d(entOld->getPosition()),
            MathFunctions::convert3dTo2d(entNew->getPosition())));

    // Trigonometric adjustment
    if (entNew->getPosition().get<0>() < entOld->getPosition().get<0>())
        towardAngle = 3.1416 - towardAngle;

    if (entNew->getPosition().get<1>() < entOld->getPosition().get<1>())
        towardAngle = -towardAngle;

    return towardAngle;
}

double computeJointMotion2DDirection(TRBuffer< Entity* > confBuffer, std::string jointName, unsigned long timelapse) {
    double towardAngle;
    int index;
    //long actualTimelapse = 0;
    long timeNew = confBuffer.getTimeFromIndex(confBuffer.size() - 1);
    long timeOld = timeNew - timelapse;
    Entity* entNew = ((Agent*) confBuffer.getDataFromIndex(confBuffer.size() - 1))->skeleton_[jointName];

    index = confBuffer.getIndexAfter(timeOld);
    //actualTimelapse = timeNew - confBuffer.getTimeFromIndex(index);   // Actual timelapse
    Entity* entOld = ((Agent*) confBuffer.getDataFromIndex(index))->skeleton_[jointName];
    towardAngle = acos(fabs(entOld->getPosition().get<0>() - entNew->getPosition().get<0>())
            / bg::distance(MathFunctions::convert3dTo2d(entOld->getPosition()),
            MathFunctions::convert3dTo2d(entNew->getPosition())));

    // Trigonometric adjustment
    if (entNew->getPosition().get<0>() < entOld->getPosition().get<0>())
        towardAngle = 3.1416 - towardAngle;

    if (entNew->getPosition().get<1>() < entOld->getPosition().get<1>())
        towardAngle = -towardAngle;

    return towardAngle;
}

std::map<std::string, double> computeMotion2DToward(std::map<std::string, TRBuffer < Entity* > > mapEnts,
        std::string agentMonitored, double towardAngle, double angleThreshold) {

    std::map<std::string, double> towardConfidence;
    double curConf;

    // This parameter won't be used here...
    double angleResult = 0.0;

    //For each entities in the same room
    for (std::map<std::string, TRBuffer < Entity*> >::iterator it = mapEnts.begin(); it != mapEnts.end(); ++it) {
        if (it->first != agentMonitored) {
            curConf = MathFunctions::isInAngle(mapEnts[agentMonitored].back(),
                    it->second.back(), towardAngle, angleThreshold, angleResult);
            if (curConf > 0.0)
                towardConfidence[it->first] = curConf;
        }
    }
    return towardConfidence;
}

std::map<std::string, double> computeJointMotion2DToward(std::map<std::string, TRBuffer < Entity* > > mapEnts,
        std::string agentMonitored, std::string jointName, double towardAngle, double angleThreshold) {

    std::map<std::string, double> towardConfidence;
    double curConf;

    // This parameter won't be used here...
    double angleResult = 0.0;

    //For each entities in the same room
    for (std::map<std::string, TRBuffer < Entity*> >::iterator it = mapEnts.begin(); it != mapEnts.end(); ++it) {
        if (it->first != agentMonitored) {
            curConf = MathFunctions::isInAngle(((Agent*) mapEnts[agentMonitored].back())->skeleton_[jointName],
                    it->second.back(), towardAngle, angleThreshold, angleResult);
            if (curConf > 0.0)
                towardConfidence[it->first] = curConf;
        }
    }
    return towardConfidence;
}

std::map<std::string, double> computeDeltaDist(std::map<std::string, TRBuffer < Entity* > > mapEnts,
        std::string agentMonitored, unsigned long timelapse) {
    std::map<std::string, double> deltaDistMap;
    double curDist = 0.0;
    double prevDist = 0.0;
    double deltaDist = 0.0;
    unsigned long timeCur = 0;
    unsigned long timePrev = 0;
    Entity * entCur(0);
    Entity * entMonitoredCur(0);
    Entity * entMonitoredPrev(0);

    //For each entities in the same room
    for (std::map<std::string, TRBuffer < Entity*> >::iterator it = mapEnts.begin(); it != mapEnts.end(); ++it) {
        if (it->first != agentMonitored) {
            // We compute the current distance
            entCur = it->second.back();
            entMonitoredCur = mapEnts[agentMonitored].back();

            //Put this in a function
            curDist = bg::distance(MathFunctions::convert3dTo2d(entCur->getPosition()),
                    MathFunctions::convert3dTo2d(entMonitoredCur->getPosition()));

            // We compute the distance at now - timelapse
            timeCur = entMonitoredCur->getTime();
            timePrev = timeCur - timelapse;

            entMonitoredPrev = mapEnts[agentMonitored].getDataFromIndex(mapEnts[agentMonitored].getIndexAfter(timePrev));

            prevDist = bg::distance(MathFunctions::convert3dTo2d(entCur->getPosition()),
                    MathFunctions::convert3dTo2d(entMonitoredPrev->getPosition()));


            //We compute Deltadist
            deltaDist = prevDist - curDist;

            // We fill towardConfidence
            deltaDistMap[it->first] = deltaDist;
        }
    }
    return deltaDistMap;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
* @brief This function compute the map required by the fact "IsLookingToward" 
*        by testing if an entity is lying in the 3D cone of agent visual attention
* @param Entity map containing all entity involved in the joint action & their 
*        respective ids
* @param Monitored agent id
* @param Distance beetween center of basement circle and agent head position
* @param Angular aperture of the cone in radians
* @return Map required by the fact "IsLookingToward" containing all entities 
*         lying in the cone and all normalized angles beetween entities and cone axis         
*/
//for convenience
typedef std::vector<double> Vec_t;
typedef std::vector<Vec_t> Mat_t;
typedef std::map<std::string, double> Map_t;
typedef std::pair<std::string, double> Pair_t;

Vec_t operator*(const Mat_t &a, const Vec_t &x)
{
  int i,j;
  int m = a.size();
  int n = x.size();

  Vec_t prod(m);

  for(i = 0; i < m; i++){
    prod[i] = 0.;
    for(j = 0; j < n; j++)
      prod[i] += a[i][j]*x[j];
  }
  return prod;
}

Vec_t operator-(const Vec_t &a , const Vec_t &b)
{
    int i;
    int m = a.size();
    int n = b.size();
    if (m == n)
    {
        Vec_t diff(m);
        for (int i = 0; i < m; ++i)
        {
            diff[i]=a[i]-b[i];
        }
        return diff;
    } else {

    }
        return (Vec_t)0;
}

Mat_t matrixfromAngle(const int &angleType , const double &angle)
{
    Mat_t mat(3);
    Vec_t row0;
    Vec_t row1;
    Vec_t row2;

    if (angleType==0)
    {
        row0.push_back(1.0);
        row0.push_back(0.0);
        row0.push_back(0.0);
        
        row1.push_back(0.0);
        row1.push_back(cos(angle));
        row1.push_back(-sin(angle));
        
        row2.push_back(0.0);
        row2.push_back(sin(angle));
        row2.push_back(cos(angle));
        
    }else{
        if (angleType==1)
        {
            row0.push_back(cos(angle));
            row0.push_back(0.0);
            row0.push_back(sin(angle));
            
            row1.push_back(0.0);
            row1.push_back(1.0);
            row1.push_back(0.0);
            
            row2.push_back(-sin(angle));
            row2.push_back(0.0);
            row2.push_back(cos(angle));
        }else{
            row0.push_back(cos(angle));
            row0.push_back(-sin(angle));
            row0.push_back(0.0);
            
            row1.push_back(sin(angle));
            row1.push_back(cos(angle));
            row1.push_back(0.0);
            
            row2.push_back(0.0);
            row2.push_back(0.0);
            row2.push_back(1.0);
        }
    }
    mat[0]=row0;
    mat[1]=row1;
    mat[2]=row2;
    return mat;
}

double magn(const Vec_t &a)
{
    int n=a.size();
    double magn=0;
    for (int i = 0; i < n; ++i)
    {
        magn+=a[i]*a[i];
    }
    return sqrt(magn);
}

double dotProd(const Vec_t &a, const Vec_t &b)
{
    int n=a.size();
    int m=b.size();
    double dotProd=0;
    if (n==m)
    {
        for (int i = 0; i < m; ++i)
        {
            dotProd+=a[i]*b[i];
        }
    }
    return dotProd;
}

std::map<std::string, double> computeIsLookingToward(std::map<std::string, TRBuffer < Entity* > > mapEnts,
    std::string agentMonitored, double deltaDist, double angularAperture)
    {
        Map_t returnMap;
        Pair_t pair;
        Entity * currentEntity;
        Entity * monitoredAgent;
        float halfAperture=angularAperture/2.f;
        Vec_t agentHeadPosition(3);
        Vec_t agentHeadOrientation(3);
        Vec_t entityPosition(3);
        Vec_t agentToEntity(3);
        Vec_t coneAxis(3);
        Vec_t coneBase(3);
        float entitytoAxisAngle;
        Mat_t rotX(3);
        Mat_t rotY(3);
        Mat_t rotZ(3);
        bool isInInfiniteCone=false;
        double angle;

        //Get the monitored agent head entity
        monitoredAgent = ((Agent*) mapEnts[agentMonitored].back())->skeleton_["head"];
        //Get 3d position from agent head
        agentHeadPosition[0]=bg::get<0>(monitoredAgent->getPosition());
        agentHeadPosition[1]=bg::get<1>(monitoredAgent->getPosition());
        agentHeadPosition[2]=bg::get<2>(monitoredAgent->getPosition());
        //Get 3d orientation (roll pitch yaw) from agent head
        agentHeadOrientation=(Vec_t)monitoredAgent->getOrientation();        
        //Compute rotation matricies from agent head orientation
        rotX=matrixfromAngle(0,agentHeadOrientation[0]);
        rotY=matrixfromAngle(1,agentHeadOrientation[1]);
        rotZ=matrixfromAngle(2,agentHeadOrientation[2]);

        for (std::map<std::string, TRBuffer < Entity*> >::iterator it = mapEnts.begin(); it != mapEnts.end(); ++it) 
        {
            if (it->first != agentMonitored)
            {
                
                //Get the current entity
                currentEntity = it->second.back();
                //Get the postion from current entity
                entityPosition[0]=bg::get<0>(currentEntity->getPosition());
                entityPosition[1]=bg::get<1>(currentEntity->getPosition());
                entityPosition[2]=bg::get<2>(currentEntity->getPosition());
                //Compute cone base coordinates
                coneBase[0]=agentHeadPosition[0]+deltaDist;
                coneBase[1]=agentHeadPosition[1];
                coneBase[2]=agentHeadPosition[2];
                //Apply rotation matrix from agent head orientation to cone base
                //coneBase=rotX*coneBase;
                coneBase=rotY*coneBase;
                coneBase=rotZ*coneBase;
                //Compute the 3d vector from agent head to current entity
                agentToEntity=agentHeadPosition-entityPosition;                
                //Compute cone axis
                coneAxis=agentHeadPosition-coneBase;
                
                angle=(dotProd(agentToEntity,coneAxis)/magn(agentToEntity)/magn(coneAxis));
                if(angle>cos(halfAperture))
                {
                    if(dotProd(agentToEntity,coneAxis)/magn(coneAxis)<magn(coneAxis))
                    {
                        returnMap.insert(std::pair<std::string,double>(currentEntity->getId(),angle));
                    }
                }
            }
        }
        return returnMap;
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::map<std::string, double> computeJointDeltaDist(std::map<std::string, TRBuffer < Entity* > > mapEnts,
        std::string agentMonitored, std::string jointName, unsigned long timelapse) {
    std::map<std::string, double> deltaDistMap;
    double curDist = 0.0;
    double prevDist = 0.0;
    double deltaDist = 0.0;
    unsigned long timeCur = 0;
    unsigned long timePrev = 0;
    Entity * entCur(0);
    Entity * entMonitoredCur(0);
    Entity * entMonitoredPrev(0);

    //For each entities in the same room
    for (std::map<std::string, TRBuffer < Entity*> >::iterator it = mapEnts.begin(); it != mapEnts.end(); ++it) {
        if (it->first != agentMonitored) {
            // We compute the current distance
            entCur = it->second.back();
            entMonitoredCur = ((Agent*) mapEnts[agentMonitored].back())->skeleton_[jointName];

            //Put this in a function
            curDist = bg::distance(MathFunctions::convert3dTo2d(entCur->getPosition()),
                    MathFunctions::convert3dTo2d(entMonitoredCur->getPosition()));

            // We compute the distance at now - timelapse
            timeCur = entMonitoredCur->getTime();
            timePrev = timeCur - timelapse;

            entMonitoredPrev = ((Agent*) mapEnts[agentMonitored].getDataFromIndex(
                    mapEnts[agentMonitored].getIndexAfter(timePrev)))->skeleton_[jointName];

            prevDist = bg::distance(MathFunctions::convert3dTo2d(entCur->getPosition()),
                    MathFunctions::convert3dTo2d(entMonitoredPrev->getPosition()));


            //We compute Deltadist
            deltaDist = curDist - prevDist;

            // We fill towardConfidence
            deltaDistMap[it->first] = deltaDist;
        }
    }
    return deltaDistMap;
}

/*void initTRBuffer(unsigned int agentMonitored, TRBuffer<Entity*>& TRBEntity, unsigned int historyLength) {
    //We need to initiate the ringbuffer... or not

    TRBuffer<Entity*> mybuffer(historyLength);
    TRBEntity = mybuffer;
}*/

bool addMonitoredAgent(std::string id) {
    if (std::find(agentsMonitored_.begin(), agentsMonitored_.end(), id) == agentsMonitored_.end()) {
        ROS_INFO("[agent_monitor][INFO] Agent %s now monitored", id.c_str());
        agentsMonitored_.push_back(id);
    } else {
        ROS_INFO("[agent_monitor][INFO] Agent %s is already monitored", id.c_str());
    }
}

bool removeMonitoredAgent(std::string id) {
    if (std::find(agentsMonitored_.begin(), agentsMonitored_.end(), id) == agentsMonitored_.end()) {
        ROS_INFO("[agent_monitor][INFO] Agent %s no more monitored", id.c_str());
        agentsMonitored_.erase(std::remove(agentsMonitored_.begin(), agentsMonitored_.end(), id), agentsMonitored_.end());
    } else {
        ROS_INFO("[agent_monitor][INFO] Agent %s is already not monitored", id.c_str());
    }
}




///////////////////////////
//   Service functions   //
///////////////////////////

bool addAgent(toaster_msgs::AddAgent::Request &req,
        toaster_msgs::AddAgent::Response & res) {
    if (req.id != "")
        res.answer = addMonitoredAgent(req.id);
    else {
        ROS_INFO("[agent_monitor][Request][WARNING] request to add agent with "
                "no id specified, sending back response: false");
        res.answer = false;
    }
    return true;
}

bool removeAgent(toaster_msgs::RemoveAgent::Request &req,
        toaster_msgs::RemoveAgent::Response & res) {
    if (req.id != "")
        res.answer = removeMonitoredAgent(req.id);
    else {
        ROS_INFO("[agent_monitor][Request][WARNING] request to remove agent with "
                "no id specified, sending back response: false");
        res.answer = false;
    }
    return true;
}

bool removeAllAgents(toaster_msgs::Empty::Request &req,
        toaster_msgs::Empty::Response & res) {

    agentsMonitored_.clear();
    ROS_INFO("[agent_monitor][Request][WARNING] request to remove all agents");
    return true;
}

bool addJointToAgent(toaster_msgs::AddJointToAgent::Request &req,
        toaster_msgs::AddJointToAgent::Response & res) {

    if (req.agentId != "")
        if (std::find(agentsMonitored_.begin(), agentsMonitored_.end(), req.agentId) != agentsMonitored_.end())
            if (req.jointName != "")
                if (mapAgentToJointsMonitored_.find(req.agentId) != mapAgentToJointsMonitored_.end())
                    // Joint was not monitored
                    if (std::find(mapAgentToJointsMonitored_[req.agentId].begin(), mapAgentToJointsMonitored_[req.agentId].end(), req.jointName) == mapAgentToJointsMonitored_[req.agentId].end()) {
                        ROS_INFO("[agent_monitor][Request][INFO] Now monitoring joint "
                                "%s of agent with id %s , sending back response: true", req.jointName.c_str(), req.agentId.c_str());
                        res.answer = true;
                        mapAgentToJointsMonitored_[req.agentId].push_back(req.jointName);
                        // Joint is already monitored
                    } else {
                        ROS_INFO("[agent_monitor][Request][INFO] Joint %s of agent "
                                "with id %s already monitored, sending back response: false", req.jointName.c_str(), req.agentId.c_str());
                        res.answer = false;
                    } else {
                    std::vector<std::string> tmpVec(1, req.jointName);
                    mapAgentToJointsMonitored_[req.agentId] = tmpVec;
                }// No jointName specified
            else {
                ROS_INFO("[agent_monitor][Request][WARNING] request to add joint "
                        "to agent with id %s with no joint name specified, sending back response: false", req.agentId.c_str());
                res.answer = false;
            } else {
            ROS_INFO("[agent_monitor][Request][WARNING] Agent with id %s is not "
                    "monitored, can't monitor his joint. Sending back response: false", req.agentId.c_str());
            res.answer = false;
        } else {
        ROS_INFO("[agent_monitor][Request][WARNING] request to addJoint with no id specified, sending back response: false");
        res.answer = false;
    }
    return true;
}

bool removeJointToAgent(toaster_msgs::RemoveJointToAgent::Request &req,
        toaster_msgs::RemoveJointToAgent::Response & res) {

    if (req.agentId != "")
        if (std::find(agentsMonitored_.begin(), agentsMonitored_.end(), req.agentId) != agentsMonitored_.end())
            if (req.jointName != "")
                // Joint was monitored
                if (std::find(mapAgentToJointsMonitored_[req.agentId].begin(),
                        mapAgentToJointsMonitored_[req.agentId].end(), req.jointName) != mapAgentToJointsMonitored_[req.agentId].end()) {
                    ROS_INFO("[agent_monitor][Request][INFO] Joint %s of agent with id %s"
                            " not monitored anymore, sending back response: true", req.jointName.c_str(), req.agentId.c_str());
                    res.answer = true;
                    mapAgentToJointsMonitored_[req.agentId].erase(std::remove(mapAgentToJointsMonitored_[req.agentId].begin(),
                            mapAgentToJointsMonitored_[req.agentId].end(), req.jointName), mapAgentToJointsMonitored_[req.agentId].end());
                    // Joint is not monitored
                } else {
                    ROS_INFO("[agent_monitor][Request][INFO] Joint %s of agent with "
                            "id %s already not monitored, sending back response: false", req.jointName.c_str(), req.agentId.c_str());
                    res.answer = false;
                }// No jointName specified
            else {
                ROS_INFO("[agent_monitor][Request][WARNING] request to remove joint "
                        "to agent with id %s with no name specified, sending back response: false", req.agentId.c_str());
                res.answer = false;
            } else {
            ROS_INFO("[agent_monitor][Request][WARNING] Agent with id %s is not "
                    "monitored, can't stop monitoring his joint. Sending back response: false", req.agentId.c_str());
            res.answer = false;
        } else {
        ROS_INFO("[agent_monitor][Request][WARNING] request to remove agent's joint "
                "with no id  specified, sending back response: false");
        res.answer = false;
    }
    return true;
}

bool removeAllJointsToAgent(toaster_msgs::RemoveAllJointsToAgent::Request &req,
        toaster_msgs::RemoveAllJointsToAgent::Response & res) {

    if (req.agentId != "")
        if (std::find(agentsMonitored_.begin(), agentsMonitored_.end(), req.agentId) != agentsMonitored_.end()) {
            ROS_INFO("[agent_monitor][Request][INFO] Agent with id %s joint's not "
                    "monitored anymore, sending back response: true", req.agentId.c_str());
            mapAgentToJointsMonitored_[req.agentId].clear();
            res.answer = true;
        } else {
            ROS_INFO("[agent_monitor][Request][INFO] Agent with id %s is already "
                    "not monitored, sending back response: false", req.agentId.c_str());
            res.answer = false;
        } else {
        ROS_INFO("[agent_monitor][Request][WARNING] request to remove agent joints' "
                "with no id and no name specified, sending back response: false");
        res.answer = false;
    }
    return true;
}

bool printAllMonitoredAgents(toaster_msgs::Empty::Request &req,
        toaster_msgs::Empty::Response & res) {

    std::string name;
    ROS_INFO("[agent_monitor][Request][PRINT] ****** Agents Monitored *******");
    for (std::vector<std::string>::iterator it = agentsMonitored_.begin(); it != agentsMonitored_.end(); ++it) {
        name = "";
        ROS_INFO("[agent_monitor][Request][PRINT] Agent id: %s", (*it).c_str());
        for (std::vector<std::string>::iterator itJoint = mapAgentToJointsMonitored_[*it].begin();
                itJoint != mapAgentToJointsMonitored_[*it].end(); ++itJoint) {
            ROS_INFO("[agent_monitor][Request][PRINT] Joint Monitored name: %s", (*itJoint).c_str());
        }
    }
    return true;
}

bool monitorAllAgents(toaster_msgs::MonitorAll::Request &req,
        toaster_msgs::MonitorAll::Response & res) {

    monitorAllHumans_ = req.monitorAll;
    monitorAllRobots_ = req.monitorAll;
    if (req.monitorAll)
        ROS_INFO("[agent_monitor][REQUEST] Start monitoring all agents");
    else
        ROS_INFO("[agent_monitor][REQUEST] Stop monitoring all agents");
    return true;
}

bool monitorAllHumans(toaster_msgs::MonitorAll::Request &req,
        toaster_msgs::MonitorAll::Response & res) {

    monitorAllHumans_ = req.monitorAll;
    if (req.monitorAll)
        ROS_INFO("[agent_monitor][REQUEST] Start monitoring all humans");
    else
        ROS_INFO("[agent_monitor][REQUEST] Stop monitoring all humans");
    return true;
}

bool monitorAllRobots(toaster_msgs::MonitorAll::Request &req,
        toaster_msgs::MonitorAll::Response & res) {

    monitorAllRobots_ = req.monitorAll;
    if (req.monitorAll)
        ROS_INFO("[agent_monitor][REQUEST] Start monitoring all robots");
    else
        ROS_INFO("[agent_monitor][REQUEST] Stop monitoring all robots");
    return true;
}

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


/////////////////////
/////// Main ////////
/////////////////////

int main(int argc, char** argv) {
    // Set this in a ros service
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
    ToasterRobotReader robotRd(node, ROBOT_FULL_CONFIG);
    ToasterObjectReader objectRd(node);



    //Services
    ros::ServiceServer serviceAdd = node.advertiseService("agent_monitor/add_agent", addAgent);
    ROS_INFO("[Request] Ready to add agent to monitor.");

    ros::ServiceServer serviceRemove = node.advertiseService("agent_monitor/remove_agent", removeAgent);
    ROS_INFO("[Request] Ready to remove monitored agent.");

    ros::ServiceServer serviceRemoves = node.advertiseService("agent_monitor/remove_all_agents", removeAllAgents);
    ROS_INFO("[Request] Ready to remove monitored agents.");

    ros::ServiceServer serviceAddJoint = node.advertiseService("agent_monitor/add_joint_to_agent", addJointToAgent);
    ROS_INFO("[Request] Ready to add joint to monitor for agent.");

    ros::ServiceServer serviceRemoveJt = node.advertiseService("agent_monitor/remove_joint_to_agent", removeJointToAgent);
    ROS_INFO("[Request] Ready to remove monitored joint to agent.");

    ros::ServiceServer serviceRemoveJts = node.advertiseService("agent_monitor/remove_all_joints_to_agent", removeAllJointsToAgent);
    ROS_INFO("[Request] Ready to remove monitored joints to agent.");

    ros::ServiceServer servicePrintMonitored = node.advertiseService("agent_monitor/print_all_monitored_agents", printAllMonitoredAgents);
    ROS_INFO("[Request] Ready to print monitored agents.");

    ros::ServiceServer serviceMonitorAllAgents = node.advertiseService("agent_monitor/monitor_all_agents", monitorAllAgents);
    ROS_INFO("[Request] Ready to monitor all agents.");

    ros::ServiceServer serviceMonitorAllHumans = node.advertiseService("agent_monitor/monitor_all_humans", monitorAllHumans);
    ROS_INFO("[Request] Ready to monitor all humans.");

    ros::ServiceServer serviceMonitorAllRobots = node.advertiseService("agent_monitor/monitor_all_robots", monitorAllRobots);
    ROS_INFO("[Request] Ready to monitor all robots.");

    ros::ServiceServer servicePointingTime = node.advertiseService("agent_monitor/pointing_time", pointingTowardTimeRequest);
    ROS_INFO("[Request] Ready to receive timed request for pointing.");

    ros::ServiceServer servicePointing = node.advertiseService("agent_monitor/pointing", pointingTowardRequest);
    ROS_INFO("[Request] Ready to receive request for pointing.");


    ros::Publisher fact_pub = node.advertise<toaster_msgs::FactList>("agent_monitor/factList", 1000);


    // Set this in a ros service?
    ros::Rate loop_rate(30);


    /************************/
    /* Start of the Ros loop*/
    /************************/

    while (node.ok()) {
        toaster_msgs::FactList factList_msg;
        toaster_msgs::Fact fact_msg;
        // We received agentMonitored

        Agent* agntCur;

        //////////////////////////////////////
        //           Updating data          //
        //////////////////////////////////////

        // If we monitor all humans, we add them to the agentsMonitored vector
        if (monitorAllHumans_)
            for (std::map<std::string, Human*>::iterator it = humanRd.lastConfig_.begin(); it != humanRd.lastConfig_.end(); ++it) {
                // If the pointer was updated and is not the swapped one
                if ((it->second->getId() != "") && (std::find(agentsMonitored_.begin(), agentsMonitored_.end(), it->second->getId()) == agentsMonitored_.end())) {
                    agentsMonitored_.push_back(it->first);
                }
            }

        // If we monitor all robots, we add them to the agentsMonitored vector
        if (monitorAllRobots_)
            for (std::map<std::string, Robot*>::iterator it = robotRd.lastConfig_.begin(); it != robotRd.lastConfig_.end(); ++it) { // If the pointer was updated and is not the swapped one
                // If the pointer was updated and is not the swapped one
                if ((it->second->getId() != "") && (std::find(agentsMonitored_.begin(), agentsMonitored_.end(), it->second->getId()) == agentsMonitored_.end())) {
                    agentsMonitored_.push_back(it->first);
                }
            }



        // All the following computation are done for each monitored agents!

        for (std::vector<std::string>::iterator itAgnt = agentsMonitored_.begin(); itAgnt != agentsMonitored_.end(); ++itAgnt) {
            //printf("[agent_monitor][DEBUG] computation for agent %s\n", itAgnt->c_str());
            Agent* agentMonitored;

            bool isHuman = true;

            // Let's find back the monitored agent:
            if (robotRd.lastConfig_.find((*itAgnt)) != robotRd.lastConfig_.end()) {
                agentMonitored = robotRd.lastConfig_[(*itAgnt)];
                isHuman = false;


            } else if (humanRd.lastConfig_.find((*itAgnt)) != humanRd.lastConfig_.end()) {
                agentMonitored = humanRd.lastConfig_[(*itAgnt)];
                isHuman = true;

            } else {
                continue;
            }


            // We verify if the buffer is already there...
            itTRB_ = mapTRBEntity_.find((*itAgnt));
            if (itTRB_ == mapTRBEntity_.end()) {

                //1st time, we initialize variables
                TRBuffer<Entity*> buffAgnt, buffJnt;

                //Swap data
                if (isHuman) {
                    agntCur = humanRd.lastConfig_[(*itAgnt)];
                    humanRd.lastConfig_[(*itAgnt)] = new Human("");
                    //memcpy(agntCur, humanRd.lastConfig_[(*itAgnt)], sizeof (Human));
                } else {
                    agntCur = robotRd.lastConfig_[(*itAgnt)];
                    robotRd.lastConfig_[(*itAgnt)] = new Robot("");
                    //memcpy(agntCur, robotRd.lastConfig_[(*itAgnt)], sizeof (Robot));
                }

                buffAgnt.push_back(agntCur->getTime(), agntCur);
                mapTRBEntity_[agntCur->getId()] = buffAgnt;
                //printf("adding agent named: reader %s, tmp %s, in buffer: %s\n", agntCur->getName().c_str(), agntCur->getName().c_str(), mapTRBEntity_[agntCur->getId()].back()->getName().c_str());



                // We will use directly the joint from the agent buffer
                /*// adding monitored joints to the entity.
                for (std::vector<std::string>::iterator itJnt = mapAgentToJointsMonitored_[ agntCur->getId() ].begin(); itJnt != mapAgentToJointsMonitored_[ agntCur->getId() ].end(); ++itJnt) {
                    jntCur = new Joint(agntCur->skeleton_[(*itJnt)]->getId(), agntCur->getId());
                    memcpy(jntCur, humanRd.lastConfig_[agentMonitored]->skeleton_[jointsMonitoredName[i]], sizeof (Joint));

                    buffJnt.push_back(jntCur->getTime(), jntCur);

                    mapTRBEntity_[jntCur->getId()] = buffJnt;
                    // printf("adding joint named: reader %d %s, tmp %s, in buffer: %s\n", agntCur->skeleton_[jointMonitoredName]->getId(), agntCur->skeleton_[jointMonitoredName]->getName().c_str(), jntCur->getName().c_str(), mapTRBEntity_[jntCur->getId()].back()->getName().c_str());
                    if (jointsMonitoredId.size() < i + 1)
                        jointsMonitoredId.push_back(jntCur->getId());

                }

                for (unsigned int i = 0; i < jointsMonitoredName.size(); i++) {
                    jntCur = new Joint(agntCur->skeleton_[jointsMonitoredName[i]]->getId(), agentMonitored);
                    memcpy(jntCur, humanRd.lastConfig_[agentMonitored]->skeleton_[jointsMonitoredName[i]], sizeof (Joint));

                    buffJnt.push_back(jntCur->getTime(), jntCur);

                    mapTRBEntity_[jntCur->getId()] = buffJnt;
                    // printf("adding joint named: reader %d %s, tmp %s, in buffer: %s\n", agntCur->skeleton_[jointMonitoredName]->getId(), agntCur->skeleton_[jointMonitoredName]->getName().c_str(), jntCur->getName().c_str(), mapTRBEntity_[jntCur->getId()].back()->getName().c_str());
                    if (jointsMonitoredId.size() < i + 1)
                        jointsMonitoredId.push_back(jntCur->getId());

                }*/


                // This module is made for temporal reasoning.
                // We need more data to make computation, so we will end the loop here.

                // This will be done at the end of the for loop
                //ros::spinOnce();
                //loop_rate.sleep();
                continue;


            } else { // Agent is present in mapTRBEntity_
                // If this is a new data we add it to the buffer
                bool newData = false;

                if (isHuman) {
                    // Verify it is not data from the swap
                    if (humanRd.lastConfig_[(*itAgnt)]->getId() != "") {
                        newData = (mapTRBEntity_[(*itAgnt)].back()->getTime() < humanRd.lastConfig_[(*itAgnt)]->getTime());
                    } else {
                        newData = false;
                    }
                } else {
                    if (robotRd.lastConfig_[(*itAgnt)]->getId() != "") {
                        newData = (mapTRBEntity_[(*itAgnt)].back()->getTime() < robotRd.lastConfig_[(*itAgnt)]->getTime());
                    } else {
                        newData = false;
                    }
                }


                if (newData) {

                    //Swap data
                    if (isHuman) {
                        agntCur = humanRd.lastConfig_[(*itAgnt)];
                        humanRd.lastConfig_[(*itAgnt)] = new Human("");
                        //memcpy(agntCur, humanRd.lastConfig_[(*itAgnt)], sizeof (Human));
                    } else {
                        agntCur = robotRd.lastConfig_[(*itAgnt)];
                        robotRd.lastConfig_[(*itAgnt)] = new Robot("");
                        //memcpy(agntCur, robotRd.lastConfig_[(*itAgnt)], sizeof (Robot));
                    }

                    mapTRBEntity_[agntCur->getId()].push_back(agntCur->getTime(), agntCur);
                    //printf("adding human named: reader %s, tmp %s, in buffer: %s\n", agntCur->getName().c_str(), agntCur->getName().c_str(), mapTRBEntity_[agntCur->getId()].back()->getName().c_str());


                    /*
                    // adding monitored joint to the entities.
                    for (unsigned int i = 0; i < jointsMonitoredName.size(); i++) {
                        jntCur = new Joint(humCur->skeleton_[jointsMonitoredName[i]]->getId(), agentMonitored);
                        memcpy(jntCur, humanRd.lastConfig_[agentMonitored]->skeleton_[jointsMonitoredName[i]], sizeof (Joint));

                        mapTRBEntity_[jntCur->getId()].push_back(jntCur->getTime(), jntCur);
                        //printf("adding joint named: reader %d %s, tmp %s, in buffer: %s\n", humCur->skeleton_[jointMonitoredName]->getId(), humCur->skeleton_[jointMonitoredName]->getName().c_str(), jntCur->getName().c_str(), mapTRBEntity_[jntCur->getId()].back()->getName().c_str());
                    }
                     */

                    // If we don't have new data
                    // Do we need this? Or should we update other entities?
                } else {
                    //printf("agent received without greater time: current is %lu < previous is %lu\n", agntCur->getTime(), mapTRBEntity_[agntCur->getId()].back()->getTime());

                    // This will be done at the end of the for loop
                    //ros::spinOnce();
                    //loop_rate.sleep();
                    continue;
                }
            }







            /////////////////////////////////////
            // Update TRBuffer for each entity //
            /////////////////////////////////////


            //ROS_DEBUG("[agent_monitor] updating TRBuffer for each entities\n");

            // for each entity
            //Put the following in a function?


            // For humans
            for (std::map<std::string, Human*>::iterator it = humanRd.lastConfig_.begin(); it != humanRd.lastConfig_.end(); ++it) {

                // if not monitored agent
                if (std::find(agentsMonitored_.begin(), agentsMonitored_.end(), it->first) == agentsMonitored_.end()) {
                    itTRB_ = mapTRBEntity_.find(it->first);

                    // If 1st data
                    if (itTRB_ == mapTRBEntity_.end()) {
                        TRBuffer<Entity*> buffHum;

                        // Data Swap

                        Human * hum = humanRd.lastConfig_[it->first];
                        humanRd.lastConfig_[it->first] = new Human("");
                        //Human * hum = new Human(it->first);
                        //memcpy(hum, humanRd.lastConfig_[it->first], sizeof (Human));

                        buffHum.push_back(hum->getTime(), hum);
                        mapTRBEntity_[it->first] = buffHum;
                        //printf("adding human name: reader %s, tmp %s, in buffer: %s\n", humanRd.lastConfig_[it->first]->getName().c_str(), hum->getName().c_str(), mapTRBEntity_[it->first].back()->getName().c_str());

                        // If this is a new data we add it
                    } else if (mapTRBEntity_[it->first].back()->getTime() < it->second->getTime()) {
                        Human * hum = humanRd.lastConfig_[it->first];
                        humanRd.lastConfig_[it->first] = new Human("");
                        //Human * hum = new Human(it->first);
                        //memcpy(hum, humanRd.lastConfig_[it->first], sizeof (Human));
                        mapTRBEntity_[it->first].push_back(hum->getTime(), hum);
                    }
                }

            }

            // For robots
            for (std::map<std::string, Robot*>::iterator it = robotRd.lastConfig_.begin(); it != robotRd.lastConfig_.end(); ++it) {

                // if not monitored agent
                if (std::find(agentsMonitored_.begin(), agentsMonitored_.end(), it->first) == agentsMonitored_.end()) {
                    itTRB_ = mapTRBEntity_.find(it->first);
                    if (itTRB_ == mapTRBEntity_.end()) {
                        TRBuffer<Entity*> buffRob;

                        Robot* rob = robotRd.lastConfig_[it->first];
                        robotRd.lastConfig_[it->first] = new Robot("");
                        //Robot* rob = new Robot(it->first);
                        //memcpy(rob, robotRd.lastConfig_[it->first], sizeof (Robot));

                        buffRob.push_back(rob->getTime(), rob);
                        mapTRBEntity_[it->first] = buffRob;
                        //printf("adding robot name: reader %s, tmp %s, in buffer: %s\n", rob->getName().c_str(), rob->getName().c_str(), mapTRBEntity_[it->first].back()->getName().c_str());

                        // If this is a new data we add it
                    } else if (mapTRBEntity_[it->first].back()->getTime() < it->second->getTime()) {
                        Robot* rob = robotRd.lastConfig_[it->first];
                        robotRd.lastConfig_[it->first] = new Robot("");
                        //Robot* rob = new Robot(it->first);
                        //memcpy(rob, robotRd.lastConfig_[it->first], sizeof (Robot));
                        mapTRBEntity_[it->first].push_back(rob->getTime(), rob);
                        //printf("adding robot name: reader %s, tmp %s, in buffer: %s\n", rob->getName().c_str(), rob->getName().c_str(), mapTRBEntity_[it->first].back()->getName().c_str());
                    }
                }

            }


            //printf("[agent_monitor] updating TRBuffer for objects\n");

            //  For Objects
            for (std::map<std::string, Object*>::iterator it = objectRd.lastConfig_.begin(); it != objectRd.lastConfig_.end(); ++it) {
                // if in same room as monitored agent and not monitored agent
                //if (roomOfInterest == it->second->getRoomId()) {
                itTRB_ = mapTRBEntity_.find(it->first);
                if (itTRB_ == mapTRBEntity_.end()) {
                    TRBuffer<Entity*> buffObj;

                    Object* obj = objectRd.lastConfig_[it->first];
                    objectRd.lastConfig_[it->first] = new Object("");
                    //Object* obj = new Object(it->first);
                    //memcpy(obj, objectRd.lastConfig_[it->first], sizeof (Object));

                    buffObj.push_back(obj->getTime(), obj);

                    mapTRBEntity_[it->first] = buffObj;
                    //printf("adding object name: reader %s, tmp %s, in buffer: %s\n", objectRd.lastConfig_[it->first]->getName().c_str(), obj->getName().c_str(), mapTRBEntity_[it->first].back()->getName().c_str());

                    // If this is a new data we add it
                } else if (mapTRBEntity_[it->first].back()->getTime() < it->second->getTime()) {
                    Object* obj = objectRd.lastConfig_[it->first];
                    objectRd.lastConfig_[it->first] = new Object("");
                    mapTRBEntity_[it->first].push_back(obj->getTime(), obj);
                    //Object* obj = new Object(it->first);
                    //memcpy(obj, objectRd.lastConfig_[it->first], sizeof (Object));
                    //printf("adding object name: reader %s, tmp %s, in buffer: %s\n", objectRd.lastConfig_[it->first]->getName().c_str(), obj->getName().c_str(), mapTRBEntity_[it->first].back()->getName().c_str());
                }
                //} // TODO: else remove

            }



            //printf("[agent_monitor] updating TRBuffer for objects done\n");





            //////////////////////////////////////////////
            // Compute facts concerning monitored agent //
            //////////////////////////////////////////////




            ROS_DEBUG("[agent_monitor] computing facts for agent %s\n", (*itAgnt).c_str());


            //if (!monitoredBufferInit) {
            //   printf("[AGENT_MONITOR][WARNING] agent monitored not found\n");
            //}else{
            //printf("[AGENT_MONITOR][DEBUG] agent from buffer %s is null? %d \n [AGENT_MONITOR][DEBUG] agent from reader %s is null? %d \n", mapTRBEntity_[(*itAgnt)].back()->getName().c_str(),  mapTRBEntity_[(*itAgnt)].back() == NULL, humanRd.lastConfig_[(*itAgnt)]->getName().c_str(), humanRd.lastConfig_[(*itAgnt)] == NULL);  
            //printf("[AGENT_MONITOR][WARNING] agent monitored buffer size %d, max_size %d, full %d, back is null? %d\n", mapTRBEntity_[(*itAgnt)].size(), mapTRBEntity_[(*itAgnt)].max_size(), mapTRBEntity_[(*itAgnt)].full(), mapTRBEntity_[(*itAgnt)].back() == NULL);



            double angleDirection = 0.0;
            std::map<std::string, double> mapIdValue;
            double radAngle=(PI*30.0)/180.0;
            mapIdValue = computeIsLookingToward(mapTRBEntity_,(*itAgnt),2,radAngle);
            for (std::map<std::string, double>::iterator it = mapIdValue.begin(); it != mapIdValue.end(); ++it)
            {
                ROS_INFO("%s is looking toward %s",(*itAgnt).c_str(),it->first.c_str());
                fact_msg.property = "IsLookingToward";
                fact_msg.propertyType = "attention";
                fact_msg.subProperty = "agent";
                fact_msg.stringValue = "";
                fact_msg.subjectId = (*itAgnt);
                fact_msg.targetId = it->first;
                fact_msg.confidence = (radAngle - it->second) / radAngle;
                fact_msg.doubleValue = it->second;
                fact_msg.time = mapTRBEntity_[(*itAgnt)].back()->getTime();
                fact_msg.subjectOwnerId = "";
                fact_msg.targetOwnerId = "";

                factList_msg.factList.push_back(fact_msg);
            }

            // If the agent is moving
            double speed = computeMotion2D(mapTRBEntity_[(*itAgnt)], oneSecond / 4);

            //We consider motion when it moves more than 3 cm during 1/4 second, so when higher than 0.12 m/s
            if (speed > (0.12)) {
                //printf("[AGENT_MONITOR][DEBUG] %s is moving %lu\n", mapTRBEntity_[(*itAgnt)].back()->getName().c_str(), mapTRBEntity_[(*itAgnt)].back()->getTime());

                double confidence = speed * 3.6 / 20.0; // Confidence is 1 if speed is 20 km/h or above
                if (confidence > 1.0)
                    confidence = 1.0;

                //Fact moving
                fact_msg.property = "IsMoving";
                fact_msg.propertyType = "motion";
                fact_msg.subProperty = "agent";
                fact_msg.subjectId = (*itAgnt);
                fact_msg.stringValue = "true";
                fact_msg.doubleValue = speed;
                fact_msg.confidence = confidence;
                fact_msg.time = mapTRBEntity_[(*itAgnt)].back()->getTime();
                fact_msg.subjectOwnerId = "";
                fact_msg.targetOwnerId = "";

                factList_msg.factList.push_back(fact_msg);

                // We compute the direction toward fact:
                angleDirection = computeMotion2DDirection(mapTRBEntity_[(*itAgnt)], oneSecond);
                mapIdValue = computeMotion2DToward(mapTRBEntity_, (*itAgnt), angleDirection, 0.5);
                for (std::map<std::string, double>::iterator it = mapIdValue.begin(); it != mapIdValue.end(); ++it) {
                    //printf("[AGENT_MONITOR][DEBUG] %s is moving toward %s with a confidence of %f\n",
                    //        mapTRBEntity_[(*itAgnt)].back()->getName().c_str(), mapTRBEntity_[it->first].back()->getName().c_str(), it->second);

                    //filter to get a minimal motion

                    if (it->second > 0.01) {

                        //Fact moving toward
                        fact_msg.property = "IsMovingToward";
                        fact_msg.propertyType = "motion";
                        fact_msg.subProperty = "direction";
                        fact_msg.subjectId = (*itAgnt);
                        fact_msg.targetId = it->first;
                        fact_msg.confidence = it->second;
                        fact_msg.doubleValue = it->second;
                        fact_msg.time = mapTRBEntity_[(*itAgnt)].back()->getTime();
                        fact_msg.subjectOwnerId = "";
                        fact_msg.targetOwnerId = "";

                        factList_msg.factList.push_back(fact_msg);
                    }
                }


                // We compute /_\distance toward entities
                mapIdValue = computeDeltaDist(mapTRBEntity_, (*itAgnt), oneSecond / 4);
                for (std::map<std::string, double>::iterator it = mapIdValue.begin(); it != mapIdValue.end(); ++it) {


                    //filter to get a minimal motion
                    double value = 0.0;
                    if (it->second > 0.01) {
                        value = it->second;

                        //Fact moving toward
                        fact_msg.property = "IsMovingToward";
                        fact_msg.propertyType = "motion";
                        fact_msg.subProperty = "distance";
                        fact_msg.subjectId = (*itAgnt);
                        fact_msg.targetId = it->first;
                        fact_msg.confidence = value;
                        fact_msg.doubleValue = value;
                        fact_msg.time = mapTRBEntity_[(*itAgnt)].back()->getTime();
                        fact_msg.subjectOwnerId = "";
                        fact_msg.targetOwnerId = "";

                        factList_msg.factList.push_back(fact_msg);
                    }
                }


                // If agent is not moving, we compute his joint motion
                // TODO: do this in 3D!
            } else {

                double dist3D;
                std::string dist3DString;

                // What is the distance between joints and objects?
                for (std::vector<std::string>::iterator itJnt = mapAgentToJointsMonitored_[(*itAgnt)].begin(); itJnt != mapAgentToJointsMonitored_[(*itAgnt)].end(); ++itJnt) {

                    Joint* curMonitoredJnt = ((Agent*) mapTRBEntity_[(*itAgnt)].back())->skeleton_[(*itJnt)];
                    for (std::map<std::string, TRBuffer < Entity*> >::iterator itEnt = mapTRBEntity_.begin(); itEnt != mapTRBEntity_.end(); ++itEnt) {
                        // if in same room as monitored agent and not monitored joint
                        //if ((roomOfInterest == it->second.back()->getRoomId()) && (it->first != jointsMonitoredId[i])) {
                        dist3D = bg::distance(curMonitoredJnt->getPosition(), itEnt->second.back()->getPosition());

                        if (dist3D < 0.05)
                            dist3DString = "reach";
                        else if (dist3D < 0.2)
                            dist3DString = "close";
                        else if (dist3D < 1.5)
                            dist3DString = "medium";
                        else if (dist3D < 8)
                            dist3DString = "far";
                        else
                            dist3DString = "out";

                        //Fact distance
                        fact_msg.property = "Distance";
                        fact_msg.propertyType = "position";
                        fact_msg.subProperty = "3D";
                        fact_msg.subjectId = curMonitoredJnt->getId();
                        fact_msg.targetId = itEnt->first;
                        fact_msg.subjectOwnerId = curMonitoredJnt->getAgentId();

                        fact_msg.valueType = 0;
                        fact_msg.stringValue = dist3DString;
                        fact_msg.doubleValue = dist3D;
                        fact_msg.confidence = 0.90;
                        fact_msg.time = curMonitoredJnt->getTime();

                        factList_msg.factList.push_back(fact_msg);

                        //}
                    }
                    // Is the joint moving?
                    speed = computeJointMotion2D(mapTRBEntity_[(*itAgnt)], (*itJnt), oneSecond / 4);

                    //We consider motion when it moves more than 3 cm during 1/4 second, so when higher than 0.12 m/s
                    if (speed > (0.12)) {
                        //   printf("[AGENT_MONITOR][DEBUG] %s of agent %s is moving %lu\n", (*itJnt).c_str(), mapTRBEntity_[(*itAgnt)].back()->getName().c_str(), mapTRBEntity_[(*itAgnt)].back()->getTime());

                        double confidence = speed * 3.6 / 20.0; // Confidence is 1 if speed is 20 km/h or above
                        if (confidence > 1.0)
                            confidence = 1.0;

                        //Fact moving
                        fact_msg.property = "IsMoving";
                        fact_msg.propertyType = "motion";
                        fact_msg.subProperty = "joint";
                        fact_msg.subjectId = curMonitoredJnt->getId();
                        fact_msg.subjectOwnerId = curMonitoredJnt->getAgentId();
                        fact_msg.valueType = 0;
                        fact_msg.stringValue = "true";
                        fact_msg.doubleValue = speed;
                        fact_msg.confidence = confidence;
                        fact_msg.time = curMonitoredJnt->getTime();

                        factList_msg.factList.push_back(fact_msg);


                        double angleDirection = 0.0;

                        // We compute the direction toward fact:
                        angleDirection = computeJointMotion2DDirection(mapTRBEntity_[(*itAgnt)], (*itJnt), oneSecond);
                        mapIdValue = computeJointMotion2DToward(mapTRBEntity_, (*itAgnt), (*itJnt), angleDirection, 0.5);
                        for (std::map<std::string, double>::iterator it = mapIdValue.begin(); it != mapIdValue.end(); ++it) {
                            // printf("[AGENT_MONITOR][DEBUG] %s of agent %s is moving toward %s with a confidence of %f\n", (*itJnt).c_str(),
                            //         mapTRBEntity_[(*itAgnt)].back()->getName().c_str(), mapTRBEntity_[it->first].back()->getName().c_str(), it->second);

                            //Fact moving toward
                            fact_msg.property = "IsMovingToward";
                            fact_msg.propertyType = "motion";
                            fact_msg.subProperty = "direction";
                            fact_msg.subjectId = curMonitoredJnt->getId();
                            fact_msg.targetId = it->first;
                            fact_msg.subjectOwnerId = curMonitoredJnt->getAgentId();
                            fact_msg.confidence = it->second;
                            fact_msg.time = curMonitoredJnt->getTime();

                            factList_msg.factList.push_back(fact_msg);
                        }

                        // Then we compute /_\distance
                        mapIdValue = computeJointDeltaDist(mapTRBEntity_, (*itAgnt), (*itJnt), oneSecond / 4);
                        for (std::map<std::string, double>::iterator it = mapIdValue.begin(); it != mapIdValue.end(); ++it) {
                            //printf("[AGENT_MONITOR][DEBUG] joint %s of agent %s has a deltadist toward  %s of %f\n", (*itJnt).c_str(),
                            //         mapTRBEntity_[(*itAgnt)].back()->getName().c_str(), mapTRBEntity_[it->first].back()->getName().c_str(), it->second);

                            //Fact moving toward
                            fact_msg.property = "IsMovingToward";
                            fact_msg.propertyType = "motion";
                            fact_msg.subProperty = "distance";
                            fact_msg.subjectId = curMonitoredJnt->getId();
                            fact_msg.targetId = it->first;
                            fact_msg.subjectOwnerId = curMonitoredJnt->getAgentId();
                            fact_msg.confidence = it->second;
                            fact_msg.time = curMonitoredJnt->getTime();
                        }
                    } // Joint moving
                } // All monitored joints
            } // Joints or full agent?

        } // each monitored agents
        //publish only if we have something
        //if (!factList_msg.factList.empty())
        fact_pub.publish(factList_msg);

        ros::spinOnce();

        loop_rate.sleep();

    }
    return 0;
}
