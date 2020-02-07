#include <string>
#include <map>
#include <vector>

#include <ros/ros.h>

#include "toaster_msgs/MonitorAll.h"
#include "toaster_msgs/Empty.h"
#include "toaster_msgs/RemoveJointToAgent.h"
#include "toaster_msgs/RemoveAllJointsToAgent.h"
#include "toaster_msgs/AddAgent.h"
#include "toaster_msgs/AddJointToAgent.h"
#include "toaster_msgs/RemoveAgent.h"

using namespace std;

class AgentManager
{
public:
  AgentManager();
  ~AgentManager() {};

  void init(ros::NodeHandle* node);

  bool startMonitorAllHumans();
  bool stopMonitorAllHumans();
  bool startMonitorAllRobots();
  bool stopMonitorAllRobots();

  vector<string> getMonitoredAgents() { return agentsMonitored_; }
  map<string, vector<string> > getMonitoredJoints() { return mapAgentToJointsMonitored_;}

  bool addMonitoredAgent(string id);
  bool removeMonitoredAgent(string id);

private:
  bool startMonitorAllHumans_;
  bool stopMonitorAllHumans_;
  bool startMonitorAllRobots_;
  bool stopMonitorAllRobots_;

  vector<string> agentsMonitored_;
  map<string, vector<string> > mapAgentToJointsMonitored_;

  ros::NodeHandle* node_;

  ros::ServiceServer serviceAdd_;
  ros::ServiceServer serviceAddJoint_;
  bool addAgent(toaster_msgs::AddAgent::Request &req,
                toaster_msgs::AddAgent::Response & res);
  bool addJointToAgent(toaster_msgs::AddJointToAgent::Request &req,
                      toaster_msgs::AddJointToAgent::Response & res);

  ros::ServiceServer serviceRemove_;
  ros::ServiceServer serviceRemoves_;
  bool removeAgent(toaster_msgs::RemoveAgent::Request &req,
                  toaster_msgs::RemoveAgent::Response & res);
  bool removeAllAgents(toaster_msgs::Empty::Request &req,
                      toaster_msgs::Empty::Response & res);

  ros::ServiceServer serviceRemoveJts_;
  ros::ServiceServer serviceRemoveJt_;
  bool removeJointToAgent(toaster_msgs::RemoveJointToAgent::Request &req,
                          toaster_msgs::RemoveJointToAgent::Response & res);
  bool removeAllJointsToAgent(toaster_msgs::RemoveAllJointsToAgent::Request &req,
                              toaster_msgs::RemoveAllJointsToAgent::Response & res);

  ros::ServiceServer servicePrintMonitored_;
  bool printAllMonitoredAgents(toaster_msgs::Empty::Request &req,
                              toaster_msgs::Empty::Response & res);

  ros::ServiceServer serviceMonitorAllAgents_;
  ros::ServiceServer serviceMonitorAllHumans_;
  ros::ServiceServer serviceMonitorAllRobots_;
  bool monitorAllAgents(toaster_msgs::MonitorAll::Request &req,
                        toaster_msgs::MonitorAll::Response & res);
  bool monitorAllHumans(toaster_msgs::MonitorAll::Request &req,
                        toaster_msgs::MonitorAll::Response & res);
  bool monitorAllRobots(toaster_msgs::MonitorAll::Request &req,
                        toaster_msgs::MonitorAll::Response & res);
};
