cmake_minimum_required(VERSION 2.8.3)
project(toaster_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
   FILES
   Agent.msg
   Area.msg
   Entity.msg
   FactList.msg
   Fact.msg
   HumanList.msg
   Human.msg
   Joint.msg
   ObjectList.msg
   Object.msg
   RobotList.msg
   Robot.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  AddAgent.srv
  AddFact.srv
  AddJointToAgent.srv
  MonitorAll.srv
  PrintAgents.srv 
  PrintAreas.srv  
  RemoveAgents.srv  
  RemoveFact.srv 
  RemoveJointsToAgent.srv
  AddArea.srv
  AddFactToAgent.srv 
  GetRelativePosition.srv
  PrintAgentJoints.srv
  PrintArea.srv
  RemoveAgent.srv
  RemoveArea.srv
  RemoveFactToAgent.srv
  RemoveJointToAgent.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime 
)
