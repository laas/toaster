cmake_minimum_required(VERSION 2.8.3)
project(toaster_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)


find_package(Boost REQUIRED COMPONENTS system)

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
  Empty.srv
  RemoveFact.srv 
  RemoveAllJointsToAgent.srv
  AddArea.srv
  AddFactToAgent.srv 
  GetRelativePosition.srv
  PrintArea.srv
  RemoveAgent.srv
  RemoveArea.srv
  RemoveFactToAgent.srv
  RemoveJointToAgent.srv
  AddStream.srv
  PointingTime.srv
  Pointing.srv
  GetFactValue.srv
  GetFacts.srv
  PutInHand.srv
  RemoveFromHand.srv
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
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES toaster_msgs_lib
   CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs message_generation message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
${catkin_INCLUDE_DIRS}
  #${Boost_INCLUDE_DIRS}  
  $ENV{HOME}/devel/include 
  $ENV{HOME}/openrobots/include/
/home/mfiore/devel/src/boost_1_56_0/
)

add_library(toaster_msgs_lib ./src/ToasterFactReader.cpp ./src/ToasterHumanReader.cpp ./src/ToasterObjectReader.cpp ./src/ToasterRobotReader.cpp)
  add_dependencies(toaster_msgs_lib toaster_msgs_generate_messages_cpp)

