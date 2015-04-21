cmake_minimum_required(VERSION 2.8.3)
project(niut_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
   FILES
   niut_GEN_POINT_3D.msg
   niut_HUMAN_LIST.msg
   niut_JOINT.msg
   niut_JOINT_STR.msg
   niut_SKELETON_STR.msg
   niut_TIME_STR.msg
   niut_TRACK_STATE.msg
   niut_USER_STR.msg
)

# Generate services in the 'srv' folder
#add_service_files(
#  FILES
#  GetPersonTrajectories.srv
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime 
)
