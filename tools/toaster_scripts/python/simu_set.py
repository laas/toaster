#!/usr/bin/env python

import sys
import rospy
import tf
from toaster_msgs.srv import *
from geometry_msgs.msg import Pose

def set_pose_client(id,name,type,owner,x,y,z,roll,pitch,yaw):
  try:
    add_entity=rospy.ServiceProxy("/toaster_simu/add_entity",AddEntity)
    res=add_entity(id, name, type, owner)

    set_entity_pose=rospy.ServiceProxy("/toaster_simu/set_entity_pose",SetEntityPose)


    pose=Pose()
    pose.position.x=x
    pose.position.y=y
    pose.position.z=z

    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    res=set_entity_pose(id, owner,type,pose)

    
  
  except rospy.ServiceException, e:
    print "Service call failed %s"%e


def set_keyboard_client(id):
  try:
    set_entity_keyboard=rospy.ServiceProxy("/toaster_simu/set_entity_keyboard",AddAgent)
    res=set_entity_keyboard(id)
    
  
  except rospy.ServiceException, e:
    print "Service call failed %s"%e

def set_agent_monitor(id):
  try:
    set_agent_monitor=rospy.ServiceProxy("/agent_monitor/add_agent",AddAgent)
    res=set_agent_monitor(id)
    
  
  except rospy.ServiceException, e:
    print "Service call failed %s"%e


if __name__=="__main__":
  rospy.wait_for_service("/toaster_simu/set_entity_pose")
  rospy.wait_for_service("/toaster_simu/add_entity")

  #Human
  set_pose_client("Bob", "Bob", "human","",8.3,4.4,0.0,0.0,0.0,-1.6)
  set_pose_client("Greg", "Greg","human","",0.0,0.0,0.0,0.0,0.0,1.6)
  #set_pose_client("Alexia", "Alexia", "human","",5.3,4.0,0.0,0.0,0.0,0.0)
  #set_pose_client("Lara", "Lara", "human","",4.3,2.4,0.0,0.0,0.0,-1.0)
  #set_pose_client("Dan", "Dan", "human","",2.3,3.4,0.0,0.0,0.0,0.0)
  set_pose_client("Micky", "Micky", "human","",1.3,2.4,0.0,0.0,0.0,0.0)


  #keyboard
  set_keyboard_client("Greg")
  set_agent_monitor("Greg")
  set_pose_client("TABLE_4","TABLE_4", "object","",11.3,4.4,0.0,0.0,0.0,1.6)
  set_pose_client("GREY_TAPE","GREY_TAPE","object","",7.3,3.4,0.0,0.0,0.0,1.6)
  set_pose_client("LOTR_TAPE","LOTR_TAPE","object","",10.3,2.4,0.0,0.0,0.0,1.6)
  set_pose_client("WALLE_TAPE","WALLE_TAPE","object","",5.3,1.4,0.0,0.0,0.0,1.6)
  #set_pose_client("IKEA_SHELF_LIGHT_1","","object",-3.2,13.9,0,0,0,-0.5)
  #set_pose_client("IKEA_SHELF_LIGHT_2","","object",-0.3,15.3,0,0,0,-1.5)
  #set_pose_client("IKEA_SHELF_LIGHT_3","","object",2.3,13.7,0,0,0,4.1) 
  #set_pose_client("HERAKLES_HUMAN1","","human",-2.2,13.6,0.0,0.0,0.0,0)
  #set_pose_client("PR2_ROBOT","","robot",1.3,12.9,0,0,0,2.9)
