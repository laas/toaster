#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream> 
#include <sstream>
#include <algorithm>
#include <string>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sstream>
#include "toaster_msgs/Entity.h"
#include "toaster_msgs/Object.h"
#include "toaster_msgs/Area.h"
#include "geometry_msgs/Point.h"
#include "toaster_msgs/ObjectList.h"
#include "toaster_msgs/Human.h"
#include "toaster_msgs/HumanList.h"
#include <vector> 
#include <map> 
#include <geometry_msgs/Polygon.h>
#include "toaster_msgs/AreaList.h"
#include <geometry_msgs/Point32.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "toaster_com");
	ros::NodeHandle n;
	
	
	toaster_msgs::Area msg4; 
	toaster_msgs::Area msga1;
	
	toaster_msgs::AreaList al;
	
	
	toaster_msgs::Object msg2; 
	toaster_msgs::Object msg3; 
	
	toaster_msgs::ObjectList msg5; 
	
	
	toaster_msgs::Human msgh1; 
	toaster_msgs::Human msgh2; 
	
	toaster_msgs::HumanList msgh; 
		
	
	
	//object 1
	msg2.meEntity.id = "Desk_1";
	msg2.meEntity.name = "Desk";
	msg2.meEntity.positionX = 20.0;
	msg2.meEntity.positionY = 20.0;
	msg2.meEntity.positionZ = 0.0;
	msg2.meEntity.orientationRoll = 0.0;
	msg2.meEntity.orientationPitch = 0.0;
	msg2.meEntity.orientationYaw = 0.0;
	
	//object 2
	msg3.meEntity.id = "IKEA_bed_FJELLSE";
	msg3.meEntity.name = "bed";
	msg3.meEntity.positionX = -10.0;
	msg3.meEntity.positionY = -5.0;
	msg3.meEntity.positionZ = 0.0;
	msg3.meEntity.orientationRoll = 0.0;
	msg3.meEntity.orientationPitch = 0.0;
	msg3.meEntity.orientationYaw = 0.0;
	
	
	//area
	msg4.id = 13;
	msg4.name = "area";
	msg4.isCircle = true;
	msg4.center.x = 15;
	msg4.center.y = 0;
	msg4.center.z = 0;
	msg4.ray = 30.0;
	
	
	
	
	msga1.id = 15;
	msga1.name = "polygon";
	msga1.isCircle = false;
	
	geometry_msgs::Point32 p;
	
	
	
		p.x=  35.0;
		p.y=  35.0;
		p.z=  0.0;
		
	msga1.poly.points.push_back(p);
	
	
		p.x=  -27.0;
		p.y=  15;
		p.z=  0.0;
	
	msga1.poly.points.push_back(p);
	
	
		p.x=  -30.0;
		p.y=  -30.0;
		p.z=  0.0;
		
		
	msga1.poly.points.push_back(p);
	
		p.x=  30.0;
		p.y=  -24;
		p.z=  0.0;
		
	msga1.poly.points.push_back(p);
	
	
	
	
	//human 1
	msgh1.meAgent.meEntity.id = "human2";
	msgh1.meAgent.meEntity.name = "human2";
	msgh1.meAgent.meEntity.positionX = -30.0;
	msgh1.meAgent.meEntity.positionY = -20.0;
	msgh1.meAgent.meEntity.positionZ = 0.0;
	msgh1.meAgent.meEntity.orientationRoll = 0.0;
	msgh1.meAgent.meEntity.orientationPitch = 0.0;
	msgh1.meAgent.meEntity.orientationYaw = 0.0;
	
	
	toaster_msgs::Joint j;
	
	
	j.meEntity.name = "head";
	j.meEntity.id = "head_human2";
	j.meEntity.positionX = -30.0;
	j.meEntity.positionY = -20.0;
	j.meEntity.positionZ = 10.0;
	j.meEntity.orientationRoll  = 90.0;
	j.meEntity.orientationPitch = 0.0;
	j.meEntity.orientationYaw  = 0.0;
	
	msgh1.meAgent.skeletonJoint.push_back(j);
	
	
	j.meEntity.name = "arm1";
	j.meEntity.id = "arm1_human2";
	j.meEntity.positionX = -32.5;
	j.meEntity.positionY = -20.0;
	j.meEntity.positionZ = 7.0;
	j.meEntity.orientationRoll  = 0.0;
	j.meEntity.orientationPitch = 45.0;
	j.meEntity.orientationYaw  = 0.0;
	
	msgh1.meAgent.skeletonJoint.push_back(j);
	
	j.meEntity.name = "arm2";
	j.meEntity.id = "arm2_human2";
	j.meEntity.positionX = -27.5;
	j.meEntity.positionY = -20.0;
	j.meEntity.positionZ = 7.0;
	j.meEntity.orientationRoll  = 0.0;
	j.meEntity.orientationPitch = -45.0;
	j.meEntity.orientationYaw  = 0.0;
	
	msgh1.meAgent.skeletonJoint.push_back(j);
	
	j.meEntity.name = "torso";
	j.meEntity.id = "torso_human2";
	j.meEntity.positionX = -30.0;
	j.meEntity.positionY = -20.0;
	j.meEntity.positionZ = 5.0;
	j.meEntity.orientationRoll  = 0.0;
	j.meEntity.orientationPitch = 0.0;
	j.meEntity.orientationYaw  = 0.0;
	
	msgh1.meAgent.skeletonJoint.push_back(j);
	
	j.meEntity.name = "leg1";
	j.meEntity.id = "leg1_human2";
	j.meEntity.positionX = -27.5;
	j.meEntity.positionY = -20.0;
	j.meEntity.positionZ = 2.5;
	j.meEntity.orientationRoll  = 0.0;
	j.meEntity.orientationPitch = 0.0;
	j.meEntity.orientationYaw  = 0.0;
	
	msgh1.meAgent.skeletonJoint.push_back(j);
	
	j.meEntity.name = "leg2";
	j.meEntity.id = "leg2_human2";
	j.meEntity.positionX = -32.5;
	j.meEntity.positionY = -20.0;
	j.meEntity.positionZ = 2.5;
	j.meEntity.orientationRoll  = 0.0;
	j.meEntity.orientationPitch = 0.0;
	j.meEntity.orientationYaw  = 0.0;
	
	msgh1.meAgent.skeletonJoint.push_back(j);
	
	
	//msgh1.meAgent.skeletonJoint[0].meEntity.name = "bras";
	//msgh1.meAgent.skeletonJoint[0].meEntity.id = 4;
	//msgh1.meAgent.skeletonJoint[0].meEntity.positionX = 0.0;
	//msgh1.meAgent.skeletonJoint[0].meEntity.positionY = -20.0;
	//msgh1.meAgent.skeletonJoint[0].meEntity.positionZ = 55.0;
	//msgh1.meAgent.skeletonJoint[0].meEntity.= 0.0;
	//msgh1.meAgent.skeletonJoint[0].meEntity. = 0.0;
	//msgh1.meAgent.skeletonJoint[0].meEntity. 0.0;
			
	//msgh1.meAgent.skeletonJoint[1].meEntity.id = 5;
	//msgh1.meAgent.skeletonJoint[1].meEntity.positionX = 0.0;
	//msgh1.meAgent.skeletonJoint[1].meEntity.positionY = -20.0;
	//msgh1.meAgent.skeletonJoint[1].meEntity.positionZ = 2.5;
	//msgh1.meAgent.skeletonJoint[1].meEntity.orientationRoll = 0.0;
	//msgh1.meAgent.skeletonJoint[1].meEntity.orientationPitch = 0.0;
	//msgh1.meAgent.skeletonJoint[1].meEntity.orientationYaw = 0.0;
	
	
	
	//human 2
	msgh2.meAgent.meEntity.id = "human1";
	msgh2.meAgent.meEntity.name = "Georges";
	msgh2.meAgent.meEntity.positionX = 15.0;
	msgh2.meAgent.meEntity.positionY = 0.0;
	msgh2.meAgent.meEntity.positionZ = 0.0;
	msgh2.meAgent.meEntity.orientationRoll = 0.0;
	msgh2.meAgent.meEntity.orientationPitch = 0.0;
	msgh2.meAgent.meEntity.orientationYaw = 0.0;
	
	
	
	
	//publishers
	ros::Publisher chatter_pub5 = n.advertise<toaster_msgs::ObjectList>("/pdg/objectList", 1000);
	ros::Publisher chatter_pub6 = n.advertise<toaster_msgs::HumanList>("/pdg/humanList", 1000);
	ros::Publisher chatter_pub7 = n.advertise<toaster_msgs::AreaList>("/area_manager/areaList", 1000);
	
	
	al.areaList.push_back(msg4);
	al.areaList.push_back(msga1);
	
	msg5.objectList.push_back(msg2) ;
	msg5.objectList.push_back(msg3) ;
	
	
		
	msgh.humanList.push_back(msgh1) ;
	msgh.humanList.push_back(msgh2) ;
	
	
	
	
	
	ros::Rate loop_rate(30);	
	
	while(ros::ok())
	{
		
		
		chatter_pub5.publish(msg5);
		chatter_pub6.publish(msgh);
		chatter_pub7.publish(al);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	
	return 0;
}



