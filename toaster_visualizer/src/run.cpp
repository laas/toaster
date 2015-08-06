#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include <iostream> 
#include <sstream>
#include <algorithm>
#include <string>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <iostream>
#include <vector> 
#include "toaster_msgs/Entity.h"
#include "toaster_msgs/Object.h"
#include "toaster_msgs/Area.h"
#include "toaster_msgs/AreaList.h"
#include "geometry_msgs/Point.h"
#include "toaster_msgs/ObjectList.h"
#include "toaster_msgs/Human.h"
#include "toaster_msgs/HumanList.h"
#include "toaster_msgs/Joint.h"
#include <map> 
#include <geometry_msgs/Polygon.h>
#include <tinyxml.h>
#include <time.h>  



//nameMarker rpoportionnal scale


class Run
{
	visualization_msgs::MarkerArray area_list;
	visualization_msgs::MarkerArray obj_list;
	visualization_msgs::MarkerArray human_list;
		
	//a vector to store allready treated marker's name and an id counter	
	std::vector<std::string> name_list;
	int id_cpt;	
	
	//a vector to store marker's color
	std::vector<std::vector<float> > color_list;
 
 
 	//subscribers	
	ros::Subscriber sub_objList;
	ros::Subscriber sub_areaList;
	ros::Subscriber sub_humanList;
	
	
	//publishers
	ros::Publisher pub_obj;
	ros::Publisher pub_area;
	ros::Publisher pub_human;
	
	public:
       
    /**
     * Constructor of the run class for toaster_vizualiser
     */
	Run()
	{		
		name_list = std::vector<std::string>();
		color_list = std::vector<std::vector<float> >(50);
		id_cpt = 1;

		ros::NodeHandle reception_node;
		ros::NodeHandle emission_node;
		
		area_list =  visualization_msgs::MarkerArray();
		obj_list = visualization_msgs::MarkerArray();
		human_list = visualization_msgs::MarkerArray();
		
		//definition of subscribers
		sub_objList = reception_node.subscribe("/pdg/objectList", 1000, &Run::chatterCallbackObjList, this);
		sub_areaList = reception_node.subscribe("/area_manager/areaList", 1000, &Run::chatterCallbackAreaList, this);
		sub_humanList = reception_node.subscribe("/pdg/humanList", 1000, &Run::chatterCallbackHumanList, this);
		
		//definition of publishers
		pub_obj = emission_node.advertise<visualization_msgs::MarkerArray>("visualisation_obj", 1000);
		pub_area = emission_node.advertise<visualization_msgs::MarkerArray>("visualisation_area", 1000);
		pub_human = emission_node.advertise<visualization_msgs::MarkerArray>("visualisation_human", 1000);	
	}
	




// **************************************** definition function of rviz markers ********************************************************

	/**
     * create a circle marker	
     * @param p  		point from geometry library locating the center of the circle
     * @param rayon  	radius of the circle
     * @param name 		marker's name
     * @return marker 	circle marker with input property
     */
	visualization_msgs::Marker defineCircle(geometry_msgs::Point p, double rayon, std::string name) 
	{
		//declarration 
		visualization_msgs::Marker marker;

		//frame id
		marker.header.frame_id = "map";
		
		//namespace
		std::ostringstream nameSpace;
		nameSpace << name;
		marker.ns = nameSpace.str();
		marker.id = id_generator(name);  //creation of an unique id based on marker's name
		
		//action
		marker.action = visualization_msgs::Marker::ADD;

		//position
 		marker.pose.position.x = p.x;
		marker.pose.position.y = p.y;
		marker.pose.position.z = p.z;
		
		//orientation
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		//color
		marker.color.r= 0.0f;
		marker.color.g= 1.0f;
		marker.color.b= 0.0f;
		marker.color.a= 1.0;

		//dimemsion
		marker.scale.x = rayon;
		marker.scale.y = rayon;
		marker.scale.z = 0.1;
		
		//type
		marker.type = 2;

		marker.lifetime = ros::Duration();
		
		return marker;
	}
	
	
	/**
     * create a polygon marker based on line marker
     * @param poly 		polygon from geometry library
     * @param scale 		thickness of the line
     * @param name 		polygon's name
     * @return marker 	line marker representating input polygon
     */
	visualization_msgs::Marker definePolynome(geometry_msgs::Polygon poly, double scale ,std::string name) 
	{
		//declarration
		visualization_msgs::Marker marker;

		//frame id
		marker.header.frame_id = "map";
		
		//namespace
		std::ostringstream nameSpace;
		nameSpace << name;
		marker.ns = nameSpace.str();
		marker.id = id_generator(name); //creation of an unique id based on marker's name
		
		//action
		marker.action = visualization_msgs::Marker::ADD;

		//orientation
		marker.pose.orientation.w  = 1.0;

		//color
		marker.color.r= 0.0f;
		marker.color.g= 1.0f;
		marker.color.b= 0.0f;
		marker.color.a= 1.0;
		
		//points
		geometry_msgs::Point p;
		for(int i = 0; i<poly.points.size(); i++)
		{
			geometry_msgs::Point p;
			
			p.x= poly.points[i].x;
			p.y= poly.points[i].y;
			p.z= poly.points[i].z;
			
			marker.points.push_back(p);
		}
		
		p.x= poly.points[0].x;
		p.y= poly.points[0].y;
		p.z= poly.points[0].z;
		
		marker.points.push_back(p);

		//scale
		marker.scale.x = scale;
		
		//type
		marker.type = visualization_msgs::Marker::LINE_STRIP;

		marker.lifetime = ros::Duration();
		
		return marker;
	}


	/**
     * create an object marker
     * @param x  		coordinates of object's base in thx x direction
     * @param y			coordinates of object's base in thx y direction
     * @param z 			coordinates of object's base in thx z direction
     * @param scale 		dimension of the marker
     * @param name 		marker's name
     * @return marker 	object marker or mesh marker if the object is in the mesh database
     */
	visualization_msgs::Marker defineObj(int  x, int y, int z, double scale, std::string name) 
	{
		//declarration 
		visualization_msgs::Marker marker;

		//frame id
		marker.header.frame_id = "map";
		
		//namespace
		std::ostringstream nameSpace;
		nameSpace << name;
		marker.ns = nameSpace.str();
		marker.id = id_generator(name);	 //creation of an unique id based on marker's name
		
		//action
		marker.action = visualization_msgs::Marker::ADD;

		//position
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = z;
		
		//orientation
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		//color
		marker.color.r= 0.0f;
		marker.color.g= 1.0f;
		marker.color.b= 1.0f;
		marker.color.a= 1.0;

		//scale
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		
		//type
		marker.type = visualization_msgs::Marker::CUBE; //marker by defaut
		
		//std::stringstream s;
		//s << ros::package::getPath("toaster_visualizer") << "/src/list_obj.xml";
		//TiXmlDocument list(s.str()); //load xml file
		
		TiXmlDocument list("src/toaster/toaster_visualizer/src/list_obj.xml"); //load xml file
		
		if(!list.LoadFile())  
		{
			ROS_WARN_ONCE("Erreur lors du chargement du fichier xml");
			ROS_WARN_ONCE("error # %d",list.ErrorId() );
			ROS_WARN_ONCE("%s",list.ErrorDesc() );
			
		}
		else
		{
			TiXmlHandle hdl(&list);
			TiXmlElement *elem = hdl.FirstChildElement().FirstChildElement().Element();
			
			std::string name_obj;
			std::string mesh_r;
			
			while(elem)   //for each element of the xml file
			{
				 name_obj = elem->Attribute("name");
				 mesh_r = elem->Attribute("mesh_ressource");
				 elem = elem->NextSiblingElement();
				 
				 if(name_obj.compare(name) == 0)   //if there is a 3d model relativ to this object  
				 {			
					marker.scale.x = 1*scale;
					marker.scale.y = 1*scale;
					marker.scale.z = 1*scale;
					 		 
					marker.color.r= 0.0f;
					marker.color.g= 0.0f;
					marker.color.b= 0.0f;
					marker.color.a= 0.0;
					 
					marker.type = visualization_msgs::Marker::MESH_RESOURCE;    //use it as mesh
					marker.mesh_resource = mesh_r;
					marker.mesh_use_embedded_materials = true;
					
					elem = NULL;
				}
			}
			

		}
		
		marker.lifetime = ros::Duration();
		
		return marker;
	}


	/**
     * create a name marker corresponding to the input marker
     * @param marker	marker to which we want to create a name marker
     * @return marker 	name marker
     */
	visualization_msgs::Marker defineName(visualization_msgs::Marker marker) 
	{
		//declaration
		visualization_msgs::Marker nameMarker = marker;
		
		//position
		nameMarker.pose.position.z = nameMarker.pose.position.z + marker.scale.x*1.25;  //put the name marker above target marker
		
		//color
		nameMarker.color.r= marker.color.r*0.7f;		//add some contrast between name marker and input marker
		nameMarker.color.g= marker.color.g*0.7f;
		nameMarker.color.b= marker.color.b*0.7f;
		nameMarker.color.a= 1.0;
		
		
		//id  
		nameMarker.id = - nameMarker.id;  //opposite id to avoid id conflicts
		
		//type
		nameMarker.type = 9;
		
		//text field
		nameMarker.text = marker.ns;
		
		//scale
		nameMarker.scale.z = 1.0*marker.scale.x;  
	   	   
		return nameMarker;
		
	}
	
	
	/**
     * create a human marker
     * @param x  			coordinates of human's base in the x direction
     * @param y			coordinates of human's base in the y direction
     * @param z 			coordinates of human's base in the z direction
     * @param scale 		dimension of the marker
     * @param name 		marker's name
     * @return marker 	mesh marker of human
     */
	visualization_msgs::Marker defineHuman(int  x, int y, int z, double scale, std::string name) 
	{
		
		//declarration
		visualization_msgs::Marker marker;

		//frame id
		marker.header.frame_id = "map";
		
		//namespace
		std::ostringstream nameSpace;
		nameSpace << name;
		marker.ns = nameSpace.str();
		marker.id = id_generator(name);  //creation of an unique id based on marker's name
	
		//action
		marker.action = visualization_msgs::Marker::ADD;

		//position
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = z;
		
		//orientation
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		//color
		marker.color.r= 0.0f;
		marker.color.g= 0.0f;
		marker.color.b= 0.0f;
		marker.color.a= 0.0;

		//scale
		marker.scale.x = 1*scale;
		marker.scale.y = 1*scale;
		marker.scale.z = 1*scale;
		
		//type de marker
		marker.type = visualization_msgs::Marker::MESH_RESOURCE;   
		marker.mesh_resource = "package://toaster_visualizer/mesh/vincent.dae";  //using 3d human model
		marker.mesh_use_embedded_materials = true;

		marker.lifetime = ros::Duration();
		
		return marker;
	}






// ******************************************************** id generator for markers  ********************************************************



	/**
     * In our context names are initialy identifier so we need to create an unique identifier for each input name
     * @param name		name of target 
     * @return id		new identifier or target's identifier if his name allready have been assigned to an identifier
     */
	int id_generator(std::string name)
	{
		if(std::find(name_list.begin(), name_list.end(), name) != name_list.end())
		{
			return find(name_list.begin(), name_list.end(), name) - name_list.begin() +1;
		}
		else
		{
			name_list.push_back(name);
			return id_cpt++;
		}
	}




// ******************************************************** adjustment functions ********************************************************


	/**
     * Function to modify a marker's orientation
     * @param marker		name of target marker
     * @param orientationX value of rotation around x axis
     * @param orientationY value of rotation around y axis
     * @param orientationZ value of rotation around z axis
     * @return marker 		new marker with input modifications
     */
	visualization_msgs::Marker setOrientation(visualization_msgs::Marker marker, float orientationX, float orientationY, float orientationZ)
	{	
		marker.pose.orientation.x = orientationX/90.0;  //divied by 90 so input angles are in degrees
		marker.pose.orientation.y = orientationY/90.0;
		marker.pose.orientation.z = orientationZ/90.0;
		
		marker.pose.orientation.w = 1.0;
	
		return marker;
	}


	/**
     * Function to modify a marker's color
     * @param marker		name of target marker
     * @param r 			value of red coefficient
     * @param g 			value of green coefficient
     * @param b 			value of blue coefficient
     * @return marker 		new marker with input modifications
     */
	visualization_msgs::Marker setColor(visualization_msgs::Marker marker, double r, double g, double b)
	{
		marker.color.r= r;
		marker.color.g= g;
		marker.color.b= b;
		
		return marker;
	}


	/**
     * Function to modify a marker's position
     * @param marker		name of target marker
     * @param x 			coordinates of marker's base in the x direction
     * @param y 			coordinates of marker's base in the y direction
     * @param z 			coordinates of marker's base in the z direction
     * @return marker 		new marker with input modifications
     */
	visualization_msgs::Marker setPosition(visualization_msgs::Marker marker, float x, float y, float z)
	{	
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = z;
		
		return marker;
	}
	
	
	/**
     * Function to modify a marker's size
     * @param marker		name of target marker
     * @param x 			size of the marker in the x direction
     * @param y 			size of the marker in the y direction
     * @param z 			size of the marker in the z direction
     * @return marker 		new marker with input modifications
     */
	visualization_msgs::Marker setSize(visualization_msgs::Marker marker, float x, float y, float z)
	{	
		marker.scale.x = 1*x;
		marker.scale.y = 1*y;
		marker.scale.z = 1*z;
		
		return marker;
	}
	
	
	
	
	/**
     * Function to give and register a random color to input marker
     * @param marker		name of target marker
     * @return marker 		new marker with input modifications
     */
	visualization_msgs::Marker setRandomColor(visualization_msgs::Marker marker)
	{	
		for(int i = 0; i< name_list.size(); i++)
		{
			if((name_list[i] == marker.ns) && color_list[i].empty())
			{	
				color_list[i].push_back(1.0);
				color_list[i].push_back(static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
				color_list[i].push_back(static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
				color_list[i].push_back(static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
				ROS_INFO("yolo");
			}
			else if((name_list[i] == marker.ns) && !color_list[i].empty())
			{
				marker.color.r= color_list[i][1];
				marker.color.g= color_list[i][2];
				marker.color.b= color_list[i][3];
			}
		}
		return marker;
	}
	

	
	
	







//******************************************************** emission/reception ********************************************************

	//reception
	
	
	
	
	
	
	/**
     * CallBack creating markers based on received toaster_msgs and adding then to obj_list
     * @param msg			reference to receive toaster_msgs::ObjectList
     * @return 			void
     */
	void chatterCallbackObjList(const toaster_msgs::ObjectList::ConstPtr& msg) //toaster object list reception
	{	  
		obj_list.markers.clear();
		
		for(int i = 0; i<msg->objectList.size(); i++)
		{
			visualization_msgs::Marker m = defineObj(msg->objectList[i].meEntity.positionX, msg->objectList[i].meEntity.positionY, msg->objectList[i].meEntity.positionZ, 
			1, msg->objectList[i].meEntity.name);
				
			m = setOrientation(m, msg->objectList[i].meEntity.orientationRoll, msg->objectList[i].meEntity.orientationPitch, msg->objectList[i].meEntity.orientationYaw);
			
			visualization_msgs::Marker mn = defineName(m);
			mn = setColor(mn, 1.0, 1.0, 1.0);
			//mn = setSize(mn, 0, 0, 0.5);
			
			obj_list.markers.push_back(mn);
			obj_list.markers.push_back(m);
			
			ROS_INFO("obj %d", m.id);
		}
	}
	
	
	/**
     * CallBack creating markers based on received toaster_msgs and adding then to area_list
     * There is two differents types of area markers so this function manages both circle and polygon types
     * @param msg			reference to receive toaster_msgs::AreaList
     * @return 			void
     */
	void chatterCallbackAreaList(const toaster_msgs::AreaList::ConstPtr& msg)
	{	  
		area_list.markers.clear();
		
		for(int i = 0; i<msg->areaList.size(); i++)
		{

                if(msg->areaList[i].isCircle == true) //circle case
                {
					visualization_msgs::Marker m = defineCircle(msg->areaList[i].center, 
					msg->areaList[i].ray, msg->areaList[i].name);
					
					m = setRandomColor(m);
					
					visualization_msgs::Marker mn = defineName(m);
					mn = setSize(mn, 0.0, 0.0, 1);
					mn = setPosition(mn, mn.pose.position.x, mn.pose.position.y, 1);  
					
					area_list.markers.push_back(m);
					area_list.markers.push_back(mn);
					
					ROS_INFO("circle %d", m.id);
				}
				else // polygon case
				{
					visualization_msgs::Marker m = definePolynome(msg->areaList[i].poly, 0.2, msg->areaList[i].name);
					
					m = setRandomColor(m);
					
					visualization_msgs::Marker mn = defineName(m);
					mn = setSize(mn, 0.0, 0.0, 1);
					
					int posx = 0;
					int posy = 0;
					
					for(int i = 0; i<m.points.size(); i++)
					{
						posx = posx + m.points[i].x;
						posy = posy + m.points[i].y;
					}
					
					mn = setPosition(mn, posx/m.points.size() , posy/m.points.size(), 1); 
					
					area_list.markers.push_back(m);
					area_list.markers.push_back(mn);
					
					ROS_INFO("poly %d", m.id);
				}
		}
	} 
	
	
	/**
     * CallBack creating markers based on received toaster_msgs and adding then to human_list
     * Humans can be representated by a single unarticulated mesh or by mutiple meshs for an articulated model
     * @param msg			reference to receive toaster_msgs::HumanList
     * @return 			void
     */
	void chatterCallbackHumanList(const toaster_msgs::HumanList::ConstPtr& msg) 
	{
		human_list.markers.clear();
		
		for(int i = 0; i<msg->humanList.size(); i++)
		{
				//non articulated human
				visualization_msgs::Marker m = defineHuman(msg->humanList[i].meAgent.meEntity.positionX, msg->humanList[i].meAgent.meEntity.positionY, msg->humanList[i].meAgent.meEntity.positionZ, 
				0.3, msg->humanList[i].meAgent.meEntity.name);
				
				m = setPosition(m, m.pose.position.x, m.pose.position.y, 0.1);
				
				m = setOrientation(m,  msg->humanList[i].meAgent.meEntity.orientationRoll, msg->humanList[i].meAgent.meEntity.orientationPitch, msg->humanList[i].meAgent.meEntity.orientationYaw);

				visualization_msgs::Marker mn = defineName(m);
				mn = setPosition(mn, mn.pose.position.x, mn.pose.position.y, 3);
				mn = setSize(mn, 0,0,1);
				mn = setColor(mn, 1.0, 1.0, 1.0);
				
				
				human_list.markers.push_back(mn);
				
			if(msg->humanList[i].meAgent.skeletonJoint.size() == 0)
			{
				human_list.markers.push_back(m);
				
				ROS_INFO("human %d", m.id);
			}
			else  //articulated human
			{
				std::vector<toaster_msgs::Joint> membres = msg->humanList[i].meAgent.skeletonJoint ;
				visualization_msgs::Marker markerTempo;
				
				int scale = 1;
				
				for(int y = 0; y<membres.size(); y++)
				{
					ROS_INFO("Membre");

					std::string name = msg->humanList[i].meAgent.skeletonJoint[y].meEntity.name;
					
					//frame id
					markerTempo.header.frame_id = "map";
					
					//namespace
					std::ostringstream nameSpace;
					nameSpace << name;
					markerTempo.ns = nameSpace.str();
					markerTempo.id = id_generator(membres[y].meEntity.name); 	//creation of an unique id based on marker's name
					
					//action
					markerTempo.action = visualization_msgs::Marker::ADD;

					//position
					markerTempo = setPosition(markerTempo, membres[y].meEntity.positionX, membres[y].meEntity.positionY, membres[y].meEntity.positionZ);
					
					//orientation
					markerTempo = setOrientation(markerTempo, membres[y].meEntity.orientationRoll, membres[y].meEntity.orientationPitch, membres[y].meEntity.orientationYaw);

					//color
					markerTempo.color.r= 1.0f;
					markerTempo.color.g= 1.0f;
					markerTempo.color.b= 1.0f;
					markerTempo.color.a= 1.0;

					//scale
					markerTempo.scale.x = 1*scale;
					markerTempo.scale.y = 1*scale;
					markerTempo.scale.z = 1*scale;
					
					//type de marker
					markerTempo.type = 3;
					
					
					//std::stringstream s;
					//s << ros::package::getPath("toaster_visualizer") << "/src/list_members.xml";
					//TiXmlDocument list(s.str()); //load xml file
					
					TiXmlDocument list("src/toaster/toaster_visualizer/src/list_members.xml"); //load xml file
					
					
					if(!list.LoadFile())  
					{
						ROS_WARN_ONCE("Erreur lors du chargement du fichier xml");
						ROS_WARN_ONCE("error # %d",list.ErrorId() );
						ROS_WARN_ONCE("%s",list.ErrorDesc() );
					}
					else
					{
						TiXmlHandle hdl(&list);
						TiXmlElement *elem = hdl.FirstChildElement().FirstChildElement().Element();
						
						std::string name_obj;
						std::string mesh_r;
						
						while(elem)   //for each element of the xml file
						{
							
							 name_obj = elem->Attribute("name");
							 mesh_r = elem->Attribute("mesh_ressource");
							 elem = elem->NextSiblingElement();
							 
							 if(name_obj.compare(name) == 0)   //if there is a 3d model relativ to this object  
							 {		
								ROS_INFO("YOLOOOOOO");	
								markerTempo.scale.x = 1*scale;
								markerTempo.scale.y = 1*scale;
								markerTempo.scale.z = 1*scale;
										 
								markerTempo.color.r= 0.0f;
								markerTempo.color.g= 0.0f;
								markerTempo.color.b= 0.0f;
								markerTempo.color.a= 0.0;
								 
								markerTempo.type = visualization_msgs::Marker::MESH_RESOURCE;    //use it as mesh
								markerTempo.mesh_resource = mesh_r;
								markerTempo.mesh_use_embedded_materials = true;
								
								elem = NULL;
							}
						}
					}

					markerTempo.lifetime = ros::Duration();
					
					human_list.markers.push_back(markerTempo);
				}
			}
		}
	}
	
	
	
	
	
	
			
			
	/**
     * Function sending all marker list to rviz
     * @return 			void
     */
	void send()
	{
		pub_area.publish(area_list);
		pub_obj.publish(obj_list);
		pub_human.publish(human_list);
		
		ros::spinOnce();
	}
};

	
	
	
	
	
/**
* Main function using class run
*/	
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Run");
	
	Run c = Run();
	
	
	ros::Rate loop_rate(30);	
	
	while(ros::ok())
	{
		c.send();
		loop_rate.sleep();
	}
	
	return 0;
}
		

