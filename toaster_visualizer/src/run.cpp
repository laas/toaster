#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
//std
#include <iostream>
#include <sstream>
#include <algorithm>
#include <string>
#include <vector>
#include <time.h>
#include <map>
//visualization_msgs
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
//toaster_msgs
#include "toaster_msgs/Entity.h"
#include "toaster_msgs/Object.h"
#include "toaster_msgs/AreaList.h"
#include "toaster_msgs/ObjectListStamped.h"
#include "toaster_msgs/HumanListStamped.h"
#include "toaster_msgs/FactList.h"
#include "toaster_msgs/RobotListStamped.h"
#include "toaster_msgs/Joint.h"
#include "toaster_msgs/Empty.h"
#include "toaster_msgs/Scale.h"
//geometry_msgs
#include "geometry_msgs/Point.h"
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Quaternion.h>

#include <tinyxml.h>
#include <tf/transform_listener.h>

//nameMarker rpoportionnal scale

class Run {
    visualization_msgs::MarkerArray area_list;
    visualization_msgs::MarkerArray obj_list;
    visualization_msgs::MarkerArray human_list;
    visualization_msgs::MarkerArray robot_list;
    visualization_msgs::MarkerArray arrow_list;

    std::vector<toaster_msgs::Fact> factList;

    //a vector to store allready treated marker's name and an id counter
    std::vector<std::string> name_list;
    int id_cpt;
    float objectNameScale_;
    float humanNameScale_;
    float robotNameScale_;
    float areaNameScale_;
    bool printNames_;

    //a vector to store marker's color
    std::map<std::string, std::vector<float> > color_map;
    std::map<std::string, double> agentMoving_map;


    //subscribers
    ros::Subscriber sub_objList;
    ros::Subscriber sub_areaList;
    ros::Subscriber sub_humanList;
    ros::Subscriber sub_robotList;
    ros::Subscriber sub_factList;
    ros::Subscriber sub_agentFactList;

    //publishers
    ros::Publisher pub_obj;
    ros::Publisher pub_area;
    ros::Publisher pub_human;
    ros::Publisher pub_robot;
    ros::Publisher pub_movingTwrd;

    TiXmlDocument listObj;
    TiXmlDocument listHuman;
    TiXmlDocument listJoint;
    TiXmlDocument listRobot;

public:

    /**
     * Constructor of the run class for toaster_visualizer
     */
    Run(ros::NodeHandle& node) {
        name_list = std::vector<std::string>();
        id_cpt = 1;
        printNames_ = true;


        objectNameScale_ = 0.2;
        humanNameScale_ = 0.3;
        robotNameScale_ = 0.3;
        areaNameScale_ = 0.3;

        area_list = visualization_msgs::MarkerArray();
        obj_list = visualization_msgs::MarkerArray();
        human_list = visualization_msgs::MarkerArray();
        robot_list = visualization_msgs::MarkerArray();
        arrow_list = visualization_msgs::MarkerArray();

        //definition of subscribers
        sub_objList = node.subscribe("/pdg/objectList", 1000, &Run::chatterCallbackObjList, this);
        sub_areaList = node.subscribe("/area_manager/areaList", 1000, &Run::chatterCallbackAreaList, this);
        sub_humanList = node.subscribe("/pdg/humanList", 1000, &Run::chatterCallbackHumanList, this);
        sub_robotList = node.subscribe("/pdg/robotList", 1000, &Run::chatterCallbackRobotList, this);
        sub_factList = node.subscribe("/pdg/factList", 1000, &Run::chatterCallbackFactList, this);
        sub_agentFactList = node.subscribe("/agent_monitor/factList", 1000, &Run::chatterCallbackAgentFactList, this);

        //definition of publishers
        pub_obj = node.advertise<visualization_msgs::MarkerArray>("/toaster_visualizer/marker_object", 1000);
        pub_area = node.advertise<visualization_msgs::MarkerArray>("/toaster_visualizer/marker_area", 1000);
        pub_human = node.advertise<visualization_msgs::MarkerArray>("/toaster_visualizer/marker_human", 1000);
        pub_robot = node.advertise<visualization_msgs::MarkerArray>("/toaster_visualizer/marker_robot", 1000);
        pub_movingTwrd = node.advertise<visualization_msgs::MarkerArray>("/toaster_visualizer/marker_motion", 1000);


        // ********************************** Services ****************************************//


        // **************************************** definition function of rviz markers ********************************************************

        //open xml files

        // Objects
        std::stringstream pathObj;
        pathObj << ros::package::getPath("toaster_visualizer") << "/src/list_obj.xml";
        listObj = TiXmlDocument(pathObj.str());

        if (!listObj.LoadFile()) {
            ROS_WARN_ONCE("Error while loading xml file");
            ROS_WARN_ONCE("error # %d", listObj.ErrorId());
            ROS_WARN_ONCE("%s", listObj.ErrorDesc());
        }


        // Humans
        std::stringstream pathHuman;
        pathHuman << ros::package::getPath("toaster_visualizer") << "/src/list_human.xml";
        listHuman = TiXmlDocument(pathHuman.str());

        if (!listHuman.LoadFile()) {
            ROS_WARN_ONCE("Error while loading xml file");
            ROS_WARN_ONCE("error # %d", listHuman.ErrorId());
            ROS_WARN_ONCE("%s", listHuman.ErrorDesc());
        }

        // Humans
        std::stringstream pathHumanJoint;
        pathHumanJoint << ros::package::getPath("toaster_visualizer") << "/src/list_human_joints.xml";
        listJoint = TiXmlDocument(pathHumanJoint.str());

        if (!listJoint.LoadFile()) {
            ROS_WARN_ONCE("Error while loading xml file");
            ROS_WARN_ONCE("error # %d", listJoint.ErrorId());
            ROS_WARN_ONCE("%s", listJoint.ErrorDesc());
        }

        // Robots
        std::stringstream pathRobot;
        pathRobot << ros::package::getPath("toaster_visualizer") << "/src/list_robot.xml";
        listRobot = TiXmlDocument(pathRobot.str());

        if (!listRobot.LoadFile()) {
            ROS_WARN_ONCE("Error while loading xml file");
            ROS_WARN_ONCE("error # %d", listRobot.ErrorId());
            ROS_WARN_ONCE("%s", listRobot.ErrorDesc());
        }
    }

    bool switchNamePrint(toaster_msgs::Empty::Request &req, toaster_msgs::Empty::Response &res) {
        ROS_INFO("[toaster_visu] switching name print");
        printNames_ = !printNames_;

        return true;
    }

    bool scaleName(toaster_msgs::Scale::Request &req, toaster_msgs::Scale::Response &res) {
        ROS_INFO("[toaster_visu] scaling printed names");

        objectNameScale_ = req.objectScale;
        humanNameScale_ = req.humanScale;
        robotNameScale_ = req.robotScale;
        areaNameScale_ = req.areaScale;

        return true;
    }



    // **************************************** definition function of rviz markers ********************************************************

    /**
     * create a circle marker
     * @param p  		point from geometry library locating the center of the circle
     * @param rayon  	radius of the circle
     * @param name 		marker's name
     * @return marker 	circle marker with input property
     */
     visualization_msgs::Marker defineCircle(geometry_msgs::Point p, double rayon, double height, std::string name) {
        //declaration
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
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.1;

        //dimemsion
        marker.scale.x = rayon * 2;
        marker.scale.y = rayon * 2;
        marker.scale.z = height;

        //type
        marker.type = 3;
        marker.lifetime = ros::Duration();

        return marker;
    }


    /**
     * create a polygon marker based on line marker
     * @param poly 		polygon from geometry library
     * @param scale 		thickness of the line
     * @param name 		polygon's name
     * @return marker 	line marker representing input polygon
     */

    visualization_msgs::MarkerArray definePolygon(geometry_msgs::Polygon poly, std::string name, double zmin, double zmax){
		//declaration
		visualization_msgs::Marker line_strip1, line_strip2, line_list;

    //frame id
		line_strip1.header.frame_id = line_strip2.header.frame_id = line_list.header.frame_id = "map";

    //namespace
    std::ostringstream nameSpace;
    nameSpace << name;

    line_strip1.ns = line_strip2.ns = line_list.ns = nameSpace.str();
    line_strip1.id = 1;
    line_strip2.id = 2;
    line_list.id = 3;

    //action
		line_strip1.action = line_strip2.action = line_list.action = visualization_msgs::Marker::ADD;

    //orientation
		line_strip1.pose.orientation.w = line_strip2.pose.orientation.w = line_list.pose.orientation.w = 1.0;

		// marker type
    line_strip1.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip2.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

		line_strip1.scale.x = 0.2;
		line_strip2.scale.x = 0.2;
		line_list.scale.x = 0.2;

		// assigning colour
		line_strip1.color.b = 1.0;
		line_strip1.color.a = 1.0;
		line_strip2.color.b = 1.0;
		line_strip2.color.a = 1.0;
		line_list.color.b = 1.0;
		line_list.color.a = 1.0;
		geometry_msgs::Point p2;
		geometry_msgs::Point p1;

		for (int i = 0; i < poly.points.size(); i++) {

			p1.x = poly.points[i].x;
			p1.y = poly.points[i].y;
			p1.z = zmin;
			p2.x = poly.points[i].x;
			p2.y = poly.points[i].y;
			p2.z = zmax;
			line_list.points.push_back(p1);
			line_list.points.push_back(p2);
			line_strip1.points.push_back(p1);
			line_strip2.points.push_back(p2);
		}

        p1.x = poly.points[0].x;
        p1.y = poly.points[0].y;
        p1.z = zmin;

        p2.x = poly.points[0].x;
        p2.y = poly.points[0].y;
        p2.z = zmax;

        line_strip1.points.push_back(p1);
        line_strip2.points.push_back(p2);
       // line_strip1 gives bottom polygonal face while line_strip2 gives upper polygonal face
       // line_list gives vertical edges

        line_strip1.lifetime = ros::Duration();
        line_strip2.lifetime = ros::Duration();
        line_list.lifetime = ros::Duration();

        visualization_msgs::MarkerArray markersarray;

        // mymarkers.markers[0] = points ;
        markersarray.markers.push_back(line_strip1);
        markersarray.markers.push_back(line_strip2);
        markersarray.markers.push_back(line_list);

        return markersarray;
}

  bool isActivated(std::string id)
  {
    bool active = false;
    for (unsigned int i = 0; i < factList.size(); i++)
    {
      if(factList[i].subjectId == id)
      {
        if(factList[i].stringValue == "active")
          active = true;
      }
    }
    return active;
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
    visualization_msgs::Marker defineObj(geometry_msgs::Pose pose, std::string name, std::string id, double scale = 1) {
        //declaration
        double roll, pitch, yaw;
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

        //pose
        marker.pose = pose;

        //orientation
        //marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw + 3.141596 / 2);

        //color
        if(isActivated(id))
        {
          marker.color.r = 0.75;
          marker.color.g = 0.5;
          marker.color.b = 0.25;
          marker.color.a = 1.0;
        }
        else
        {
          marker.color.r = 0.25;
          marker.color.g = 0.5;
          marker.color.b = 0.75;
          marker.color.a = 1.0;
        }

        //scale
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        //type
        marker.type = visualization_msgs::Marker::CUBE; //marker by default

        TiXmlHandle hdl(&listObj);
        TiXmlElement *elem = hdl.FirstChildElement().FirstChildElement().Element();

        std::string name_obj;
        std::string mesh_r;

        while (elem) //for each element of the xml file
        {
            name_obj = elem->Attribute("name");
            mesh_r = elem->Attribute("mesh_resource");
            elem = elem->NextSiblingElement();

            if (name_obj.compare(name) == 0) //if there is a 3d model relativ to this object
            {
                marker.scale.x = scale;
                marker.scale.y = scale;
                marker.scale.z = scale;

                if(isActivated(id))
                {
                  marker.color.r = 0.75;
                  marker.color.g = 0.5;
                  marker.color.b = 0.25;
                  marker.color.a = 0.5;
                }
                else
                {
                  marker.color.r = 0.0;
                  marker.color.g = 0.0;
                  marker.color.b = 0.0;
                  marker.color.a = 0.0;
                }

                marker.type = visualization_msgs::Marker::MESH_RESOURCE; //use it as mesh
                marker.mesh_resource = mesh_r;
                marker.mesh_use_embedded_materials = true;

                elem = NULL;
            }
        }

        marker.lifetime = ros::Duration(1.0);

        return marker;
    }

    /**
     * create a name marker corresponding to the input marker
     * @param marker	marker to which we want to create a name marker
     * @return marker 	name marker
     */
    visualization_msgs::Marker defineName(visualization_msgs::Marker marker) {
        //declaration
        std::stringstream ss;
        visualization_msgs::Marker nameMarker = marker;

        //position
        nameMarker.pose.position.z = nameMarker.pose.position.z + marker.scale.x * 1.25; //put the name marker above target marker

        //color
        nameMarker.color.r = marker.color.r * 0.7; //add some contrast between name marker and input marker
        nameMarker.color.g = marker.color.g * 0.7;
        nameMarker.color.b = marker.color.b * 0.7;
        nameMarker.color.a = 1.0;


        //id
        nameMarker.id = -nameMarker.id; //opposite id to avoid id conflicts

        //type
        nameMarker.type = 9;

        //text field
        nameMarker.text = marker.ns;

        ss << nameMarker.text << "_name";
        nameMarker.ns = ss.str();

        //scale
        nameMarker.scale.z = 0.5 * marker.scale.x;

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
    visualization_msgs::Marker defineHuman(geometry_msgs::Pose pose, double scale, std::string name) {

        //declaration
        double roll, pitch, yaw;
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

        //position
        /*tf::Quaternion q;
        tf::quaternionMsgToTF(marker.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        yaw += 3.141596 / 2;

        pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
         */
        marker.pose = pose;

        //color
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.0;

        //scale
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;

        //type of marker
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;

        TiXmlHandle hdl(&listHuman);
        TiXmlElement *elem = hdl.FirstChildElement().FirstChildElement().Element();

        std::string name_obj;
        std::string mesh_r;

        while (elem) //for each element of the xml file
        {
            name_obj = elem->Attribute("name");
            mesh_r = elem->Attribute("mesh_resource");
            elem = elem->NextSiblingElement();

            if (name_obj.compare(name) == 0) //if there is a 3d model relative to this human
            {
                marker.mesh_resource = mesh_r;
                marker.mesh_use_embedded_materials = true;

                elem = NULL;
            } else {
                if (pose.position.z < -0.4) {
                    //human is seating
                    marker.mesh_resource = "package://toaster_visualizer/mesh/toaster_humans/humanSeat.dae"; //using 3d human model
                } else {
                    marker.mesh_resource = "package://toaster_visualizer/mesh/toaster_humans/human.dae"; //using 3d human model
                }
                marker.mesh_use_embedded_materials = true;
            }
        }

        marker.lifetime = ros::Duration(1.0);

        return marker;
    }

    /**
     * create a robot marker
     * @param x  			coordinates of robot's base in the x direction
     * @param y			coordinates of robot's base in the y direction
     * @param z 			coordinates of robot's base in the z direction
     * @param scale 		dimension of the marker
     * @param name 		marker's name
     * @return marker 	mesh marker of robot
     */
    visualization_msgs::Marker defineRobot(geometry_msgs::Pose pose, double scale, std::string name) {

        //declaration
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

        //pose
        marker.pose = pose;

        //orientation
        //marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

        //color
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.0;

        //scale
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;

        //type of marker
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;

        TiXmlHandle hdl(&listRobot);
        TiXmlElement *elem = hdl.FirstChildElement().FirstChildElement().Element();

        std::string name_obj;
        std::string mesh_r;

        while (elem) //for each element of the xml file
        {
            name_obj = elem->Attribute("name");
            mesh_r = elem->Attribute("mesh_resource");
            elem = elem->NextSiblingElement();

            if (name_obj.compare(name) == 0) //if there is a 3d model relative to this robot
            {
                marker.mesh_resource = mesh_r;
                marker.mesh_use_embedded_materials = true;

                elem = NULL;
            } else {
                marker.mesh_resource = "package://toaster_visualizer/mesh/toaster_robots/pr2.dae"; //using default 3d robot model
                marker.mesh_use_embedded_materials = true;
            }
        }
        marker.lifetime = marker.lifetime = ros::Duration(1.0);

        return marker;
    }

    visualization_msgs::Marker defineArrow(visualization_msgs::Marker& sub, visualization_msgs::Marker& targ, double confidence, bool distance) {

        //declaration
        visualization_msgs::Marker marker;

        //frame id
        marker.header.frame_id = "map";

        //namespace
        std::ostringstream nameSpace;
        std::string subtype = "distance";
        if (!distance)
            subtype = "direction";
        nameSpace << sub.ns << " MvTwd " << subtype << targ.ns;
        marker.ns = nameSpace.str();
        marker.id = id_generator(nameSpace.str()); //creation of an unique id based on marker's name

        //action
        marker.action = visualization_msgs::Marker::ADD;

        //position
        geometry_msgs::Point point0;
        geometry_msgs::Point point1;

        point0.x = sub.pose.position.x;
        point0.y = sub.pose.position.y;
        point0.z = sub.pose.position.z;

        point1.x = targ.pose.position.x;
        point1.y = targ.pose.position.y;
        point1.z = targ.pose.position.z;

        if (distance) {
            point0.x += 0.1;
            point0.y += 0.1;
            point1.y += 0.1;
            point1.x += 0.1;
        }


        marker.points.push_back(point0);
        marker.points.push_back(point1);

        //color
        if (distance)
            marker.color.r = confidence;
        else
            marker.color.g = confidence;
        marker.color.a = 1.0;

        //scale
        marker.scale.x = 0.05;
        marker.scale.y = 0.2;
        marker.scale.z = 0.3;

        //type of marker
        marker.type = visualization_msgs::Marker::ARROW;
        marker.mesh_use_embedded_materials = true;

        marker.lifetime = ros::Duration(0.01);

        return marker;
    }


    // ******************************************************** id generator for markers  ********************************************************

    /**
     * In our context names are identifier so we need to create an unique identifier for each input name
     * @param name		name of target
     * @return id		new identifier or target's identifier if his name already have been assigned to an identifier
     */
    int id_generator(std::string name) {
        if (std::find(name_list.begin(), name_list.end(), name) != name_list.end()) {
            return std::find(name_list.begin(), name_list.end(), name) - name_list.begin() + 1;
        } else {
            name_list.push_back(name);
            return id_cpt++;
        }
    }

    bool isSeating(const toaster_msgs::Human hum) {
        //Find the head:
        unsigned int i = 0;
        for (i = 0; i < hum.meAgent.skeletonNames.size(); i++) {
            if (hum.meAgent.skeletonNames[i] == "head") {
                break;
            }
        }
        if (i == hum.meAgent.skeletonNames.size()) {
            return false;
        } else {
            if (hum.meAgent.skeletonJoint[i].meEntity.pose.position.z < 1.4)
                return true;
            else
                return false;
        }
    }


    // ******************************************************** adjustment functions ********************************************************

    /**
     * Function to modify a marker's color
     * @param marker		name of target marker
     * @param r 			value of red coefficient
     * @param g 			value of green coefficient
     * @param b 			value of blue coefficient
     * @return marker 		new marker with input modifications
     */
    visualization_msgs::Marker setColor(visualization_msgs::Marker marker, double r, double g, double b) {
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;

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
    visualization_msgs::Marker setPosition(visualization_msgs::Marker marker, float x, float y, float z) {
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
    visualization_msgs::Marker setSize(visualization_msgs::Marker marker, float x, float y, float z) {
        marker.scale.x = x;
        marker.scale.y = y;
        marker.scale.z = z;

        return marker;
    }

    /**
     * Function to give and register a random color to input marker
     * @param marker		name of target marker
     * @return marker 		new marker with input modifications
     */
    visualization_msgs::Marker setRandomColor(visualization_msgs::Marker marker) {
        if (color_map.find(marker.ns) == color_map.end()) {
            std::vector<float> color_list;
            color_list.push_back(static_cast<float> (rand()) / static_cast<float> (RAND_MAX));
            color_list.push_back(static_cast<float> (rand()) / static_cast<float> (RAND_MAX));
            color_list.push_back(static_cast<float> (rand()) / static_cast<float> (RAND_MAX));
            color_map[marker.ns] = color_list;
        }

        marker.color.r = (color_map[marker.ns])[0];
        marker.color.g = (color_map[marker.ns])[1];
        marker.color.b = (color_map[marker.ns])[2];
        return marker;
    }

    //******************************************************** emission/reception ********************************************************

    //reception

    /**
     * CallBack creating markers based on received toaster_msgs and adding then to obj_list
     * @param msg			reference to receive toaster_msgs::ObjectList
     * @return 			void
     */
    void chatterCallbackObjList(const toaster_msgs::ObjectListStamped::ConstPtr& msg) //toaster object list reception
    {
        obj_list.markers.clear();

        for (int i = 0; i < msg->objectList.size(); i++) {
            visualization_msgs::Marker m = defineObj(msg->objectList[i].meEntity.pose,
                                                      msg->objectList[i].meEntity.name,
                                                      msg->objectList[i].meEntity.id);

            if (printNames_) {
                visualization_msgs::Marker mn = defineName(m);
                mn = setColor(mn, 0.0, 0.0, 1.0);
                mn = setSize(mn, 0, 0, objectNameScale_);

                obj_list.markers.push_back(mn);
            }
            obj_list.markers.push_back(m);

            ROS_DEBUG("obj %d", m.id);
        }
        // extra object for environment
        geometry_msgs::Pose p;
        p.position.x = 0.3;
        p.position.y = 0.3;
        p.position.z = -0.05;
        p.orientation.w = 1.0;
        visualization_msgs::Marker m = defineObj(p, "env", "");
        obj_list.markers.push_back(m);
    }

    /**
     * CallBack creating markers based on received toaster_msgs and adding then to area_list
     * There is two different types of area markers so this function manages both circle and polygon types
     * @param msg			reference to receive toaster_msgs::AreaList
     * @return 			void
     */
    void chatterCallbackAreaList(const toaster_msgs::AreaList::ConstPtr& msg) {
        area_list.markers.clear();

        for (int i = 0; i < msg->areaList.size(); i++) {

            if (msg->areaList[i].isCircle == true) //circle case
            {
                visualization_msgs::Marker m = defineCircle(msg->areaList[i].center,
                        msg->areaList[i].ray, msg->areaList[i].height, msg->areaList[i].name);

                m = setRandomColor(m);

                visualization_msgs::Marker mn = defineName(m);
                mn = setSize(mn, 0.0, 0.0, 0.3);
                mn = setPosition(mn, mn.pose.position.x, mn.pose.position.y, mn.pose.position.z + 0.5);

                area_list.markers.push_back(m);
                area_list.markers.push_back(mn);

                ROS_DEBUG("circle %d", m.id);
            } else // polygon case
            {
                visualization_msgs::MarkerArray m = definePolygon(msg->areaList[i].poly, msg->areaList[i].name, msg->areaList[i].zmin, msg->areaList[i].zmax);

               for(int i =0 ; i<3 ; i++)
                m.markers[i] = setRandomColor(m.markers[i]);

                visualization_msgs::MarkerArray mn ;
                for(int i =0 ; i<3 ; i++)
                { mn.markers.push_back(defineName(m.markers[i]));
                mn.markers.push_back(setSize(mn.markers[i], 0.0, 0.0, 0.1));}

                double posx = 0.0;
                double posy = 0.0;
                double posz = 0.0;
				for (int j = 0; j<3; j++)
               { for (int i = 0; i < m.markers[j].points.size(); i++) {
                    posx = posx + m.markers[j].points[i].x;
                    posy = posy + m.markers[j].points[i].y;
                    posz = posz + msg->areaList[i].zmin;
                }

                mn.markers[j] = setPosition(mn.markers[j], posx / m.markers[j].points.size(), posy / m.markers[j].points.size(),  + posz / m.markers[j].points.size());

                area_list.markers.push_back(m.markers[j]);
                area_list.markers.push_back(mn.markers[j]);
              }
               // ROS_DEBUG("poly %d", m.id);
            }

        }

    }
    /**
     * CallBack creating markers based on received toaster_msgs and adding then to robot_list
     * Robots can be represented by a single unarticulated mesh or by multiple meshs for an articulated model
     * @param msg			reference to receive toaster_msgs::RobotList
     * @return 			void
     */
    void chatterCallbackRobotList(const toaster_msgs::RobotListStamped::ConstPtr& msg) {
        robot_list.markers.clear();

        for (int i = 0; i < msg->robotList.size(); i++) {
            //non articulated robot
            visualization_msgs::Marker m = defineRobot(msg->robotList[i].meAgent.meEntity.pose,
                    1.0, msg->robotList[i].meAgent.meEntity.name);

            if (printNames_) {
                visualization_msgs::Marker mn = defineName(m);
                mn = setPosition(mn, mn.pose.position.x, mn.pose.position.y, mn.pose.position.z + 1);
                mn = setSize(mn, 0, 0, robotNameScale_);

                //If the robot is moving, we intensify its name color
                std::map<std::string, double>::const_iterator it = agentMoving_map.find(msg->robotList[i].meAgent.meEntity.id);
                if (it != agentMoving_map.end())
                    mn = setColor(mn, 0.4 + it->second * 0.6, 0.0, 0.0);
                else
                    mn = setColor(mn, 0.2, 0.0, 0.0);


                robot_list.markers.push_back(mn);
            }

            robot_list.markers.push_back(m);

            ROS_DEBUG("robot %d", m.id);
        }
    }

    /**
     * CallBack creating markers based on received toaster_msgs and adding then to human_list
     * Humans can be represented by a single unarticulated mesh or by multiple meshs for an articulated model
     * @param msg			reference to receive toaster_msgs::HumanList
     * @return 			void
     */
    void chatterCallbackHumanList(const toaster_msgs::HumanListStamped::ConstPtr& msg) {
        human_list.markers.clear();

        for (int i = 0; i < msg->humanList.size(); i++) {
            //non articulated human
            visualization_msgs::Marker m = defineHuman(msg->humanList[i].meAgent.meEntity.pose,
                    1.0, msg->humanList[i].meAgent.meEntity.name);

            if (printNames_) {
                visualization_msgs::Marker mn = defineName(m);
                mn = setPosition(mn, mn.pose.position.x, mn.pose.position.y, mn.pose.position.z + 1);
                mn = setSize(mn, 0, 0, humanNameScale_);

                //If the human is moving, we intensify its color
                std::map<std::string, double>::const_iterator it = agentMoving_map.find(msg->humanList[i].meAgent.meEntity.id);
                if (it != agentMoving_map.end())
                    mn = setColor(mn, 0.0, 0.4 + it->second * 0.6, 0.0);
                else
                    mn = setColor(mn, 0.0, 0.2, 0.0);

                human_list.markers.push_back(mn);
            }

            if (msg->humanList[i].meAgent.skeletonJoint.size() == 0) {
                human_list.markers.push_back(m);

                ROS_DEBUG("human %d", m.id);
            } else //articulated human
            {
                std::vector<toaster_msgs::Joint> joints = msg->humanList[i].meAgent.skeletonJoint;
                visualization_msgs::Marker markerTempo;

                int scale = 1;

                for (int y = 0; y < joints.size(); y++) {
                    ROS_DEBUG("joint");

                    std::string name = msg->humanList[i].meAgent.skeletonJoint[y].meEntity.name;

                    //frame id
                    markerTempo.header.frame_id = "map";

                    //namespace
                    markerTempo.ns = name;
                    markerTempo.id = id_generator(joints[y].meEntity.name); //creation of an unique id based on marker's name

                    //action
                    markerTempo.action = visualization_msgs::Marker::ADD;

                    // Pose
                    markerTempo.pose = joints[y].meEntity.pose;

                    //color
                    markerTempo.color.r = 1.0;
                    markerTempo.color.g = 1.0;
                    markerTempo.color.b = 1.0;
                    markerTempo.color.a = 1.0;

                    //scale
                    markerTempo.scale.x = 0.1 * scale;
                    markerTempo.scale.y = 0.1 * scale;
                    markerTempo.scale.z = 0.1 * scale;

                    //type of marker
                    markerTempo.type = 3;

                    TiXmlHandle hdl(&listJoint);
                    TiXmlElement *elem = hdl.FirstChildElement().FirstChildElement().Element();

                    std::string name_obj;
                    std::string mesh_r;

                    while (elem) //for each element of the xml file
                    {
                        name_obj = elem->Attribute("name");
                        mesh_r = elem->Attribute("mesh_resource");
                        elem = elem->NextSiblingElement();

                        if (name_obj.compare(name) == 0) //if there is a 3d model related to this object
                        {


                            markerTempo.scale.x = scale;
                            markerTempo.scale.y = scale;
                            markerTempo.scale.z = scale;


                            markerTempo.color.r = 0.0;
                            markerTempo.color.g = 0.0;
                            markerTempo.color.b = 0.0;
                            markerTempo.color.a = 0.0;

                            markerTempo.type = visualization_msgs::Marker::MESH_RESOURCE; //use it as mesh

                            if (isSeating(msg->humanList[i]) && name == "base") {
                                markerTempo.mesh_resource = "package://toaster_visualizer/mesh/toaster_humans/mocapMorse/baseSeat.dae"; //using 3d human model
                            } else {
                                markerTempo.mesh_resource = mesh_r;
                            }
                            markerTempo.mesh_use_embedded_materials = true;

                            elem = NULL;
                        }
                    }

                    markerTempo.lifetime = ros::Duration(1.0);

                    human_list.markers.push_back(markerTempo);
                }
            }
        }
    }

    void chatterCallbackFactList(const toaster_msgs::FactList::ConstPtr& msg)
    {
      factList.clear();
      for(unsigned int i = 0; i < msg->factList.size(); i++)
      {
        factList.push_back(msg->factList[i]);
      }
    }

    void chatterCallbackAgentFactList(const toaster_msgs::FactList::ConstPtr& msg) {
        agentMoving_map.clear();
        arrow_list.markers.clear();

        for (int iFact = 0; iFact < msg->factList.size(); iFact++) {
            if (msg->factList[iFact].property == "IsMoving")
                agentMoving_map[msg->factList[iFact].subjectId] = msg->factList[iFact].confidence;
            else if (msg->factList[iFact].property == "IsMovingToward") {

                bool foundSub = false;
                bool foundTarg = false;
                visualization_msgs::Marker sub;
                visualization_msgs::Marker targ;
                visualization_msgs::Marker arrow;

                //Let's look for the subject position:
                for (unsigned int i = 0; i < human_list.markers.size(); i++) {
                    if (human_list.markers[i].ns.compare(msg->factList[iFact].subjectId) == 0) {
                        sub = human_list.markers[i];
                        foundSub = true;
                        break;
                    }
                }

                if (foundSub) {
                    for (unsigned int i = 0; i < human_list.markers.size(); i++) {

                        if (human_list.markers[i].ns.compare(msg->factList[iFact].targetId) == 0) {

                            targ = human_list.markers[i];
                            foundTarg = true;
                            break;
                        }
                    }

                    if (!foundTarg) {
                        for (unsigned int i = 0; i < robot_list.markers.size(); i++) {
                            if (robot_list.markers[i].ns.compare(msg->factList[iFact].targetId) == 0) {
                                targ = robot_list.markers[i];
                                foundTarg = true;
                                break;
                            }
                        }
                        if (!foundTarg) {
                            for (unsigned int i = 0; i < obj_list.markers.size(); i++) {
                                if (obj_list.markers[i].ns.compare(msg->factList[iFact].targetId) == 0) {
                                    targ = obj_list.markers[i];
                                    foundTarg = true;
                                    break;
                                }
                            }
                            if (!foundTarg) {
                                continue;
                            }
                        }
                    }


                } else {
                    for (unsigned int i = 0; i < robot_list.markers.size(); i++) {
                        if (robot_list.markers[i].ns.compare(msg->factList[iFact].subjectId) == 0) {
                            sub = human_list.markers[i];
                            foundSub = true;
                            break;
                        }
                    }

                    if (foundSub) {
                        for (unsigned int i = 0; i < human_list.markers.size(); i++) {
                            if (human_list.markers[i].ns.compare(msg->factList[iFact].targetId) == 0) {
                                targ = human_list.markers[i];
                                foundTarg = true;
                                break;
                            }
                        }
                        if (!foundTarg) {
                            for (unsigned int i = 0; i < robot_list.markers.size(); i++) {
                                if (robot_list.markers[i].ns.compare(msg->factList[iFact].targetId) == 0) {
                                    targ = robot_list.markers[i];
                                    foundTarg = true;
                                    break;
                                }
                            }
                            if (!foundTarg) {
                                for (unsigned int i = 0; i < obj_list.markers.size(); i++) {
                                    if (obj_list.markers[i].ns.compare(msg->factList[iFact].targetId) == 0) {
                                        targ = human_list.markers[i];
                                        foundTarg = true;
                                        break;
                                    }
                                }
                                if (!foundTarg) {
                                    continue;
                                }
                            }
                        }
                    }
                }

                // Create arrow
                if (foundTarg && foundSub) {
                    arrow = defineArrow(sub, targ, msg->factList[iFact].confidence, msg->factList[iFact].subProperty.compare("distance") == 0);
                    arrow_list.markers.push_back(arrow);
                }
            }
        }
    }

    /**
     * Function sending all marker list to rviz
     * @return 			void
     */
    void send() {
        pub_area.publish(area_list);
        pub_obj.publish(obj_list);
        pub_human.publish(human_list);
        pub_robot.publish(robot_list);
        pub_movingTwrd.publish(arrow_list);

        ros::spinOnce();
    }
};

/**
 * Main function using class run
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "Run");
    ros::NodeHandle node;
    ROS_INFO("[toaster-visu] launched");



    Run c = Run(node);


    ros::Rate loop_rate(30);

    ros::ServiceServer switch_name_print;
    switch_name_print = node.advertiseService("toaster_visualizer/switch_name_print", &Run::switchNamePrint, &c);


    ros::ServiceServer name_scale;
    name_scale = node.advertiseService("toaster_visualizer/scale_name", &Run::scaleName, &c);


    while (ros::ok()) {
        c.send();
        loop_rate.sleep();
    }

    return 0;
}
