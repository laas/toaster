#include <string>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "geometry_msgs/Point.h"
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Quaternion.h>

#include <tinyxml.h>

#ifndef MARKERCREATOR_H
#define MARKERCREATOR_H

namespace MarkerCreator
{

  /**
   * create a circle marker
   * @param p  		point from geometry library locating the center of the circle
   * @param rayon  	radius of the circle
   * @param name 		marker's name
   * @return marker 	circle marker with input property
   */
  visualization_msgs::Marker defineCircle(geometry_msgs::Point p, double rayon, double height, std::string name, int id);

  /**
   * create a polygon marker based on line marker
   * @param poly 		polygon from geometry library
   * @param scale 		thickness of the line
   * @param name 		polygon's name
   * @return marker 	line marker representing input polygon
   */
  visualization_msgs::MarkerArray definePolygon(geometry_msgs::Polygon poly, std::string name, double zmin, double zmax);

  /**
   * create a name marker corresponding to the input marker
   * @param marker	marker to which we want to create a name marker
   * @return marker 	name marker
   */
  visualization_msgs::Marker defineName(visualization_msgs::Marker marker);

  /**
   * create an object marker
   * @param x  		coordinates of object's base in thx x direction
   * @param y			coordinates of object's base in thx y direction
   * @param z 			coordinates of object's base in thx z direction
   * @param scale 		dimension of the marker
   * @param name 		marker's name
   * @return marker 	object marker or mesh marker if the object is in the mesh database
   */
  visualization_msgs::Marker defineObj(geometry_msgs::Pose pose, std::string name, bool activated, int id, TiXmlDocument& listObj, double scale = 1);

  /**
   * create a human marker
   * @param x  			coordinates of human's base in the x direction
   * @param y			coordinates of human's base in the y direction
   * @param z 			coordinates of human's base in the z direction
   * @param scale 		dimension of the marker
   * @param name 		marker's name
   * @return marker 	mesh marker of human
   */
  visualization_msgs::Marker defineHuman(geometry_msgs::Pose pose, double scale, std::string name, int id, TiXmlDocument& listHuman);

  /**
   * create a robot marker
   * @param x  			coordinates of robot's base in the x direction
   * @param y			coordinates of robot's base in the y direction
   * @param z 			coordinates of robot's base in the z direction
   * @param scale 		dimension of the marker
   * @param name 		marker's name
   * @return marker 	mesh marker of robot
   */
  visualization_msgs::Marker defineRobot(geometry_msgs::Pose pose, double scale, std::string name, int id, TiXmlDocument& listRobot);

  visualization_msgs::Marker defineArrow(visualization_msgs::Marker& sub, visualization_msgs::Marker& targ, double confidence, bool distance, int id);
}

#endif /*MARKERCREATOR_H*/
