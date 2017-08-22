#include <string>
#include <vector>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "geometry_msgs/Point.h"
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Quaternion.h>

#include <tinyxml.h>

#ifndef MARKERCREATOR_H
#define MARKERCREATOR_H

class MarkerCreator
{
public:
  /**
   * create a circle marker
   * @param p  		point from geometry library locating the center of the circle
   * @param rayon  	radius of the circle
   * @param name 		marker's name
   * @return marker 	circle marker with input property
   */
  static visualization_msgs::Marker defineCircle(geometry_msgs::Point p, double rayon, double height, std::string name, int id);

  /**
   * create a polygon marker based on line marker
   * @param poly 		polygon from geometry library
   * @param scale 		thickness of the line
   * @param name 		polygon's name
   * @return marker 	line marker representing input polygon
   */
  static visualization_msgs::MarkerArray definePolygon(geometry_msgs::Polygon poly, std::string name, double zmin, double zmax);

  /**
   * create a name marker corresponding to the input marker
   * @param marker	marker to which we want to create a name marker
   * @return marker 	name marker
   */
  static visualization_msgs::Marker defineName(visualization_msgs::Marker marker);

  /**
   * create an object marker
   * @param x  		coordinates of object's base in thx x direction
   * @param y			coordinates of object's base in thx y direction
   * @param z 			coordinates of object's base in thx z direction
   * @param scale 		dimension of the marker
   * @param name 		marker's name
   * @return marker 	object marker or mesh marker if the object is in the mesh database
   */
  static visualization_msgs::Marker defineObj(geometry_msgs::Pose pose, std::string name, bool activated, int id, TiXmlDocument& listObj, double scale = 1);

  /**
   * create a human marker
   * @param x  			coordinates of human's base in the x direction
   * @param y			coordinates of human's base in the y direction
   * @param z 			coordinates of human's base in the z direction
   * @param scale 		dimension of the marker
   * @param name 		marker's name
   * @return marker 	mesh marker of human
   */
  static visualization_msgs::Marker defineHuman(geometry_msgs::Pose pose, double scale, std::string name, int id, TiXmlDocument& listHuman);

  /**
   * create a robot marker
   * @param x  			coordinates of robot's base in the x direction
   * @param y			coordinates of robot's base in the y direction
   * @param z 			coordinates of robot's base in the z direction
   * @param scale 		dimension of the marker
   * @param name 		marker's name
   * @return marker 	mesh marker of robot
   */
  static visualization_msgs::Marker defineRobot(geometry_msgs::Pose pose, double scale, std::string name, int id, TiXmlDocument& listRobot);

  static visualization_msgs::Marker defineArrow(visualization_msgs::Marker& sub, visualization_msgs::Marker& targ, double confidence, bool distance, int id);

  /////////////////////////////
  /*MARKERS UTILITY FUNCTIONS*/
  /////////////////////////////

  /**
   * Function to modify a marker's color
   * @param marker		name of target marker
   * @param r 			value of red coefficient
   * @param g 			value of green coefficient
   * @param b 			value of blue coefficient
   * @return marker 		new marker with input modifications
   */
  static visualization_msgs::Marker setColor(visualization_msgs::Marker marker, double r, double g, double b);
  /**
   * Function to modify a marker's position
   * @param marker		name of target marker
   * @param x 			coordinates of marker's base in the x direction
   * @param y 			coordinates of marker's base in the y direction
   * @param z 			coordinates of marker's base in the z direction
   * @return marker 		new marker with input modifications
   */
  static visualization_msgs::Marker setPosition(visualization_msgs::Marker marker, float x, float y, float z);

  /**
   * Function to modify a marker's size
   * @param marker		name of target marker
   * @param x 			size of the marker in the x direction
   * @param y 			size of the marker in the y direction
   * @param z 			size of the marker in the z direction
   * @return marker 		new marker with input modifications
   */
  static visualization_msgs::Marker setSize(visualization_msgs::Marker marker, float x, float y, float z);

  /**
   * Function to give and register a random color to input marker
   * @param marker		name of target marker
   * @return marker 		new marker with input modifications
   */
  static visualization_msgs::Marker setRandomColor(visualization_msgs::Marker marker);

private:
   static std::map<std::string, std::vector<float> > colorMap_;
};

#endif /*MARKERCREATOR_H*/
