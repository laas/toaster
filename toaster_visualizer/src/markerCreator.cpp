#include "markerCreator.h"

std::map<std::string, std::vector<float> > MarkerCreator::colorMap_;

visualization_msgs::Marker MarkerCreator::defineCircle(geometry_msgs::Point p, double rayon, double height, std::string name, int id) {
  //declaration
  visualization_msgs::Marker marker;

  //frame id
  marker.header.frame_id = "map";

  //namespace
  std::ostringstream nameSpace;
  nameSpace << name;
  marker.ns = nameSpace.str();
  marker.id = id;

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
  marker.color.a = 0.2;

  //dimemsion
  marker.scale.x = rayon * 2;
  marker.scale.y = rayon * 2;
  marker.scale.z = height;

  //type
  marker.type = 3;
  marker.lifetime = ros::Duration();

  return marker;
}

visualization_msgs::MarkerArray MarkerCreator::definePolygon(geometry_msgs::Polygon poly, std::string name, double zmin, double zmax){
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

visualization_msgs::Marker MarkerCreator::defineObj(geometry_msgs::Pose pose, std::string name, bool activated, int id, TiXmlDocument& listObj, double scale){
  //declaration
  double roll, pitch, yaw;
  visualization_msgs::Marker marker;

  //frame id
  marker.header.frame_id = "map";

  //namespace
  std::ostringstream nameSpace;
  nameSpace << name;
  marker.ns = nameSpace.str();
  marker.id = id; //creation of an unique id based on marker's name

  //action
  marker.action = visualization_msgs::Marker::ADD;

  //pose
  marker.pose = pose;

  //orientation
  //marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw + 3.141596 / 2);

  //color
  if(activated)
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

      marker.type = visualization_msgs::Marker::MESH_RESOURCE; //use it as mesh
      marker.mesh_resource = mesh_r;
      marker.mesh_use_embedded_materials = true;

      if(activated)
      {
        marker.color.r = 0.75;
        marker.color.g = 0.5;
        marker.color.b = 0.25;
        marker.color.a = 0.9;
      }
      else
      {
        marker.color.r = 0.25;
        marker.color.g = 0.5;
        marker.color.b = 0.75;
        marker.color.a = 0.1;
      }

      elem = NULL;
    }
  }

  marker.lifetime = ros::Duration(1.0);

  return marker;
}

visualization_msgs::Marker MarkerCreator::defineName(visualization_msgs::Marker marker) {
  //declaration
  std::stringstream ss;
  visualization_msgs::Marker nameMarker = marker;

  //position
  nameMarker.pose.position.z = nameMarker.pose.position.z + marker.scale.x * 1.25; //put the name marker above target marker

  //color
  nameMarker.color.r = marker.color.r * 0.9; //add some contrast between name marker and input marker
  nameMarker.color.g = marker.color.g * 0.9;
  nameMarker.color.b = marker.color.b * 0.9;
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

visualization_msgs::Marker MarkerCreator::defineHuman(geometry_msgs::Pose pose, double scale, std::string name, int id, TiXmlDocument& listHuman) {

    //declaration
    double roll, pitch, yaw;
    visualization_msgs::Marker marker;

    //frame id
    marker.header.frame_id = "map";

    //namespace
    std::ostringstream nameSpace;
    nameSpace << name;
    marker.ns = nameSpace.str();
    marker.id = id; //creation of an unique id based on marker's name

    //action
    marker.action = visualization_msgs::Marker::ADD;

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

visualization_msgs::Marker MarkerCreator::defineRobot(geometry_msgs::Pose pose, double scale, std::string name, int id, TiXmlDocument& listRobot) {

    //declaration
    visualization_msgs::Marker marker;

    //frame id
    marker.header.frame_id = "map";

    //namespace
    std::ostringstream nameSpace;
    nameSpace << name;
    marker.ns = nameSpace.str();
    marker.id = id; //creation of an unique id based on marker's name

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

visualization_msgs::Marker MarkerCreator::defineArrow(visualization_msgs::Marker& sub, visualization_msgs::Marker& targ, double confidence, bool distance, int id) {

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
    marker.id = id; //id_generator(nameSpace.str()); //creation of an unique id based on marker's name

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
  /////////////////////////////
  /*MARKERS UTILITY FUNCTIONS*/
  /////////////////////////////

visualization_msgs::Marker MarkerCreator::setColor(visualization_msgs::Marker marker, double r, double g, double b) {
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;

    return marker;
}

visualization_msgs::Marker MarkerCreator::setPosition(visualization_msgs::Marker marker, float x, float y, float z) {
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;

    return marker;
}

visualization_msgs::Marker MarkerCreator::setSize(visualization_msgs::Marker marker, float x, float y, float z) {
    marker.scale.x = x;
    marker.scale.y = y;
    marker.scale.z = z;

    return marker;
}

visualization_msgs::Marker MarkerCreator::setRandomColor(visualization_msgs::Marker marker) {
    if (colorMap_.find(marker.ns) == colorMap_.end()) {
        std::vector<float> color_list;
        color_list.push_back(static_cast<float> (rand()) / static_cast<float> (RAND_MAX));
        color_list.push_back(static_cast<float> (rand()) / static_cast<float> (RAND_MAX));
        color_list.push_back(static_cast<float> (rand()) / static_cast<float> (RAND_MAX));
        colorMap_[marker.ns] = color_list;
    }

    marker.color.r = (colorMap_[marker.ns])[0];
    marker.color.g = (colorMap_[marker.ns])[1];
    marker.color.b = (colorMap_[marker.ns])[2];
    return marker;
}
