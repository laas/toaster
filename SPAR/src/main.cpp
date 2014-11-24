//This file will compute the spatial facts concerning agents present in the interaction.

#include "SPAR/PDGHumanReader.h"
#include "SPAR/PDGRobotReader.h"
#include "toaster-lib/CircleArea.h"
#include <iterator>

bg::model::point <double, 2, bg::cs::cartesian> convert3dTo2d(bg::model::point<double, 3, bg::cs::cartesian> position){
  bg::model::point <double, 2, bg::cs::cartesian> point2d;
  point2d.set<0>(position.get<0>());
  point2d.set<1>(position.get<1>());

  return point2d;
}

// Entity should be a vector or a map with all entities
// This function update all the area that depends on an entity position.
void updateEntityArea(std::map<int, Area*>& mpArea, Entity* entity){
  for(std::map<int, Area*>::iterator it = mpArea.begin() ; it != mpArea.end() ; ++it){
    if( it->second->getMyOwner() == entity->getId() )
        // Dangerous casting? Only CircleArea has Owner...
        ((CircleArea*) it->second)->setCenter( convert3dTo2d( entity->getPosition() ) );
  }
}

int main(int argc, char** argv){
  const bool AGENT_FULL_CONFIG = false;  //If false we will use only position and orientation

  ros::init(argc, argv, "SPAR");

  ros::NodeHandle node;

  //Data reading
  PDGHumanReader humanRd(node, AGENT_FULL_CONFIG);
  PDGRobotReader robotRd(node, AGENT_FULL_CONFIG);

  
  ros::Rate loop_rate(30);
  
  // Vector of Area
  // It should be possible to add an area on the fly with a ros service.
  std::map<int, Area*> mapArea;

  // Origin point
  bg::model::point<double, 2, bg::cs::cartesian> origin(0.0, 0.0);
  
  // AreaRays:
  double interDist = 4.0;
  double dangerDist = 1.5;

  // TODO: put this in a service with parameters
  // We create 2 Area for the robot:
  // Interacting
  CircleArea* interacting = new CircleArea(0,  origin, interDist);
  interacting->setMyOwner(1);
  // Danger
  CircleArea* danger = new CircleArea(1,  origin, dangerDist);
  danger->setMyOwner(1);
  
  mapArea[0] = interacting;
  mapArea[1] = danger;
  
  /************************/
  /* Start of the Ros loop*/
  /************************/
  
  while( node.ok() ){
    if( (humanRd.m_LastConfig[101] != NULL) && (robotRd.m_LastConfig[1] != NULL) ){
        // We update area with robot center
        updateEntityArea(mapArea, robotRd.m_LastConfig[1]);
        
      //printf("Last time human 101: %lu\n", humanRd.m_LastConfig[101]->getTime());

        //if(human is in an area){
                //do stuff
    }    
    ros::spinOnce();

    loop_rate.sleep();

  }
  return 0;
}
