//This file will compute the spatial facts concerning agents present in the interaction.

#include "SPAR/PDGHumanReader.h"
#include "SPAR/PDGRobotReader.h"
#include "toaster-lib/CircleArea.h"
#include <iterator>

// Look into boost geometry if there is a better way to do this
bg::model::point <double, 2, bg::cs::cartesian> convert3dTo2d(bg::model::point<double, 3, bg::cs::cartesian> position){
  bg::model::point <double, 2, bg::cs::cartesian> point2d;
  point2d.set<0>(position.get<0>());
  point2d.set<1>(position.get<1>());

  return point2d;
}

// Entity should be a vector or a map with all entities
// This function update all the area that depends on an entity position.
void updateEntityArea(std::map<unsigned int, Area*>& mpArea, Entity* entity){
  for(std::map<unsigned int, Area*>::iterator it = mpArea.begin() ; it != mpArea.end() ; ++it){
    if( it->second->getMyOwner() == entity->getId() )
        // Dangerous casting? Only CircleArea has Owner...
        ((CircleArea*) it->second)->setCenter( convert3dTo2d( entity->getPosition() ) );
  }
}



void updateInArea(Entity* ent, std::map<unsigned int, Area*>& mpArea){
  for(std::map<unsigned int, Area*>::iterator it = mpArea.begin() ; it != mpArea.end() ; ++it){
      // If we already know that entity is in Area, we update if needed.
      if( ent->isInArea( it->second->getId() ) )
              if( it->second->isPointInArea( convert3dTo2d( ent->getPosition() ) ) )
                continue;
              else{
                  ent->removeInArea(it->second->getId());
                  it->second->removeEntity( ent->getId() );
              }
      // Same if entity is not in Area
      else
    if( it->second->isPointInArea( convert3dTo2d( ent->getPosition() ) ) ){
        ent->inArea.push_back(it->second->getId());
        it->second->insideEntities.push_back( ent->getId() );
    }else
        continue;
  }
}

bool isFacing(Entity* entFacing, Entity* entSubject, double angleThreshold){
  
	double entFacingAngle = entFacing->getOrientation()[2];
	double entFacingSubjectAngle = acos( ( fabs( entFacing->getPosition().get<0>() - entSubject->getPosition().get<0>() ) ) 
                   /  bg::distance( convert3dTo2d(entFacing->getPosition()), convert3dTo2d(entSubject->getPosition()) ) );
   
        // Trigonometric adjustment
	if ( entSubject->getPosition().get<0>() < entFacing->getPosition().get<0>() )
	  entFacingSubjectAngle = 3.1416 - entFacingSubjectAngle;
        
	if (entSubject->getPosition().get<1>() < entFacing->getPosition().get<1>())
	  entFacingSubjectAngle = - entFacingSubjectAngle;
        
        double angleResult = fabs(entFacingAngle - entFacingSubjectAngle);
        
	if( angleResult > angleThreshold ) {
	  return false;
	}
	else { 
	  return true;
	}			
}

int main(int argc, char** argv){
  // Set this in a ros service
  const bool AGENT_FULL_CONFIG = false;  //If false we will use only position and orientation

  ros::init(argc, argv, "SPAR");
  ros::NodeHandle node;

  //Data reading
  PDGHumanReader humanRd(node, AGENT_FULL_CONFIG);
  PDGRobotReader robotRd(node, AGENT_FULL_CONFIG);
  //PDGObjectReader objectRd(node);

  // Set this in a ros service?
  ros::Rate loop_rate(30);

  // Vector of Area
  // It should be possible to add an area on the fly with a ros service.
  std::map<unsigned int, Area*> mapArea;

  // Origin point
  bg::model::point<double, 2, bg::cs::cartesian> origin(0.0, 0.0);

  // AreaRays:
  double interDist = 1.5;
  double dangerDist = 0.5;

  // TODO: put this in a service with parameters
  // We create 2 Area for the robot:
  // Interacting
  CircleArea* interacting = new CircleArea(0,  origin, interDist);
  interacting->setMyOwner(1);
  interacting->setName("pr2_interacting");
  // Danger
  CircleArea* danger = new CircleArea(1,  origin, dangerDist);
  danger->setMyOwner(1);
  danger->setName("pr2_danger");

  mapArea[0] = interacting;
  mapArea[1] = danger;

  /************************/
  /* Start of the Ros loop*/
  /************************/

  while( node.ok() ){
    if( (humanRd.m_LastConfig[101] != NULL) && (robotRd.m_LastConfig[1] != NULL) ){
        // We update area with robot center
        updateEntityArea(mapArea, robotRd.m_LastConfig[1]);

        // We update entities vector inArea
        updateInArea(humanRd.m_LastConfig[101], mapArea);
        
        if( humanRd.m_LastConfig[101]->isInArea(0) ){
            // We will compute here facts that are relevant for interacting
            isFacing( humanRd.m_LastConfig[101], robotRd.m_LastConfig[1], 0.5 );
        }
        else if( humanRd.m_LastConfig[101]->isInArea(1) ){
            // We will compute here facts that are relevant when human is in danger zone
        }
        else{
            // We will compute here facts that are relevant for human out of interacting zone
        }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
