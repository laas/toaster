//This file will compute the spatial facts concerning agents present in the interaction.

#include "SPAR/PDGHumanReader.h"
#include "SPAR/PDGRobotReader.h"



int main(int argc, char** argv){
  const bool AGENT_FULL_CONFIG = false;  //If false we will use only position and orientation

  ros::init(argc, argv, "SPAR");

  ros::NodeHandle node;

  //Data reading
  PDGHumanReader humanRd(node, AGENT_FULL_CONFIG);
  PDGRobotReader robotRd(node, AGENT_FULL_CONFIG);

  
  ros::Rate loop_rate(30);

  while( node.ok() ){
    if(humanRd.m_LastConfig[101] != NULL){
      printf("Last time human 101: %lu\n", humanRd.m_LastConfig[101]->getTime());

      if(humanRd.isPresent(101))
        printf("[SPAR] human 101 is present! position: %f, %f, %F\n", humanRd.m_LastConfig[101]->getPosition().get<0>(), 
		humanRd.m_LastConfig[101]->getPosition().get<1>(), humanRd.m_LastConfig[101]->getPosition().get<2>());
    }
    if(robotRd.m_LastConfig[1] != NULL){
      printf("Last time robot 1: %lu\n", robotRd.m_LastConfig[1]->getTime());

      if(robotRd.isPresent(1))
        printf("[SPAR] robot 1 is present! position: %f, %f, %F\n", robotRd.m_LastConfig[1]->getPosition().get<0>(), 
		robotRd.m_LastConfig[1]->getPosition().get<1>(), robotRd.m_LastConfig[1]->getPosition().get<2>());
    }
    ros::spinOnce();

    loop_rate.sleep();

  }
  return 0;
}
