//This file will compute the spatial facts concerning agents present in the interaction.

#include "SPAR/PDGHumanReader.h"



int main(int argc, char** argv){
  const bool AGENT_FULL_CONFIG = false;  //If false we will use only position and orientation

  ros::init(argc, argv, "SPAR");

  ros::NodeHandle node;

  //Data reading
  PDGHumanReader humanRd(node, AGENT_FULL_CONFIG);
  //robotReader robotRd(node, 1, AGENT_FULL_CONFIG);

  
  ros::Rate loop_rate(30);

  while( node.ok() ){
    if(humanRd.m_LastConfig[101] != NULL){
      printf("Last time human 101: %lu\n", humanRd.m_LastConfig[101]->getTime());

      if(humanRd.isPresent(101))
        printf("[SPAR] human 101 is present! position: %f, %f, %F\n", humanRd.m_LastConfig[101]->getPosition().get<0>(), 
		humanRd.m_LastConfig[101]->getPosition().get<1>(), humanRd.m_LastConfig[101]->getPosition().get<2>());
    }
    ros::spinOnce();

    loop_rate.sleep();

  }
  return 0;
}
