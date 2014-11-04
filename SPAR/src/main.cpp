//This file will compute the spatial facts concerning agents present in the interaction.

#include "SPAR/HumanReader.h"



int main(int argc, char** argv){
  const bool AGENT_FULL_CONFIG = false;  //If false we will use only position and orientation

  ros::init(argc, argv, "SPAR");

  ros::NodeHandle node;

  //Data reading
  HumanReader humanRd(node, AGENT_FULL_CONFIG);
  //robotReader robotRd(node, 1, AGENT_FULL_CONFIG);

  
  ros::Rate loop_rate(30);

  while( node.ok() ){
    printf("Last time human 101: %d\n", morseHumanRd.m_LastTime[101]);

    if(HumanReader.isPresent(101))
      printf("[SPAR] human 101 is present! position: %f, %f, %F\n", morseHumanRd.m_LastConfig[101]->getPosition().get<0>(), 
		morseHumanRd.m_LastConfig[101]->getPosition().get<1>(), morseHumanRd.m_LastConfig[101]->getPosition().get<2>());

    ros::spinOnce();

    loop_rate.sleep();

  }

}
