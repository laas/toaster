#include "PDG/MorseHumanReader.h"
#include "PDG/Pr2RobotReader.h"


int main(int argc, char** argv){
  const bool AGENT_FULL_CONFIG = false;  //If false we will use only position and orientation


  ros::init(argc, argv, "PDG");


  ros::NodeHandle node;

  MorseHumanReader morseHumanRd(node, AGENT_FULL_CONFIG);
  Pr2RobotReader pr2RobotRd(node, 1, AGENT_FULL_CONFIG);

  tf::TransformListener listener;
  while( node.ok() ){
    ros::spinOnce();
    morseHumanRd.updateHumans(listener);
    printf("Last time human 101: %d\n", morseHumanRd.m_LastTime[101]);

    pr2RobotRd.updateRobot(listener);
    printf("Last time robot pr2: %d\n", pr2RobotRd.m_LastTime[1]);

  }  
}
