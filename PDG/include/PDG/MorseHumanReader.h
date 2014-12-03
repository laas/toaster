#include "HumanReader.h"

#include <ostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>

class MorseHumanReader : public HumanReader
{

  public:
    //Constructor
    MorseHumanReader(ros::NodeHandle& node, bool fullHuman);
    //Destructor
    ~MorseHumanReader();
    
    void init();
    void updateHumans(tf::TransformListener &listener);
    void updateHuman(tf::TransformListener &listener, int humId, std::string humanBase);

  private:
    //static void humanJointStateCallBack(const sensor_msgs::JointState::ConstPtr& msg);
    ros::Subscriber sub_;
};
