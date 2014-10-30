#include "HumanReader.h"

#include <ostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

class MorseHumanReader : public HumanReader
{

  public:
    MorseHumanReader(ros::NodeHandle& node, bool fullHuman);
    void init();
    void updateHumans(tf::TransformListener &listener);
    void updateHuman(tf::TransformListener &listener, int humId, std::string humanBase);

  private:
    //static void humanJointStateCallBack(const sensor_msgs::JointState::ConstPtr& msg);
    ros::Subscriber sub_;
};
