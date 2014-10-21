#include <ostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

class MorseHumanReader : public HumanReader{

  public:
    MorseHumanReader::MorseHumanReader(ros::NodeHandle& node, fullHuman);
    void updateHuman(tf::TransformListener &listener);

  private:
    static void humanJointStateCallBack(const sensor_msgs::JointState::ConstPtr& msg);
    ros::Subscriber sub;
}
