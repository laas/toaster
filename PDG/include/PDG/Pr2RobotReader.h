#include "RobotReader.h"

#include <ostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

class Pr2RobotReader : public RobotReader{

  public:
    Pr2RobotReader(ros::NodeHandle& node, int id, bool fullRobot);
    void updateRobot(tf::TransformListener &listener);
    void init();

  private:
    ros::Subscriber sub_;
    int pr2Id_;
};
