#include <ostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

class Pr2RobotReader : public RobotReader{

  public:
    Pr2RobotReader::Pr2RobotReader(ros::NodeHandle& node, fullRobot);
    void updateRobot(tf::TransformListener &listener);

  private:
    ros::Subscriber sub;
}
