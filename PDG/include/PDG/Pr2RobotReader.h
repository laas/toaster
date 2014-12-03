#include "RobotReader.h"

#include <ostream>
#include <tf/transform_listener.h>

class Pr2RobotReader : public RobotReader{

  public:
    Pr2RobotReader(unsigned int id, bool fullRobot);
    void updateRobot(tf::TransformListener &listener);
    
    //Destructor
    ~Pr2RobotReader();

  private:
    ros::Subscriber sub_;
    unsigned int pr2Id_;
    std::vector<std::string> pr2JointsName_;
    void initJointsName();
    void init();
    void setRobotJointLocation(tf::TransformListener &listener, Joint* joint);
};
