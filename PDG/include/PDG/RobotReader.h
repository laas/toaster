// A robot reader is a class that will read data from a middleware message
// and fill a Robot class from toaster-lib accordingly to publish on a ros topic.

#include <ros/ros.h>
#include "toaster-lib/Robot.h"
#include <map>
#include <string>

class RobotReader{

    public:
        std::map<unsigned int, Robot*> lastConfig_;
        bool fullRobot_;

        virtual void init() = 0;     // This function will depend on the robot

        bool isPresent(unsigned int id);
};
