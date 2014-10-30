// A robot reader is a class that will read data from a middleware message
// and fill a Robot class from toaster-lib accordingly to publish on a ros topic.

#include <ros/ros.h>
#include "toaster-lib/Robot.h"
#include "std_msgs/String.h"
#include <map>
#include <string>

class RobotReader{

    public:
        std::map<int, Robot*> m_LastConfig;
        std::map<int, unsigned long> m_LastTime;
        bool fullRobot_;

        virtual void init() = 0;     // This function will depend on the type of input used.
			             // It will fill the m_LastConfig map.

    protected:
        ros::NodeHandle node_;
};
