// A human reader is a class that will read data from a middleware message
// and fill a Human class from toaster-lib accordingly to publish on a ros topic.

#include <ros/ros.h>
#include "toaster-lib/Human.h"
#include <map>
#include <string>

class HumanReader{

    public:
        std::map<unsigned int, Human*> m_LastConfig;
        bool fullHuman_;

        bool isPresent(unsigned int id);

    protected:
        ros::NodeHandle node_;
};
