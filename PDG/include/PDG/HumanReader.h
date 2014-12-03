// A human reader is a class that will read data from a middleware message
// and fill a Human class from toaster-lib accordingly to publish on a ros topic.

#include <ros/ros.h>
#include "toaster-lib/Human.h"
#include <map>

class HumanReader{

    public:
        std::map<unsigned int, Human*> lastConfig_;
        bool fullHuman_;

        bool isPresent(unsigned int id);

    protected:
        ros::NodeHandle node_;
};
