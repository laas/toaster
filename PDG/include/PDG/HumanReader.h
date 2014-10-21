// A human reader is a class that will read data from a middleware message
// and fill a Human class from toaster-lib accordingly to publish on a ros topic.

#include <ros/ros.h>
#include "toaster-lib/Human.h"
#include "std_msgs/String.h"
#include <map>
#include <string>

class HumanReader{

    public:
        std::map<int, Human> m_LastConfig;
        std::map<int, unsigned long> m_LastTime;
        bool fullHuman_;

        virtual HumanReader(ros::NodeHandle& node, fullHuman) = 0;     // This function will depend on the type of input used.
							               // It will fill the m_LastConfig map.

        bool isPresent(int id);

    private:
        ros::NodeHandle node_;
};
