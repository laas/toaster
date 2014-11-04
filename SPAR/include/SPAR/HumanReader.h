//This class will be used to read toaster-lib humans from topic.
#include <ros/ros.h>
#include "toaster-lib/Human.h"
#include "std_msgs/String.h"
#include <map>

class HumanReader{

    public:
        std::map<int, Human*> m_LastConfig;
        bool fullHuman_;

        HumanReader(ros::NodeHandle& node, bool fullHuman);

        static void humanJointStateCallBack(const PDG::Human::ConstPtr& msg);

