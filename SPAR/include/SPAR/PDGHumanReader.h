//This class will be used to read toaster-lib humans from topic.
#include <ros/ros.h>
#include "toaster-lib/Human.h"
#include "PDG/Human.h"
#include <map>

class PDGHumanReader{

    public:
       std::map<int, Human*> m_LastConfig;
       bool fullHuman_;

       bool isPresent(int id);

       PDGHumanReader(ros::NodeHandle& node, bool fullHuman);

    private:
       void humanJointStateCallBack(const PDG::Human::ConstPtr& msg);

};

