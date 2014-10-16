// This class read topic from niut and convert data into toaster-lib type.
#include <ros/ros.h>
#include "humanMonitor/niut_HUMAN_LIST.h"
#include "toaster-lib/Human.h"
#include "std_msgs/String.h"
#include <map>
#include <string>

class HumanReader{

    public:
        HumanReader(ros::NodeHandle& node);     // This function will fill the m_LastConfig map.

    private:
        ros::Subscriber sub;
        void humanJointCallBack(const humanMonitor::niut_HUMAN_LIST::ConstPtr& msg);
};
