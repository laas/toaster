// This class read topic from mocap and convert data into toaster-lib type.

#include "HumanReader.h"

#include <ros/ros.h>
#include "tf/transform_listener.h"
#include "std_msgs/String.h"
#include <string>
#include "geometry_msgs/PoseStamped.h"
#include "spencer_tracking_msgs/TrackedPersons.h"
#include "spencer_tracking_msgs/TrackedPerson.h"
#include "tf/transform_listener.h"
#include <sys/time.h>
#include <math.h>
#include <ostream>

class MocapHumanReader : public HumanReader {
public:
    MocapHumanReader(ros::NodeHandle& node, std::string topic); // This function will fill the m_LastConfig map.

private:
    ros::Subscriber sub_;
    void optitrackCallback(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg);
    void getHumanJointLocation(tf::TransformListener &listener, int joint, std::string personId);
    tf::TransformListener listener_;
};