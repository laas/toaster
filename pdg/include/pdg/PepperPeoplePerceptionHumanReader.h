#ifndef PEPPERPEOPLEPERCEPTIONHUMANREADER_H
#define PEPPERPEOPLEPERCEPTIONHUMANREADER_H
#include "pdg/HumanReader.h"
#include <nao_interaction_msgs/PersonDetectedArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class PepperPeoplePerceptionHumanReader : public HumanReader
{
public:
    PepperPeoplePerceptionHumanReader(ros::NodeHandle &node);
    ~PepperPeoplePerceptionHumanReader();

private:
    ros::Subscriber sub_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_br;
    void personDetectCallback(const nao_interaction_msgs::PersonDetectedArray::ConstPtr &msg);

};

#endif // PEPPERPEOPLEPERCEPTIONHUMANREADER_H
