/* 
 * File:   MorseHumanReader.h
 * Author: Grégoire Milliez
 * mail: gregoire.milliez@laas.fr
 *  Copyright 2014 LAAS/CNRS. All rights reserved.
 *
 * Created on December 3, 2014, 6:19 PM
 */

#ifndef MORSEHUMANREADER_H
#define	MORSEHUMANREADER_H

#include "HumanReader.h"

#include <tf/transform_listener.h>

class MorseHumanReader : public HumanReader
{

  public:
    //Constructor
    MorseHumanReader(ros::NodeHandle& node, bool fullHuman);
    //Destructor
    ~MorseHumanReader();
    
    void init();
    void updateHumans(tf::TransformListener &listener);
    void updateHuman(tf::TransformListener &listener, int humId, std::string humanBase);

  private:
    //static void humanJointStateCallBack(const sensor_msgs::JointState::ConstPtr& msg);
    ros::Subscriber sub_;
};

#endif /* MORSEHUMANREADER_H */