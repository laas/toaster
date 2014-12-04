/* 
 * File:   NiutHumanReader.h
 * Author: Grégoire Milliez
 * mail: gregoire.milliez@laas.fr
 *  Copyright 2014 LAAS/CNRS. All rights reserved.
 *
 * Created on December 3, 2014, 6:19 PM
 */

// This class read topic from niut and convert data into toaster-lib type.

#ifndef NIUTHUMANREADER_H
#define	NIUTHUMANREADER_H

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

#endif /* NIUTHUMANREADER_H */
