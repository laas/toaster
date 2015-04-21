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
#include "niut_msgs/niut_HUMAN_LIST.h"
#include "toaster-lib/Human.h"
#include "HumanReader.h"

class NiutHumanReader : public HumanReader {
public:
    NiutHumanReader(ros::NodeHandle& node, double * kinectPos, bool fullHuman);

private:
    static const unsigned short NB_MAX_NIUT = 16;
    double* kinectPos_;
    ros::Subscriber sub_;

    //Functions
    void humanJointCallBack(const niut_msgs::niut_HUMAN_LIST::ConstPtr& msg);
    void projectJoint(Joint& joint, double* kinectPos);
    void updateJoint(int i, int j, Joint& curJoint, int toasterId, std::vector<int>& trackedJoints,
            const niut_msgs::niut_HUMAN_LIST::ConstPtr& msg);

};

#endif /* NIUTHUMANREADER_H */
