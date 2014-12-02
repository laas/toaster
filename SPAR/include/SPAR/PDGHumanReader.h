
/* 
 * File:   PDGRobotReader.h
 * Author: gmilliez
 *
 * Created on November 12, 2014, 6:24 PM
 * This class will be used to read toaster-lib humans from topic.

 */

#ifndef PDGHUMANREADER_H
#define	PDGHUMANREADER_H

#include <ros/ros.h>
#include "toaster-lib/Human.h"
#include "PDG/Human.h"
#include <map>

class PDGHumanReader{

    public:
       std::map<unsigned int, Human*> m_LastConfig;
       bool fullHuman_;

       bool isPresent(unsigned int id);

       PDGHumanReader(ros::NodeHandle& node, bool fullHuman);

    private:
       void humanJointStateCallBack(const PDG::Human::ConstPtr& msg);
       ros::Subscriber sub_;

};

#endif	/* PDGHUMANREADER_H */