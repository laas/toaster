/* 
 * File:   PDGFactReader.h
 * Author: gmilliez
 *
 * Created on February 6, 2015, 3:24 AM
 */

#ifndef PDGFACTREADER_H
#define	PDGFACTREADER_H

#include <ros/ros.h>
#include "pdg/FactList.h"

class PDGFactReader{

    public:
       pdg::FactList lastMsgFact;
       
       PDGFactReader(ros::NodeHandle& node, std::string subTopic);

    private:
       void factCallBack(const pdg::FactList::ConstPtr& msg);
       ros::Subscriber sub_;

};

#endif	/* PDGFACTREADER_H */

