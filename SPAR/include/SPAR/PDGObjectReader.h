/* 
 * File:   PDGObjectReader.h
 * Author: gmilliez
 *
 * Created on December 12, 2014, 2:20 PM
 */

#ifndef PDGOBJECTREADER_H
#define	PDGOBJECTREADER_H

#include <ros/ros.h>
#include "toaster-lib/Object.h"
#include "PDG/ObjectList.h"
#include <map>

class PDGObjectReader{

    public:
       std::map<unsigned int, Object*> lastConfig_;

       bool isPresent(unsigned int id);

       PDGObjectReader(ros::NodeHandle& node);

    private:
       void objectStateCallBack(const PDG::ObjectList::ConstPtr& msg);
       ros::Subscriber sub_;

};

#endif	/* PDGOBJECTREADER_H */

