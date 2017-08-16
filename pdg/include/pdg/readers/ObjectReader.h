/*
 * File:  ObjectReader.h
 * Author: Grégoire Milliez
 * mail: gregoire.milliez@laas.fr
 *  Copyright 2014 LAAS/CNRS. All rights reserved.
 *
 * Created on December 3, 2014, 6:19 PM
 */

// An object reader is a class that will read data from a middleware message
// and fill a DynamicObject class from toaster-lib accordingly to publish on a ros topic.

#ifndef OBJECTREADER_H
#define	OBJECTREADER_H

#include <ros/ros.h>
#include "toaster-lib/MovableObject.h"
#include "pdg/readers/Reader.h"
#include "pdg/types.h"
#include <map>
#include <string>
#include <unistd.h>
#include <mutex>
#include <toaster_msgs/FactList.h>

class ObjectReader : public Reader<MovableObject>{
public:
  ObjectReader();
  virtual ~ObjectReader();

  static std::map<std::string, MovableObject*> globalLastConfig_;

  virtual void Publish(struct toasterList_t& list_msg) {};
  void Publish(struct toasterList_t& list_msg, struct objectIn_t& objectIn);

  protected:
  ros::Subscriber sub_;
  static unsigned int nbReaders_;
  static unsigned int nbObjects_; /// total object number
  unsigned int nbLocalObjects_;

  bool isPresent(std::string id);
  void increaseNbObjects();

  toaster_msgs::Fact DefaultFactMsg(string id, MovableObject* object, struct objectIn_t& objectIn);
  void putInHand(struct objectIn_t& objectIn, string id, MovableObject* object, struct toasterList_t& list_msg);

  static std::mutex lastConfigMutex_;
  static std::vector<ObjectReader*> childs_;
};

#endif /* OBJECTREADER_H */
