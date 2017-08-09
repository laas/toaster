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
#include <map>
#include <string>
#include <unistd.h>

class ObjectReader {
public:
  ObjectReader();
  ~ObjectReader();

  static std::map<std::string, MovableObject*> lastConfig_;

protected:
  ros::Subscriber sub_;
  static unsigned int nbReaders_;
  static unsigned int nbObjects_; /// total object number
  unsigned int nbLocalObjects_;

  void increaseNbObjects();

  bool isPresent(std::string id);

  static bool lastConfigMutex_;
  static bool WaitMutex(unsigned int durationSecond)
  {
    while(durationSecond-- > 0)
    {
      if(!lastConfigMutex_)
      {
        lastConfigMutex_ = true;
        return true;
      }
      usleep(1000);
    }
    return false;
  }

  static void releaseMutex()
  {
    lastConfigMutex_ = false;
  }

};

#endif /* OBJECTREADER_H */
