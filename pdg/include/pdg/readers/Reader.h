/*
 * File:   Reader.h
 * Author: Guillaume SArthou
 * mail: gsarthou@laas.fr
 *  Copyright 2017 LAAS/CNRS. All rights reserved.
 *
 * Created on August 9, 2017
 */

#ifndef READER_H
#define	READER_H

#include <string>
#include <toaster_msgs/SetEntityPose.h>

#include "toaster-lib/MovableIoTObject.h"
#include "pdg/utility/EntityUtility.h"

template <typename T>
class Reader{

public:
  Reader() {};
  ~Reader() {};

  std::map<std::string, T*> lastConfig_;

  virtual bool isPresent(std::string id) = 0;

  void updateEntityPose(Entity& newPoseEnt, std::string id, Entity* storedEntity);
  void updateEntityPose(Entity& newPoseEnt);
};

template <typename T>
void Reader<T>::updateEntityPose(Entity& newPoseEnt, std::string id, Entity* storedEntity)
{
  if(newPoseEnt.getId() == id)
  {
      updateEntity(newPoseEnt, storedEntity);
      //Reset newPoseEnt_
      newPoseEnt.setId("");
  }
}

template <typename T>
void Reader<T>::updateEntityPose(Entity& newPoseEnt)
{
  for (typename std::map<std::string, T*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it)
      updateEntityPose(newPoseEnt, it->first, (Entity*)it->second);
}


#endif	/* READER_H */
