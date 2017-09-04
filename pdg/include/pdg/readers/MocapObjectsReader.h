/*
 * File:   MocapObjectsReader.h
 * Author: gsarthou
 *
 * Created on 30/08/2017
 */

#ifndef MOCAPOBJECTSREADER_H
#define	MOCAPOBJECTSREADER_H

//This class read topic from mocap and converts data into toaster-lib type.

#include "ObjectReader.h"
#include "pdg/readers/MocapObjectReader.h"

#include <ros/ros.h>
#include <string>
#include <vector>
#include "optitrack/or_pose_estimator_state.h"

class MocapObjectsReader {
public:
    MocapObjectsReader() {};
    ~MocapObjectsReader();

    void init(ros::NodeHandle* node,
              std::string topics,
              std::string ids,
              std::string param = "/pdg/MocapObject");

    void setActivation(bool activated);

    void extract(std::vector<ObjectReader*>& objectReaders);

private:
  std::vector<MocapObjectReader*> readers_;
  void split(const std::string &txt, std::vector<std::string> &strs, char ch);
};

#endif	/* MOCAPOBJECTSREADER_H */
