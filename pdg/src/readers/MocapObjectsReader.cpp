/*
 * File:   MocapObjectsReader.cpp
 * Author: gsarthou
 *
 */

#include "pdg/readers/MocapObjectsReader.h"

#include <ostream>
#include <sstream>

MocapObjectsReader::~MocapObjectsReader()
{
  for(unsigned int i = 0; i < readers_.size(); i++)
  {
    delete readers_[i];
  }
  readers_.clear();
}

void MocapObjectsReader::init(ros::NodeHandle* node,
          std::string topics,
          std::string ids,
          std::string param)
{
  std::string topic = topics + ":";
  std::vector<std::string> mocapTopic;
  std::string id = ids + ":";
  std::vector<std::string> mocapId;

  split(topic, mocapTopic, ':');
  split(id, mocapId, ':');

  if(mocapTopic.size() == mocapId.size())
  {
    readers_.resize(mocapTopic.size());
    for(unsigned int i = 0; i < readers_.size(); i++)
    {
      readers_[i] = new MocapObjectReader();
      readers_[i]->init(node, mocapTopic[i], mocapId[i], param);
    }
  }
  else
    std::cout << "[PDG ERROR] Mocap readers parameters not of same length " << std::endl;
}

void MocapObjectsReader::extract(std::vector<ObjectReader*>& objectReaders)
{
  for(unsigned int i = 0; i < readers_.size(); i++)
  {
    objectReaders.push_back(readers_[i]);
  }
}

void MocapObjectsReader::split(const std::string &txt, std::vector<std::string> &strs, char ch)
{
  istringstream iss(txt.c_str());
  std::string s;
  while(getline(iss, s, ch))
  {
    strs.push_back(s);
  }
}

void MocapObjectsReader::setActivation(bool activated)
{
  for(unsigned int i = 0; i < readers_.size(); i++)
  {
    readers_[i]->setActivation(activated);
  }
}
