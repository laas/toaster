/*
 * File:   HumanReader.h
 * Author: Grégoire Milliez
 * mail: gregoire.milliez@laas.fr
 *  Copyright 2014 LAAS/CNRS. All rights reserved.
 *
 * Created on December 3, 2014, 6:19 PM
 */

// A human reader is a class that will read data from a middleware message
// and fill a Human class from toaster-lib accordingly to publish on a ros topic.


#ifndef HUMANREADER_H
#define	HUMANREADER_H

#include "toaster-lib/Human.h"
#include "pdg/readers/Reader.h"
#include "pdg/types.h"
#include <map>
#include <string>

class HumanReader : public Reader<Human>
{
  public:
    HumanReader();
    HumanReader(const HumanReader&) = delete;
    virtual ~HumanReader();

    virtual void Publish(struct toasterList_t& list_msg);

    void setFullConfig(bool fullConfig) {fullHuman_ = fullConfig; }
    void setFullConfig(std::string param)
    {
      if (node_->hasParam(param))
        node_->getParam(param, fullHuman_); 
    }

  public:
    bool fullHuman_;

    bool isPresent(std::string id);

    toaster_msgs::Fact DefaultFactMsg(std::string subjectId, uint64_t factTime);
};

#endif	/* HUMANREADER_H */
