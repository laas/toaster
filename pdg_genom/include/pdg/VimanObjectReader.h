/* 
 * File:   VimanObjectReader.h
 * Author: Grégoire Milliez
 * mail: gregoire.milliez@laas.fr
 *  Copyright 2014 LAAS/CNRS. All rights reserved.
 *
 * Created on December 3, 2014, 6:19 PM
 */

#ifndef VIMANOBJECTREADER_H
#define	VIMANOBJECTREADER_H

#include "ObjectReader.h"
#include "middleware/GenomPoster.h"
#include "viman/vimanStruct.h"

class VimanObjectReader : public ObjectReader
{

  public:
    //Constructor
    VimanObjectReader();
    //Destructor
    ~VimanObjectReader();
    void updateObjects();
    void init(std::string posterName);
    
  private:
    GenomPoster* vimanPoster_;
    VimanObjectPublicArray vimanPosterStruct_;
    
    //init functions    
    void initObject(unsigned int i);
};

#endif /* VIMANOBJECTREADER_H */
