/* 
 * File:   SparkFactReader.h
 * Author: gmilliez
 *
 * Created on May 31, 2015, 1:00 PM
 */

#ifndef SPARKFACTREADER_H
#define	SPARKFACTREADER_H

#include "pdg/FactReader.h"
#include "middleware/GenomPoster.h"
#include "spark/sparkStruct.h"

class SparkFactReader : public FactReader
{

  public:
    //Constructor
    SparkFactReader();
    //Destructor
    ~SparkFactReader();
    void updateFacts();
    
    //init functions    
    void init(std::string posterName);
    
  private:
    GenomPoster* sparkFactPoster_;
    SPARK_ALL_FACT sparkFactPosterStruct_;
    
};

#endif /* SPARKFACTREADER_H */
