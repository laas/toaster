/* 
 * File:   SparkObjectReader.h
 * Author: gmilliez
 *
 * Created on January 22, 2015, 3:17 PM
 */

#ifndef SPARKOBJECTREADER_H
#define	SPARKOBJECTREADER_H

#include "ObjectReader.h"
#include "middleware/GenomPoster.h"
#include "spark/sparkStruct.h"

class SparkObjectReader : public ObjectReader {
public:
    //Constructor
    SparkObjectReader();
    //Destructor
    ~SparkObjectReader();
    void updateObjects();

    void init(std::string posterName);

private:
    GenomPoster* sparkPoster_;
    STRUCT_SPARK_CURRENT_ENVIRONMENT sparkPosterStruct_;

    //init functions    
    void initObject(unsigned int i);
};

#endif	/* SPARKOBJECTREADER_H */

