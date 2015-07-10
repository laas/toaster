/* 
 * File:   SparkObjectReader.cpp
 * Author: gmilliez
 * 
 * Created on January 22, 2015, 3:17 PM
 */

#include "pdg/SparkObjectReader.h"

//Constructor

SparkObjectReader::SparkObjectReader() {
    std::cout << "[PDG] Initializing SparkObjectReader" << std::endl;
}

void SparkObjectReader::init(std::string posterName) {
    /* declaration of the poster reader threads */
    sparkPoster_ = new GenomPoster(posterName, (char*) (&sparkPosterStruct_), sizeof (sparkPosterStruct_), 10);
    sparkPoster_->getPosterStuct((char*) (&sparkPosterStruct_));
    nbObjects_ = sparkPosterStruct_.freeflyerNb;
    for (unsigned int i = 0; i < nbObjects_; i++) {
        initObject(i);
    }

}

void SparkObjectReader::initObject(unsigned int i) {
    MovableObject* myObject = new MovableObject(objectIdOffset_ + i);
    //Initialize position:
    myObject->position_.set<0>(0.0);
    myObject->position_.set<1>(0.0);
    myObject->position_.set<2>(0.0);

    myObject->orientation_.push_back(0.0);
    myObject->orientation_.push_back(0.0);
    myObject->orientation_.push_back(0.0);
    lastConfig_[objectIdOffset_ + i] = myObject;

}

void SparkObjectReader::updateObjects() {

    sparkPoster_->update();
    sparkPoster_->getPosterStuct((char*) (&sparkPosterStruct_));
    unsigned int i_obj = 0; /// iterator on detected object

    // Verify that it was indeed updated
    if (sparkPoster_->getUpdatedStatus()) {

        nbObjects_ = sparkPosterStruct_.freeflyerNb;
        // Add objects if needed
        while (lastConfig_.size() != nbObjects_) {
            if (lastConfig_.size() < nbObjects_)
                initObject(lastConfig_.size());
            else {
                std::map<unsigned int, MovableObject*>::iterator it = lastConfig_.end();
                it--;
                lastConfig_.erase(it);
            }
        }



        for (i_obj = 0; i_obj < nbObjects_; i_obj++) {

            //Set position and orientation
            lastConfig_[objectIdOffset_ + i_obj]->position_.set<0>(sparkPosterStruct_.freeflyer[i_obj].q[0]);
            lastConfig_[objectIdOffset_ + i_obj]->position_.set<1>(sparkPosterStruct_.freeflyer[i_obj].q[1]);
            lastConfig_[objectIdOffset_ + i_obj]->position_.set<2>(sparkPosterStruct_.freeflyer[i_obj].q[2]);
            lastConfig_[objectIdOffset_ + i_obj]->orientation_[0] = sparkPosterStruct_.freeflyer[i_obj].q[3];
            lastConfig_[objectIdOffset_ + i_obj]->orientation_[1] = sparkPosterStruct_.freeflyer[i_obj].q[4];
            lastConfig_[objectIdOffset_ + i_obj]->orientation_[2] = sparkPosterStruct_.freeflyer[i_obj].q[5];

            //Set the time and name
            lastConfig_[objectIdOffset_ + i_obj]->setName(sparkPosterStruct_.freeflyer[i_obj].name.name);
            lastConfig_[objectIdOffset_ + i_obj]->setTime(sparkPosterStruct_.time);

            lastConfig_[objectIdOffset_ + i_obj]->setConfidence(65);
        }
    }
}


//Destructor

SparkObjectReader::~SparkObjectReader() {
    for (std::map<unsigned int, MovableObject*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it) {
        delete it->second;
    }
}
