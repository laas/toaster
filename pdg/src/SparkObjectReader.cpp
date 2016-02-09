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
}

void SparkObjectReader::updateObjects() {

    sparkPoster_->update();
    sparkPoster_->getPosterStuct((char*) (&sparkPosterStruct_));
    unsigned int i_obj = 0; /// iterator on detected object

    // Verify that it was indeed updated
    if (sparkPoster_->getUpdatedStatus()) {

        nbObjects_ = sparkPosterStruct_.freeflyerNb;

        for (i_obj = 0; i_obj < nbObjects_; i_obj++) {

            std::stringstream s;
            s << "viman_object_" << sparkPosterStruct_.freeflyer[i_obj].name.name;

            //If object is not in lastConfig_
            if (lastConfig_.find(s.str()) == lastConfig_.end()) {
                MovableObject* myObject = new MovableObject(s.str());
                lastConfig_[s.str()] = myObject;
            }

            //Set position and orientation
            lastConfig_[s.str()]->position_.set<0>(sparkPosterStruct_.freeflyer[i_obj].q[0]);
            lastConfig_[s.str()]->position_.set<1>(sparkPosterStruct_.freeflyer[i_obj].q[1]);
            lastConfig_[s.str()]->position_.set<2>(sparkPosterStruct_.freeflyer[i_obj].q[2]);
            lastConfig_[s.str()]->orientation_[1] = -sparkPosterStruct_.freeflyer[i_obj].q[3];
            lastConfig_[s.str()]->orientation_[0] = -sparkPosterStruct_.freeflyer[i_obj].q[4];
            lastConfig_[s.str()]->orientation_[2] = sparkPosterStruct_.freeflyer[i_obj].q[5];

            //Set the time and name
            lastConfig_[s.str()]->setName(sparkPosterStruct_.freeflyer[i_obj].name.name);
            lastConfig_[s.str()]->setTime(sparkPosterStruct_.time);

            lastConfig_[s.str()]->setConfidence(65);
        }
    }
}

//Destructor

SparkObjectReader::~SparkObjectReader() {
    for (std::map<std::string, MovableObject*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it) {
        delete it->second;
    }
}
