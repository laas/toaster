#include "pdg/VimanObjectReader.h"

//Constructor

VimanObjectReader::VimanObjectReader() {
    std::cout << "[PDG] Initializing VimanObjectReader" << std::endl;
}

void VimanObjectReader::init(std::string posterName) {
    /* declaration of the poster reader threads */
    vimanPoster_ = new GenomPoster(posterName, (char*) (&vimanPosterStruct_), sizeof (vimanPosterStruct_), 10);
    vimanPoster_->getPosterStuct((char*) (&vimanPosterStruct_));
    nbObjects_ = vimanPosterStruct_.nbObjects;
}

void VimanObjectReader::updateObjects() {
    vimanPoster_->update();
    vimanPoster_->getPosterStuct((char*) (&vimanPosterStruct_));
    unsigned int i_obj = 0; /// iterator on detected object

    // Verify that it was indeed updated
    if (vimanPoster_->getUpdatedStatus()) {
        nbObjects_ = vimanPosterStruct_.nbObjects;

        for (i_obj = 0; i_obj < nbObjects_; i_obj++) {

            if (vimanPosterStruct_.objects[i_obj].found_Stereo || vimanPosterStruct_.objects[i_obj].found_Left || vimanPosterStruct_.objects[i_obj].found_Right) {

                std::stringstream s;
                s << "viman_object_" << vimanPosterStruct_.objects[i_obj].name;

                //If object is not in lastConfig_
                if (lastConfig_.find(s.str()) == lastConfig_.end()) {
                    MovableObject* myObject = new MovableObject(s.str());
                    lastConfig_[s.str()] = myObject;
                }

                //Set position and orientation
                lastConfig_[s.str()]->position_.set<0>(vimanPosterStruct_.objects[i_obj].thetaMatOrigin.px);
                lastConfig_[s.str()]->position_.set<1>(vimanPosterStruct_.objects[i_obj].thetaMatOrigin.py);
                lastConfig_[s.str()]->position_.set<2>(vimanPosterStruct_.objects[i_obj].thetaMatOrigin.pz);
                lastConfig_[s.str()]->orientation_[1] = -vimanPosterStruct_.objects[i_obj].eulerOrigin.roll;
                lastConfig_[s.str()]->orientation_[0] = -vimanPosterStruct_.objects[i_obj].eulerOrigin.pitch;
                lastConfig_[s.str()]->orientation_[2] = vimanPosterStruct_.objects[i_obj].eulerOrigin.yaw;

                //Set the time and name
                lastConfig_[s.str()]->setName(vimanPosterStruct_.objects[i_obj].name);
                lastConfig_[s.str()]->setTime(vimanPosterStruct_.objects[i_obj].tacq_usec + pow(10, 9) * vimanPosterStruct_.objects[i_obj].tacq_sec);

                //Set confidence
                if (vimanPosterStruct_.objects[i_obj].found_Stereo)
                    lastConfig_[s.str()]->setConfidence(95);
                else
                    lastConfig_[s.str()]->setConfidence(75);

            }
        }
    }
}


//Destructor

VimanObjectReader::~VimanObjectReader() {
    for (std::map<std::string, MovableObject*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it) {
        delete it->second;
    }
}
