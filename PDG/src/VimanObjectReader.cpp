#include "PDG/VimanObjectReader.h"

//Constructor

VimanObjectReader::VimanObjectReader() {
    std::cout << "[PDG] Initializing VimanObjectReader" << std::endl;
    init();
}

void VimanObjectReader::init() {
    /* declaration of the poster reader threads */
    vimanPoster_ = new GenomPoster("vimanObjectPose", (char*) (&vimanPosterStruct_), sizeof (vimanPosterStruct_), 10);
    vimanPoster_->getPosterStuct( (char*) (&vimanPosterStruct_) );
    nbObjects_ = vimanPosterStruct_.nbObjects;
    for (unsigned int i = 0; i < nbObjects_; i++) {
        initObject(i);
    }
}

void VimanObjectReader::initObject(unsigned int i) {
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

void VimanObjectReader::updateObjects() {
    vimanPoster_->update();
    vimanPoster_->getPosterStuct( (char*) (&vimanPosterStruct_) );
    unsigned int i_obj = 0; /// iterator on detected object

    // Verify that it was indeed updated
    if (vimanPoster_->getUpdatedStatus()) {
        nbObjects_ = vimanPosterStruct_.nbObjects;
        // Add objects if needed
        while (nbObjects_ < lastConfig_.size())
            initObject(lastConfig_.size());

        for (i_obj = 0; i_obj < nbObjects_; i_obj++) {
            
            if (vimanPosterStruct_.objects[i_obj].found_Stereo || vimanPosterStruct_.objects[i_obj].found_Left || vimanPosterStruct_.objects[i_obj].found_Right) {
                lastConfig_[objectIdOffset_ + i_obj]->position_.set<0>(vimanPosterStruct_.objects[i_obj].eulerOrigin.x);
                lastConfig_[objectIdOffset_ + i_obj]->position_.set<1>(vimanPosterStruct_.objects[i_obj].eulerOrigin.y);
                lastConfig_[objectIdOffset_ + i_obj]->position_.set<2>(vimanPosterStruct_.objects[i_obj].eulerOrigin.z);
                lastConfig_[objectIdOffset_ + i_obj]->orientation_[0] = vimanPosterStruct_.objects[i_obj].eulerOrigin.roll;
                lastConfig_[objectIdOffset_ + i_obj]->orientation_[1] = vimanPosterStruct_.objects[i_obj].eulerOrigin.pitch;
                lastConfig_[objectIdOffset_ + i_obj]->orientation_[2] = vimanPosterStruct_.objects[i_obj].eulerOrigin.yaw;
                
                //Set confidence
                if (vimanPosterStruct_.objects[i_obj].found_Stereo)
                    lastConfig_[objectIdOffset_ + i_obj]->setConfidence(95);
                else
                    lastConfig_[objectIdOffset_ + i_obj]->setConfidence(75);
                
            }
        }
    }
}
                
                
                        /*//If We do not used pom then thetaMatOrigin is not valid and we have to do projection to get x,y,z.
                        if(isKinectLocalizedByPom){
                          x = vimanPosterStruct_.objects[i_obj].thetaMatOrigin.px; // X
                          y = vimanPosterStruct_.objects[i_obj].thetaMatOrigin.py; // Y
                          z = vimanPosterStruct_.objects[i_obj].thetaMatOrigin.pz; // Z
                        }
                        else {
                          p3d_mat4Copy(p3d_mat4IDENTITY, posMat);
                          posMat[0][0] =  vimanPosterStruct_.objects[i_obj].thetaMatRobot.nx;
                          posMat[1][0] =  vimanPosterStruct_.objects[i_obj].thetaMatRobot.ny;
                          posMat[2][0] =  vimanPosterStruct_.objects[i_obj].thetaMatRobot.nz;
                          posMat[0][1] =  vimanPosterStruct_.objects[i_obj].thetaMatRobot.ox;
                          posMat[1][1] =  vimanPosterStruct_.objects[i_obj].thetaMatRobot.oy;
                          posMat[2][1] =  vimanPosterStruct_.objects[i_obj].thetaMatRobot.oz;
                          posMat[0][2] =  vimanPosterStruct_.objects[i_obj].thetaMatRobot.ax;
                          posMat[1][2] =  vimanPosterStruct_.objects[i_obj].thetaMatRobot.ay;
                          posMat[2][2] =  vimanPosterStruct_.objects[i_obj].thetaMatRobot.az;
                          posMat[0][3] =  vimanPosterStruct_.objects[i_obj].thetaMatRobot.px; // X
                          posMat[1][3] =  vimanPosterStruct_.objects[i_obj].thetaMatRobot.py; // Y
                          posMat[2][3] =  vimanPosterStruct_.objects[i_obj].thetaMatRobot.pz; // Z

                          //Get the pose of the robot (jido or pr2) in the scene
                          p3d_mat4Copy(gRobot->robotPt->joints[1]->abs_pos, posRobotMat);

                          // compute the pose of the object in the scene (world) frame
                          p3d_mat4Mult(posRobotMat, posMat, posObjectInSceneMat);

                          x = posObjectInSceneMat[0][3]; // X
                          y = posObjectInSceneMat[1][3]; // Y
                          z = posObjectInSceneMat[2][3]; // Z
                        }

        
 
                        gEntities->entities[e_j]->detection_time = vimanPosterStruct_.objects[i_obj].tacq_usec;
                        
            } // endif viman detects stereo


        } // endfor vimanposter objects
    }
}*/

//Destructor

VimanObjectReader::~VimanObjectReader() {
    for (std::map<unsigned int, MovableObject*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it) {
        delete it->second;
    }
}