#include "PDG/VimanObjectReader.h"

//Constructor
VimanObjectReader::VimanObjectReader(){
  std::cout << "[PDG] Initializing VimanObjectReader" << std::endl;
  init();
}

void VimanObjectReader::init(){
    /* declaration of the poster reader threads */
    vimanPoster_ = new GenomPoster("vimanObjectPose", (char*)(&vimanPosterStruct_), sizeof(vimanPosterStruct_), 10);
}

void VimanObjectReader::updateObjects() {
    vimanPoster_->update();
    vimanPosterStruct_ = vimanPoster_->getPosterStuct();
    unsigned int i_obj = 0; /// iterator on detected object

      if(vimanPoster_->getUpdatedStatus){
              nbObjects_ = vimanPosterStruct_.nbObjects;
              /*for(i_obj = 0; i_obj < nbObjects_; i_obj++) {


      if(gEntities->entities[e_j]->is_fixed == TRUE) {
        // update only is_fixed status in position update management poster.
        if(gEntities->entities[e_j]->subtype == HRI_MOVABLE_OBJECT)
          updateObjectPositionUpdateManagementPoster(e_j, gEntities->entities[e_j]->robotPt->name, 0, 0, 0, 0, 0, measurement_type);
        continue;
      }


      if (objectsPosePoster->objects[i_obj].found_Stereo || objectsPosePoster->objects[i_obj].found_Left || objectsPosePoster->objects[i_obj].found_Right) {

        prev_x = gEntities->entities[e_j]->robotPt->joints[1]->abs_pos[0][3];
        prev_y = gEntities->entities[e_j]->robotPt->joints[1]->abs_pos[1][3];
        prev_z = gEntities->entities[e_j]->robotPt->joints[1]->abs_pos[2][3];


        //If We do not used pom then thetaMatOrigin is not valid and we have to do projection to get x,y,z.
        if(isKinectLocalizedByPom){
          x = objectsPosePoster->objects[i_obj].thetaMatOrigin.px; // X
          y = objectsPosePoster->objects[i_obj].thetaMatOrigin.py; // Y
          z = objectsPosePoster->objects[i_obj].thetaMatOrigin.pz; // Z
        }
        else {
          p3d_mat4Copy(p3d_mat4IDENTITY, posMat);
          posMat[0][0] =  objectsPosePoster->objects[i_obj].thetaMatRobot.nx;
          posMat[1][0] =  objectsPosePoster->objects[i_obj].thetaMatRobot.ny;
          posMat[2][0] =  objectsPosePoster->objects[i_obj].thetaMatRobot.nz;
          posMat[0][1] =  objectsPosePoster->objects[i_obj].thetaMatRobot.ox;
          posMat[1][1] =  objectsPosePoster->objects[i_obj].thetaMatRobot.oy;
          posMat[2][1] =  objectsPosePoster->objects[i_obj].thetaMatRobot.oz;
          posMat[0][2] =  objectsPosePoster->objects[i_obj].thetaMatRobot.ax;
          posMat[1][2] =  objectsPosePoster->objects[i_obj].thetaMatRobot.ay;
          posMat[2][2] =  objectsPosePoster->objects[i_obj].thetaMatRobot.az;
          posMat[0][3] =  objectsPosePoster->objects[i_obj].thetaMatRobot.px; // X
          posMat[1][3] =  objectsPosePoster->objects[i_obj].thetaMatRobot.py; // Y
          posMat[2][3] =  objectsPosePoster->objects[i_obj].thetaMatRobot.pz; // Z

          //Get the pose of the robot (jido or pr2) in the scene
          p3d_mat4Copy(gRobot->robotPt->joints[1]->abs_pos, posRobotMat);

          // compute the pose of the object in the scene (world) frame
          p3d_mat4Mult(posRobotMat, posMat, posObjectInSceneMat);

          x = posObjectInSceneMat[0][3]; // X
          y = posObjectInSceneMat[1][3]; // Y
          z = posObjectInSceneMat[2][3]; // Z
        }

        /// add new perception info in pose perception info poster.
        if(SDI_F->state.fillPosePerceptionInfoPoster && SDI_F->state.fillPosePerceptionInfoPosterStartReady && newVimanImage)
        addNewEntityPosePerceptionInPosePerceptionPoster(e_j,objectsPosePoster->objects[i_obj].found_Stereo,objectsPosePoster->objects[i_obj].found_Left,objectsPosePoster->objects[i_obj].found_Right,x,y,z);

        /// If Ongoing Inferrence, Test Perception - Inferrence conflict
        if (gEntities->entities[e_j]->hasInferrence)
          assessPerceptionInferrenceConflict(e_j, x, y, z);

        // Update position if it exceeds a certain distance threshold
        // or if object is considered as static and current distance between
        // former and new position is lower than minimum distance between former
        // and new position since last motion
        // => This is to get better positionning for furnitures without updating too much
        // TODO: It would be more relevent to take the average position of the object.
        //printf("Object %s %f %f %f %f %f %f %f %f %f\n",gEntities->entities[e_j]->robotPt->name ,x,y,z,prev_x,prev_y,prev_z ,objectsPosePoster->objects[i_obj].thetaMatRobot.px ,objectsPosePoster->objects[i_obj].thetaMatRobot.py,objectsPosePoster->objects[i_obj].thetaMatRobot.pz);
        dist = D3D(x,y,z,prev_x,prev_y,prev_z);
        // Update Object Position - Threshold much higher for mono to give priority to stereo position with better localization.
        updateMovingObjPos = false;
        updateStaticObjPos = false;
        if (objectsPosePoster->objects[i_obj].found_Stereo) {
          measurement_type =  PRECISE_STATIC;
          if( dist > SPARK_OBJECT_DEFAULT_UPDATE_DIST){
            //printf("Object %s moves of %f \n", gEntities->entities[e_j]->robotPt->name , dist);
            updateMovingObjPos = true;
            gEntities->entities[e_j]->minStaticDist = dist;
          }
          else if (((SPARK_OBJECT_MOTION) gEntities->entities[e_j]->filtered_motion == OBJ_STATIC) && (dist < gEntities->entities[e_j]->minStaticDist)){
            //printf("Object %s positioned is improved \n",gEntities->entities[e_j]->robotPt->name);
            updateStaticObjPos = true;
            gEntities->entities[e_j]->minStaticDist = dist;
          }
          if(updateMovingObjPos){
            //entity will be considered as moving at least during the next SPARK_OBJECT_HISTORY_LENGTH steps.
            gEntities->entities[e_j]->last_ismoving_iter = SPARK_OBJECT_HISTORY_LENGTH;
            measurement_type =  PRECISE_MOVING;
          }
        }
        /// We are detecting but not in stereo
        else {
          if( dist > SPARK_OBJECT_DEFAULT_ROUGH_UPDATE_DIST){
            gEntities->entities[e_j]->last_ismoving_iter = SPARK_OBJECT_HISTORY_LENGTH;
            measurement_type =  ROUGH_MOVING;
            updateMovingObjPos = true;
          }
          else
            measurement_type =  ROUGH_STATIC;
        }

        if(updateMovingObjPos || updateStaticObjPos ) {
          objectQ =  MY_ALLOC(double, gEntities->entities[e_j]->robotPt->nb_dof);

          //If We do not use Pom it was already done above
          if(isKinectLocalizedByPom){
            p3d_mat4Copy(p3d_mat4IDENTITY, posMat);
            posMat[0][0] =  objectsPosePoster->objects[i_obj].thetaMatRobot.nx;
            posMat[1][0] =  objectsPosePoster->objects[i_obj].thetaMatRobot.ny;
            posMat[2][0] =  objectsPosePoster->objects[i_obj].thetaMatRobot.nz;
            posMat[0][1] =  objectsPosePoster->objects[i_obj].thetaMatRobot.ox;
            posMat[1][1] =  objectsPosePoster->objects[i_obj].thetaMatRobot.oy;
            posMat[2][1] =  objectsPosePoster->objects[i_obj].thetaMatRobot.oz;
            posMat[0][2] =  objectsPosePoster->objects[i_obj].thetaMatRobot.ax;
            posMat[1][2] =  objectsPosePoster->objects[i_obj].thetaMatRobot.ay;
            posMat[2][2] =  objectsPosePoster->objects[i_obj].thetaMatRobot.az;
            posMat[0][3] =  objectsPosePoster->objects[i_obj].thetaMatRobot.px; // X
            posMat[1][3] =  objectsPosePoster->objects[i_obj].thetaMatRobot.py; // Y
            posMat[2][3] =  objectsPosePoster->objects[i_obj].thetaMatRobot.pz; // Z

            //Get the pose of the robot (jido or pr2) in the scene
            p3d_mat4Copy(gRobot->robotPt->joints[1]->abs_pos, posRobotMat);

            // compute the pose of the object in the scene (world) frame
            p3d_mat4Mult(posRobotMat, posMat, posObjectInSceneMat);
          }

          //p3d_set_freeflyer_pose_into(gEntities->entities[e_j]->robotPt, posMat, objectQ);
          p3d_set_freeflyer_pose_into(gEntities->entities[e_j]->robotPt, posObjectInSceneMat, objectQ);

          p3d_set_and_update_this_robot_conf(gEntities->entities[e_j]->robotPt, objectQ);

          if(place_robot_conf_to_environment_poster(gEntities->entities[e_j]->robotPt, objectQ))
            publish_environment_poster();

          MY_FREE(objectQ, double, gEntities->entities[e_j]->robotPt->nb_dof); /* FREE */
          //TODO: Check if mightabilities work with spark
          //if(SDI_F->state.mightabilitiesEnabled)
          //  execute_Mightability_Map_functions();
          //Object position is updated in Move3D
        }

        if(gEntities->entities[e_j]->detection_time != objectsPosePoster->objects[i_obj].tacq_usec){
          gEntities->entities[e_j]->is_present = TRUE;
          gEntities->entities[e_j]->is_detected = TRUE;

          //Update Object Position Update Managment Poster Value For this object
          //if it is a
          //It will be used by supervisor to drive visual attention
          if(gEntities->entities[e_j]->subtype == HRI_MOVABLE_OBJECT)
            updateObjectPositionUpdateManagementPoster(e_j, gEntities->entities[e_j]->robotPt->name, x, y, z, objectsPosePoster->objects[i_obj].tacq_sec , dist, measurement_type);


          //if(((HRI_MOTION) objectsPosePoster->objects[i_obj].motion) == HRI_MOVING)
          //gEntities->entities[e_j]->last_ismoving_iter = SPARK_OBJECT_HISTORY_LENGTH;
        }
        gEntities->entities[e_j]->last_detection_time = gEntities->entities[e_j]->detection_time;
        gEntities->entities[e_j]->detection_time = objectsPosePoster->objects[i_obj].tacq_usec;

          //update_object_motion(e_j, objectsPosePoster->objects[i].tacq_usec, objectsPosePoster->objects[i].motion);
      } // endif viman detects stereo
      else {
        if(gEntities->entities[e_j]->undetection_status !=  HRI_NEVER_DETECTED){
          gEntities->entities[e_j]->is_detected = FALSE;
        }


        //*******************************
        // INFERED POSITION MANAGEMENT
        //*******************************

        //Manage Object In Hand For Robot
        //More Generally we could handle object position of object who are not percieved but for which we can infer some position
        if(gEntities->entities[e_j]->hasInferrence || gEntities->entities[e_j]->disappeared){

          //&& !strcmp(agent->object_name, gEntities->entities[e_j]->robotPt->name)) {

          //x, y, z are now at 6, 7 and 8 index of the q.
          //We should try to manage it better
          // We find old position in environment poster
          prev_x = gEntities->entities[e_j]->robotPt->ROBOT_POS[6];
          prev_y = gEntities->entities[e_j]->robotPt->ROBOT_POS[7];
          prev_z = gEntities->entities[e_j]->robotPt->ROBOT_POS[8];

          // Put inferred x,y,z
          if(gEntities->entities[e_j]->hasInferrence){
            setXYZInferredPosition(e_j);
            /// for SPARK_PRECISE_ROBOT_HAND position. It's already processed exactly inside spark
            if ( gEntities->entities[e_j]->inferrenceType == SPARK_PRECISE_ROBOT_HAND){
              x = gEntities->entities[e_j]->robotPt->joints[1]->abs_pos[0][3];
              y = gEntities->entities[e_j]->robotPt->joints[1]->abs_pos[1][3];
              z = gEntities->entities[e_j]->robotPt->joints[1]->abs_pos[2][3];
            }
            else {
              x = gEntities->entities[e_j]->infx;
              y = gEntities->entities[e_j]->infy;
              z = gEntities->entities[e_j]->infz;
            }
          }
          else if (gEntities->entities[e_j]->disappeared){
            x = 0;
            y = 0;
            z = 0;
          }

          dist = D3D(x, y, z,prev_x,prev_y,prev_z);
          updateMovingObjPos = false;
          updateStaticObjPos = false;
          if( dist > SPARK_OBJECT_DEFAULT_UPDATE_DIST){
            updateMovingObjPos = true;
            gEntities->entities[e_j]->minStaticDist = dist;
          }
          else if (((SPARK_OBJECT_MOTION) gEntities->entities[e_j]->filtered_motion == OBJ_STATIC) && (dist < gEntities->entities[e_j]->minStaticDist)){
            updateStaticObjPos = true;
            gEntities->entities[e_j]->minStaticDist = dist;
          }
          measurement_type =  INFERRED_STATIC;

          if(updateMovingObjPos || updateStaticObjPos ) {
            if(updateMovingObjPos){
              measurement_type =  INFERRED_MOVING;
              gEntities->entities[e_j]->last_ismoving_iter = SPARK_OBJECT_HISTORY_LENGTH;
            }

            //Change Position In Interface except for SPARK_PRECISE_ROBOT_HAND
            if ( gEntities->entities[e_j]->inferrenceType != SPARK_PRECISE_ROBOT_HAND){
              objectQ = MY_ALLOC(double, gEntities->entities[e_j]->robotPt->nb_dof); /* ALLOC */
              p3d_get_robot_config_into(gEntities->entities[e_j]->robotPt, &objectQ);
              objectQ[6] = x;
              objectQ[7] = y;
              objectQ[8] = z;
              p3d_set_and_update_this_robot_conf(gEntities->entities[e_j]->robotPt, objectQ);
              MY_FREE(objectQ, double, gEntities->entities[e_j]->robotPt->nb_dof); /* FREE */
            }

            //Publish Environment Poster
            if(place_robot_p3d_rob_to_environment_poster(gEntities->entities[e_j]->robotPt))
              publish_environment_poster();
          }
          else
          if(gEntities->entities[e_j]->subtype == HRI_MOVABLE_OBJECT)
            updateObjectPositionUpdateManagementPoster(e_j,gEntities->entities[e_j]->robotPt->name,x,y,z,time(0) , dist , measurement_type);

        }
        /// Object is not in hand
        else {
          if(gEntities->entities[e_j]->subtype == HRI_MOVABLE_OBJECT)
            updateObjectPositionUpdateManagementPoster(e_j,gEntities->entities[e_j]->robotPt->name,0,0,0,0, 0 , measurement_type);
        }

      }
    } // endfor vimanposter objects*/
      }
}

//Destructor
VimanObjectReader::~VimanObjectReader(){
    for(std::map<unsigned int, MovableObject*>::iterator it = lastConfig_.begin() ; it != lastConfig_.end() ; ++it){
        delete it->second;
    }
}