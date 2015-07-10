// This class read topic from niut and convert data into toaster-lib type.
#include <pdg/NiutHumanReader.h>
#include "toaster-lib/MathFunctions.h"

NiutHumanReader::NiutHumanReader(ros::NodeHandle& node, double * kinectPos, 
        bool fullHuman) : kinectPos_(kinectPos) {
    node_ = node;
    fullHuman_ = fullHuman;
    std::cout << "[PDG]Initializing HumanReader" << std::endl;
    // ******************************************
    // Starts listening to the joint_states topic
    // ******************************************
    sub_ = node_.subscribe("/niut/Human", 1, &NiutHumanReader::humanJointCallBack, this);
}

void NiutHumanReader::humanJointCallBack(const niut_msgs::niut_HUMAN_LIST::ConstPtr& msg) {
    Joint curJoint(10000, 100);
    std::vector<int> trackedJoints;
    trackedJoints.push_back(0);
    trackedJoints.push_back(3);
    trackedJoints.push_back(9);
    trackedJoints.push_back(15);
    trackedJoints.push_back(22);

    //msg->filtered_users[i];
    for (int i = 0; i < NB_MAX_NIUT; i++) {
        if (msg->filtered_users[i].trackedId > -1 && msg->filtered_users[i].date.t_sec != 0) {

            unsigned int toasterId = humanIdOffset_ + msg->filtered_users[i].trackedId;
            //If this human was not assigned yet, we create it.
            if (lastConfig_[toasterId] == NULL) {
                Human* tmpHuman = new Human(toasterId);
                lastConfig_[toasterId] = tmpHuman;
            }


            if (fullHuman_) {
                for (unsigned int j = 0; j < trackedJoints.size(); j++) {
                    updateJoint(i, j, curJoint, toasterId, trackedJoints, msg);

                    lastConfig_[toasterId]->skeleton_[curJoint.getName()]->position_.set<0>(curJoint.position_.get<0>());
                    lastConfig_[toasterId]->skeleton_[curJoint.getName()]->position_.set<1>(curJoint.position_.get<1>());
                    lastConfig_[toasterId]->skeleton_[curJoint.getName()]->position_.set<2>(curJoint.position_.get<2>());
                    lastConfig_[toasterId]->skeleton_[curJoint.getName()]->setTime(msg->filtered_users[i].date.t_sec * pow(10, 9) +
                            msg->filtered_users[i].date.t_usec);


                    // We update the human position with the torso (joint 3)
                    if (j == 3) {
                        lastConfig_[toasterId]->position_.set<0>(curJoint.position_.get<0>());
                        lastConfig_[toasterId]->position_.set<1>(curJoint.position_.get<1>());
                        lastConfig_[toasterId]->position_.set<2>(curJoint.position_.get<2>());
                        lastConfig_[toasterId]->setTime(msg->filtered_users[i].date.t_sec * pow(10, 9) +
                                msg->filtered_users[i].date.t_usec);

                        // TODO; Compute the human orientation
                    }

                }
            } else {// not fullHuman_
                // We update the human position with the torso (joint 3)
                updateJoint(i, 3, curJoint, toasterId, trackedJoints, msg);
                lastConfig_[toasterId]->position_.set<0>(curJoint.position_.get<0>());
                lastConfig_[toasterId]->position_.set<1>(curJoint.position_.get<1>());
                lastConfig_[toasterId]->position_.set<2>(curJoint.position_.get<2>());
                lastConfig_[toasterId]->setTime(msg->filtered_users[i].date.t_sec * pow(10, 9) +
                        msg->filtered_users[i].date.t_usec);

                // TODO: compute the human orientation


            }

        }
    }
}

void NiutHumanReader::updateJoint(int i, int j, Joint& curJoint, int toasterId, std::vector<int>& trackedJoints, const niut_msgs::niut_HUMAN_LIST::ConstPtr& msg) {

    std::map<int, std::string> niutToString;
    niutToString[0] = "HEAD";
    niutToString[3] = "TORSO";
    niutToString[9] = "L_HAND";
    niutToString[15] = "R_HIP";
    niutToString[22] = "R_HAND";

    double x, y, z;
    int niutJoint = trackedJoints[j];
    std::string jointString = niutToString[niutJoint];

    //Allocate Joint if needed
    if (lastConfig_[toasterId]->skeleton_[jointString] == NULL) {
        Joint* tmpJoint = new Joint(10001 + j, toasterId);
        tmpJoint->setName(jointString);
        tmpJoint->setAgentId(toasterId);
        lastConfig_[toasterId]->skeleton_[jointString] = tmpJoint;
    }


    x = msg->filtered_users[i].skeleton.joint[niutJoint].position.x;
    y = msg->filtered_users[i].skeleton.joint[niutJoint].position.y;
    z = msg->filtered_users[i].skeleton.joint[niutJoint].position.z;
    curJoint.position_.set<0>(x);
    curJoint.position_.set<1>(y);
    curJoint.position_.set<2>(z);
    projectJoint(curJoint, kinectPos_);
}

void NiutHumanReader::projectJoint(Joint& joint, double* kinectPos) {

    double translation[4][4], rotX[4][4], rotY[4][4], rotZ[4][4], transformation[4][4], tmp1[4][4], tmp2[4][4];

    Joint oldJoint(joint.getId(), joint.getAgentId());
    oldJoint = joint;

    //translation transformation matrice
    translation[0][0] = 1;
    translation[0][1] = 0;
    translation[0][2] = 0;
    translation[0][3] = kinectPos[0];

    translation[1][0] = 0;
    translation[1][1] = 1;
    translation[1][2] = 0;
    translation[1][3] = kinectPos[1];

    translation[2][0] = 0;
    translation[2][1] = 0;
    translation[2][2] = 1;
    translation[2][3] = kinectPos[2];

    translation[3][0] = 0;
    translation[3][1] = 0;
    translation[3][2] = 0;
    translation[3][3] = 1;

    //rotation in x axe transformation matrice
    rotX[0][0] = 1;
    rotX[0][1] = 0;
    rotX[0][2] = 0;
    rotX[0][3] = 0;

    rotX[1][0] = 0;
    rotX[1][1] = cos(kinectPos[3]);
    rotX[1][2] = -sin(kinectPos[3]);
    rotX[1][3] = 0;

    rotX[2][0] = 0;
    rotX[2][1] = sin(kinectPos[3]);
    rotX[2][2] = cos(kinectPos[3]);
    rotX[2][3] = 0;

    rotX[3][0] = 0;
    rotX[3][1] = 0;
    rotX[3][2] = 0;
    rotX[3][3] = 1;

    //rotation in y axe transformation matrice
    rotY[0][0] = cos(kinectPos[4]);
    rotY[0][1] = 0;
    rotY[0][2] = sin(kinectPos[4]);
    rotY[0][3] = 0;

    rotY[1][0] = 0;
    rotY[1][1] = 1;
    rotY[1][2] = 0;
    rotY[1][3] = 0;

    rotY[2][0] = -sin(kinectPos[4]);
    rotY[2][1] = 0;
    rotY[2][2] = cos(kinectPos[4]);
    rotY[2][3] = 0;

    rotY[3][0] = 0;
    rotY[3][1] = 0;
    rotY[3][2] = 0;
    rotY[3][3] = 1;

    //rotation in z axe transformation matrice
    rotZ[0][0] = cos(kinectPos[5]);
    rotZ[0][1] = -sin(kinectPos[5]);
    rotZ[0][2] = 0;
    rotZ[0][3] = 0;

    rotZ[1][0] = sin(kinectPos[5]);
    rotZ[1][1] = cos(kinectPos[5]);
    rotZ[1][2] = 0;
    rotZ[1][3] = 0;

    rotZ[2][0] = 0;
    rotZ[2][1] = 0;
    rotZ[2][2] = 1;
    rotZ[2][3] = 0;

    rotZ[3][0] = 0;
    rotZ[3][1] = 0;
    rotZ[3][2] = 0;
    rotZ[3][3] = 1;

    //transformation matrice = translation*rotX*rotY*rotZ
    MathFunctions::multiplyMatrices4x4(tmp1[0], rotX[0], rotY[0]);
    MathFunctions::multiplyMatrices4x4(tmp2[0], rotZ[0], tmp1[0]);
    MathFunctions::multiplyMatrices4x4(transformation[0], translation[0], tmp2[0]);

    //final transformation

    //         Y  X                  Z  Y
    //         | /                   | /
    // Kinect  |/ ____ Z  ,   World  |/_____X

    joint.position_.set<0>(transformation[0][0] * oldJoint.position_.get<2>() + transformation[0][1] * oldJoint.position_.get<0>() + transformation[0][2] * oldJoint.position_.get<1>() + transformation[0][3]);
    joint.position_.set<1>(transformation[1][0] * oldJoint.position_.get<2>() + transformation[1][1] * oldJoint.position_.get<0>() + transformation[1][2] * oldJoint.position_.get<1>() + transformation[1][3]);
    joint.position_.set<2>(transformation[2][0] * oldJoint.position_.get<2>() + transformation[2][1] * oldJoint.position_.get<0>() + transformation[2][2] * oldJoint.position_.get<1>() + transformation[2][3]);

}
