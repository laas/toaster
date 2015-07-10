/* 
 * File:   SpencerRobotReader.cpp
 * Author: gmilliez
 * 
 * Created on April 20, 2015, 5:15 PM
 */

#include "pdg/SpencerRobotReader.h"

SpencerRobotReader::SpencerRobotReader(bool fullRobot) {
    fullRobot_ = fullRobot;
    std::cout << "Initializing SpencerRobotReader" << std::endl;
    init();
}

// Maybe get this from a config file?

void SpencerRobotReader::initJointsName() {
    spencerJointsName_.push_back("base_link");
}

void SpencerRobotReader::init() {
    Robot* curRobot = new Robot(robotIdOffset_);
    //TODO: setname with id
    curRobot->setName("spencer");
    initJointsName();
    lastConfig_[robotIdOffset_] = curRobot;
}

void SpencerRobotReader::updateRobot(tf::TransformListener &listener) {
    Robot* curRobot = lastConfig_[robotIdOffset_];
    Joint* curJoint = new Joint(10001, robotIdOffset_);
    curJoint->setName(spencerJointsName_[0]);

    // We start with base:
    setRobotJointLocation(listener, curJoint);

    curRobot->setOrientation(curJoint->getOrientation());
    curRobot->setPosition(curJoint->getPosition());
    curRobot->setTime(curJoint->getTime());

    //printf("spencer robot: %f, %f, %f\n", curRobot->getPosition().get<0>(), curRobot->getPosition().get<1>(), curRobot->getPosition().get<2>());

    delete curJoint;


}

void SpencerRobotReader::setRobotJointLocation(tf::TransformListener &listener, Joint* joint) {
    tf::StampedTransform transform;
    std::string jointId = "/";
    std::vector<double> jointOrientation;
    bg::model::point<double, 3, bg::cs::cartesian> jointPosition;
    jointId.append(joint->getName());
    try {
        ros::Time now = ros::Time::now();
        listener.waitForTransform("/map", jointId,
                now, ros::Duration(3.0));
        listener.lookupTransform("/map", jointId,
                now, transform);

        //Joint position
        jointPosition.set<0>(transform.getOrigin().x());
        jointPosition.set<1>(transform.getOrigin().y());
        jointPosition.set<2>(transform.getOrigin().z());

        //Joint orientation
        //curRobot->orientation.push_back(tf::getRoll(transform.getRotation()));
        //curRobot->orientation.push_back(tf::getPitch(transform.getRotation()));
        jointOrientation.push_back(0.0);
        jointOrientation.push_back(0.0);
        jointOrientation.push_back(tf::getYaw(transform.getRotation()));

        joint->setTime(now.toNSec());
        joint->setPosition(jointPosition);
        joint->setOrientation(jointOrientation);

    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
}

SpencerRobotReader::~SpencerRobotReader() {
}

