#include "pdg/Pr2RobotReader.h"

Pr2RobotReader::Pr2RobotReader(bool fullRobot) {
    fullRobot_ = fullRobot;
    std::cout << "Initializing Pr2RobotReader" << std::endl;
    init();
}


// Maybe get this from a config file?

void Pr2RobotReader::initJointsName() {
    pr2JointsName_.push_back("base_link");
    pr2JointsName_.push_back("torso_lift_joint");
    pr2JointsName_.push_back("head_pan_joint");
    pr2JointsName_.push_back("head_tilt_joint");
    pr2JointsName_.push_back("laser_tilt_mount_joint");
    pr2JointsName_.push_back("r_shoulder_pan_joint");
    pr2JointsName_.push_back("r_shoulder_lift_joint");
    pr2JointsName_.push_back("r_upper_arm_roll_joint");
    pr2JointsName_.push_back("r_elbow_flex_joint");
    pr2JointsName_.push_back("r_forearm_roll_joint");
    pr2JointsName_.push_back("r_wrist_flex_joint");
    pr2JointsName_.push_back("r_wrist_roll_joint");
    pr2JointsName_.push_back("r_gripper_joint");
    pr2JointsName_.push_back("l_shoulder_pan_joint");
    pr2JointsName_.push_back("l_shoulder_lift_joint");
    pr2JointsName_.push_back("l_upper_arm_roll_joint");
    pr2JointsName_.push_back("l_elbow_flex_joint");
    pr2JointsName_.push_back("l_forearm_roll_joint");
    pr2JointsName_.push_back("l_wrist_flex_joint");
    pr2JointsName_.push_back("l_wrist_roll_joint");
    pr2JointsName_.push_back("l_gripper_joint");
}

void Pr2RobotReader::init() {
    Robot* curRobot = new Robot(robotIdOffset_);
    //TODO: setname with id
    curRobot->setName("pr2");
    initJointsName();
    if (fullRobot_) {
        for (unsigned int i = 0; i < pr2JointsName_.size(); i++) {
            curRobot->skeleton_[pr2JointsName_[i]] = new Joint(10001 + i, robotIdOffset_);
        }
    }
    lastConfig_[robotIdOffset_] = curRobot;
}

void Pr2RobotReader::updateRobot(tf::TransformListener &listener) {
    Robot* curRobot = lastConfig_[robotIdOffset_];
    Joint* curJoint = new Joint(10001, robotIdOffset_);
    curJoint->setName(pr2JointsName_[0]);

    // We start with base:
    setRobotJointLocation(listener, curJoint);

    curRobot->setOrientation(curJoint->getOrientation());
    curRobot->setPosition(curJoint->getPosition());
    curRobot->setTime(curJoint->getTime());
    
    delete curJoint;

    //Then other joints if needed
    if (fullRobot_) {
        for (unsigned int i = 0; i < pr2JointsName_.size(); i++) {
            curJoint = curRobot->skeleton_[pr2JointsName_[i]];
            curJoint->setName(pr2JointsName_[i]);
            setRobotJointLocation(listener, curJoint);
        }
    }
}

void Pr2RobotReader::setRobotJointLocation(tf::TransformListener &listener, Joint* joint) {
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

//Destructor

Pr2RobotReader::~Pr2RobotReader() {
    for (std::map<unsigned int, Robot*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it) {
        delete it->second;
    }
}
