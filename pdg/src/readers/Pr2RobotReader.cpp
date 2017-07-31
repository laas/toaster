#include "pdg/readers/Pr2RobotReader.h"

Pr2RobotReader::Pr2RobotReader(ros::NodeHandle& node, bool fullRobot) {
    fullRobot_ = fullRobot;
    std::cout << "Initializing Pr2RobotReader" << std::endl;
    initJointsName_ = false;
    init();
    if (fullRobot)
        sub_ = node.subscribe("joint_states", 1, &Pr2RobotReader::pr2JointStateCallBack, this);
}


// Maybe get this from a config file?

/*void Pr2RobotReader::initJointsName() {
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
}*/

void Pr2RobotReader::init() {
    Robot* curRobot = new Robot("pr2");
    //TODO: setname with id
    curRobot->setName("PR2_ROBOT");
    /*initJointsName();
    if (fullRobot_) {
        std::stringstream jointId;
        jointId << "pr2";
        for (unsigned int i = 0; i < pr2JointsName_.size(); i++) {
            jointId << pr2JointsName_[i];
            curRobot->skeleton_[pr2JointsName_[i]] = new Joint(jointId.str(), "pr2");
        }
    }*/
    lastConfig_["pr2"] = curRobot;
}

void Pr2RobotReader::updateRobot(tf::TransformListener &listener) {
    Robot* curRobot = lastConfig_["pr2"];
    Joint* curJoint = new Joint("pr2_base_link", "pr2");
    curJoint->setName("base_link");

    // We start with base:
    setRobotJointLocation(listener, curJoint);

    curRobot->setOrientation(curJoint->getOrientation());
    curRobot->setPosition(curJoint->getPosition());
    curRobot->setTime(curJoint->getTime());

    delete curJoint;

    //Then other joints if needed
    if (fullRobot_ && initJointsName_) {
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

    ROS_DEBUG("current joint %s \n", jointId.c_str());

    try {
        ros::Time now = ros::Time::now();
        ros::Time last = ros::Time(0);
        listener.waitForTransform("/map", jointId,
                last, ros::Duration(0.0));
        listener.lookupTransform("/map", jointId,
                last, transform);

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

void Pr2RobotReader::pr2JointStateCallBack(const sensor_msgs::JointState::ConstPtr & msg) {
    if (!initJointsName_) {
        for (unsigned int i = 0; i < msg->name.size(); i++) {
            std::string jointName = msg->name[i];
            jointName = jointName.substr(0, msg->name[i].size() - 5);
            if (jointName.compare("r_gripper_") == 0 || jointName.compare("l_gripper_") == 0)
                jointName.append("palm_");
            jointName.append("link");
            pr2JointsName_.push_back(jointName);
            std::stringstream jointId;
            jointId << "pr2";
            jointId << msg->name[i];
            lastConfig_["pr2"]->skeleton_[pr2JointsName_[i]] = new Joint(jointId.str(), "pr2");

        }
        initJointsName_ = true;
    }

    if (pr2JointsName_.size() == msg->position.size()) {
        for (unsigned int i = 0; i < pr2JointsName_.size(); i++) {
            lastConfig_["pr2"]->skeleton_[pr2JointsName_[i]]->position = msg->position[i];
        }
    }
}


//Destructor

Pr2RobotReader::~Pr2RobotReader() {
    for (std::map<std::string, Robot*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it) {
        delete it->second;
    }
}
