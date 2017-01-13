#include "pdg/PepperPeoplePerceptionHumanReader.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
PepperPeoplePerceptionHumanReader::PepperPeoplePerceptionHumanReader(ros::NodeHandle &node)
{
    ROS_INFO("Initializing PepperPeoplePerceptionHumanReader");
    ROS_DEBUG("Initializing PepperPeoplePerceptionHumanReader");
    sub_=node.subscribe("/pepper_robot/people_detected",0,&PepperPeoplePerceptionHumanReader::personDetectCallback,this);
    if(!ros::param::has("people_detected_frame")){
        ros::param::set("people_detected_frame","/Head");
    }
}

PepperPeoplePerceptionHumanReader::~PepperPeoplePerceptionHumanReader()
{

}

void PepperPeoplePerceptionHumanReader::personDetectCallback(const nao_interaction_msgs::PersonDetectedArray::ConstPtr &msg)
{
    ROS_INFO("pepper people detected: %lu",msg->person_array.size());
    for(uint i=0;i<msg->person_array.size();i++){
        const nao_interaction_msgs::PersonDetected &person=msg->person_array[i];
        //create a new human with the same id as the message
        std::ostringstream oss_id;
        Human *curHuman;
        Joint* curJoint;
        //oss_id <<"HumanPepper" << person.id;
        oss_id<<"PEPPER_DETECTED_HUMAN";
        std::string humId(oss_id.str());
        ros::Time now(msg->header.stamp);

        ROS_INFO("Pepper people detected %s",humId.c_str());
        if (lastConfig_.find(humId) == lastConfig_.end()) {
            curHuman = new Human(humId);
            curHuman->setName(humId);
            lastConfig_[humId]=curHuman;
        } else {
            curHuman = lastConfig_[humId];
        }
        curHuman->setAge(person.face.age);
        curHuman->setTime(now.toNSec());

        std::string jointNameHead = "head";

        if (curHuman->skeleton_.find(jointNameHead) == curHuman->skeleton_.end()) {
            curJoint = new Joint(jointNameHead, humId);
            curJoint->setName(jointNameHead);
            curHuman->skeleton_[jointNameHead] = curJoint;
        } else {
            curJoint = curHuman->skeleton_[jointNameHead];
        }

        //the head position is in camera frame
        geometry_msgs::PoseStamped pose_in,pose_out;
        pose_in.header=msg->header;
        pose_in.pose= person.person.position;

        geometry_msgs::PoseStamped rot_head,rot_head_out;
        std::string rot_frame_id;
        ros::param::param<std::string>("people_detected_frame",rot_frame_id,"/Head");
        rot_head.header.frame_id=rot_frame_id;
        ROS_INFO_ONCE("PepperPeoplePerception: using %s as base frame for rotation. Set value of ~people_detected_frame parameter",rot_frame_id.c_str());
        if(person.person.face_detected){
            //the head rotation is in XX frame
            rot_head.pose.orientation=person.gaze.head_angle;
        }else{
            rot_head.pose.orientation=pose_in.pose.orientation;
        }

        try{
            tf::Stamped<tf::Transform> tf_in;
            tf::poseStampedMsgToTF(pose_in,tf_in);
            tf::StampedTransform tf_st_in(tf::Transform(tf_in.getRotation(),tf_in.getOrigin()),tf_in.stamp_,tf_in.frame_id_,"/pepper_robot/people_detected/"+humId);
            tf_br.sendTransform(tf_st_in);
        }catch(tf::TransformException e){
            ROS_WARN("tf exception while broadcasting Pepper people detected tf: %s",e.what());
        }

        try{

            ROS_DEBUG("lookup transform %s to %s","/map",msg->header.frame_id.c_str());
            tf_listener_.waitForTransform("/map",msg->header.frame_id,ros::Time(0),ros::Duration(1));
            tf_listener_.transformPose("/map",pose_in,pose_out);
            tf_listener_.transformPose("/map",rot_head,rot_head_out);
        }catch(tf::LookupException e){
            ROS_ERROR("%s",e.what());
            return;
        }catch(tf::ExtrapolationException e){
            ROS_WARN("%s",e.what());
            return;
        }catch(tf::TransformException e){
            ROS_ERROR("%s",e.what());
            return;
        }

        tf::Quaternion q;
        tf::quaternionMsgToTF(rot_head_out.pose.orientation,q);
        double roll, pitch, yaw;
        //apply 180° rotation around global Z axis
        tf::Quaternion q_corr(tf::Vector3(0,0,1),M_PI);
        q=q* q_corr ;
        tf::Matrix3x3 m(q);
        m.getEulerYPR(yaw, pitch, roll);


        bg::model::point<double, 3, bg::cs::cartesian> headPosition;
        headPosition.set<0>(pose_out.pose.position.x);
        headPosition.set<1>(pose_out.pose.position.y);
        headPosition.set<2>(pose_out.pose.position.z);
        std::vector<double> headOrientation;
        headOrientation.push_back(roll);
        headOrientation.push_back(pitch);
        headOrientation.push_back(yaw);

        curJoint->setPosition(headPosition);
        curJoint->setOrientation(headOrientation);
        curJoint->setTime(now.toNSec());


        //update the base
        Joint *jointBase;
        std::string jointBaseName("base");

        if (curHuman->skeleton_.find(jointBaseName) == curHuman->skeleton_.end()) {
            jointBase = new Joint(jointBaseName, humId);
            jointBase->setName(jointBaseName);
            curHuman->skeleton_[jointBaseName]=jointBase;
        } else {
            jointBase = curHuman->skeleton_[jointBaseName];
        }

        bg::model::point<double, 3, bg::cs::cartesian> basePosition=headPosition;
        basePosition.set<2>(0.0);
        std::vector<double> baseOrientation;
        baseOrientation.push_back(0.0);
        baseOrientation.push_back(0.0);
        baseOrientation.push_back(yaw);


        jointBase->setPosition(basePosition);
        jointBase->setOrientation(baseOrientation);
        jointBase->setTime(now.toNSec());

        curHuman->setPosition(basePosition);
        curHuman->setOrientation(baseOrientation);
    }

    std::vector<std::string> to_delete;
    ulong now = ros::Time(msg->header.stamp).toNSec();
    ulong persistence = 5*1e9; // 1e9 ns=1sec
    for(typeof(lastConfig_.begin()) it=lastConfig_.begin();it!=lastConfig_.end();++it){
        Human *curHuman=it->second;
        if(curHuman->getTime()<now-persistence){
            to_delete.push_back(it->first);
        }
    }
    for(typeof(to_delete.begin()) it=to_delete.begin();it!=to_delete.end();++it){
        lastConfig_.erase(*it);
    }


    //TODO : clear old humans not present anymore
}

