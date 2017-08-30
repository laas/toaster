// Message generated class
#include <toaster_msgs/AddStream.h>
#include <toaster_msgs/PutInHand.h>
#include <toaster_msgs/RemoveFromHand.h>
#include <toaster_msgs/SetEntityPose.h>
#include "tf/transform_datatypes.h"
#include "std_msgs/String.h"

//tf
#include <tf/transform_broadcaster.h>

//Utility
#include "pdg/utility/EntityUtility.h"

#include "pdg/readers/MorseHumanReader.h"
#include "pdg/readers/MocapHumanReader.h"
#include "pdg/readers/AdreamMocapHumanReader.h"
#include "pdg/readers/GroupHumanReader.h"
#include "pdg/readers/ToasterSimuHumanReader.h"

#include "pdg/readers/Pr2RobotReader.h"
#include "pdg/readers/SpencerRobotReader.h"
#include "pdg/readers/ToasterSimuRobotReader.h"

#include "pdg/readers/ArObjectReader.h"
#include "pdg/readers/GazeboObjectReader.h"
#include "pdg/readers/ToasterSimuObjectReader.h"
#include "pdg/readers/OM2MObjectReader.h"
#include "pdg/readers/MocapObjectReader.h"

#include "pdg/types.h"

struct fullConfig_t fullConfig;

// Stream to activate
MorseHumanReader morseHumanRd(fullConfig.Human_);
//NiutHumanReader niutHumanRd;
GroupHumanReader groupHumanRd(false);
MocapHumanReader mocapHumanRd(false);
AdreamMocapHumanReader adreamMocapHumanRd(false);
ToasterSimuHumanReader toasterSimuHumanRd(false);

Pr2RobotReader pr2RobotRd(fullConfig.Robot_);
SpencerRobotReader spencerRobotRd;
ToasterSimuRobotReader toasterSimuRobotRd(fullConfig.Robot_);

ToasterSimuObjectReader toasterSimuObjectRd;
ArObjectReader arObjectRd;
OM2MObjectReader om2mObjectRd;
GazeboObjectReader gazeboRd;
MocapObjectReader mocapObjectRd;

struct objectIn_t objectIn;

//Used to change position of an entity
Entity newPoseEnt_("");

//////////////
// Services //
//////////////

bool addStream(toaster_msgs::AddStream::Request &req,
        toaster_msgs::AddStream::Response & res) {

    morseHumanRd.setActivation(req.morseHuman);
    //niutHumanRd.setActivation(req.niutHuman);
    groupHumanRd.setActivation(req.groupHuman);
    mocapHumanRd.setActivation(req.mocapHuman);
    adreamMocapHumanRd.setActivation(req.adreamMocapHuman);
    toasterSimuHumanRd.setActivation(req.toasterSimuHuman);

    pr2RobotRd.setActivation(req.pr2Robot);
    spencerRobotRd.setActivation(req.spencerRobot);
    toasterSimuRobotRd.setActivation(req.toasterSimuRobot);

    toasterSimuObjectRd.setActivation(req.toasterSimuObject);
    arObjectRd.setActivation(req.arObject);
    om2mObjectRd.setActivation(req.om2mObject);
    gazeboRd.setActivation(req.gazeboObject);
    mocapObjectRd.setActivation(req.mocapObject);

    ROS_INFO("[pdg] setting pdg input");

    return true;
}

bool putInHand(toaster_msgs::PutInHand::Request &req,
        toaster_msgs::PutInHand::Response & res) {

    ROS_INFO("[pdg][Request][put_in_hand] we got request to put object %s in "
            "agent %s  joint's %s\n", req.objectId.c_str(), req.agentId.c_str(), req.jointName.c_str());


    if (req.agentId == "") {
        ROS_INFO("[pdg][Request][put_in_hand][WARNING] we didn't find agent %s\n", req.agentId.c_str());
        return false;
    }

    if (req.objectId == "") {
        ROS_INFO("[pdg][Request][put_in_hand][WARNING] we didn't find object %s\n", req.objectId.c_str());
        return false;
    }

    if (req.jointName != "") {
        objectIn.Agent_[req.objectId] = req.agentId;
        objectIn.Hand_[req.objectId] = req.jointName;
        return true;
    } else {
        ROS_INFO("[pdg][Request][put_in_hand][WARNING] joint name is empty %s\n", req.jointName.c_str());
        return false;
    }
}

bool removeFromHand(toaster_msgs::RemoveFromHand::Request &req,
        toaster_msgs::RemoveFromHand::Response & res) {

    ROS_INFO("[pdg][Request][remove_from_hand] we got request to remove object %s \n", req.objectId.c_str());

    if (req.objectId == "") {
        ROS_INFO("[pdg][Request][put_in_hand][WARNING] we didn't find object %s\n", req.objectId.c_str());
        return false;
    }

    objectIn.Agent_.erase(req.objectId);
    objectIn.Hand_.erase(req.objectId);

    return true;
}

bool setEntityPose(toaster_msgs::SetEntityPose::Request &req,
        toaster_msgs::SetEntityPose::Response & res) {

    if (req.id != "") {
        double roll, pitch, yaw;
        newPoseEnt_.setId(req.id);
        newPoseEnt_.position_.set<0>(req.pose.position.x);
        newPoseEnt_.position_.set<1>(req.pose.position.y);
        newPoseEnt_.position_.set<2>(req.pose.position.z);

        tf::Quaternion q;

        tf::quaternionMsgToTF(req.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	     ros::Time now = ros::Time::now();
	     newPoseEnt_.setTime(now.toNSec());

        newPoseEnt_.orientation_[0] = roll;
        newPoseEnt_.orientation_[1] = pitch;
        newPoseEnt_.orientation_[2] = yaw;
        ROS_INFO("[toaster_simu][Request][INFO] request to set entity pose with "
                "id %s successful", req.id.c_str());
        res.answer = true;

    } else {

        ROS_WARN("[toaster_simu][Request] request to set entity pose with "
                "no id specified, sending back response: false");
        res.answer = false;
    }
    return true;
}

//////////////////
// TF broadcast //
//////////////////
void broadcastTfEntity(tf::TransformBroadcaster &tf_br,toaster_msgs::Entity &entity,const std::string &prefix,ros::Time &stamp){
    geometry_msgs::Pose &pose=entity.pose;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.position.x,pose.position.y,pose.position.z));
    transform.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
    tf_br.sendTransform(tf::StampedTransform(transform,stamp,"map",prefix+"/"+entity.id));
}
void broadcastTfAgent(tf::TransformBroadcaster &tf_br,toaster_msgs::Agent &agent,const std::string &prefix,ros::Time &stamp){
    geometry_msgs::Pose &pose=agent.meEntity.pose;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.position.x,pose.position.y,pose.position.z));
    transform.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
    tf_br.sendTransform(tf::StampedTransform(transform,stamp,"map",prefix+agent.meEntity.id+"/base"));

    for(uint i_jnt=0;i_jnt<agent.skeletonJoint.size();++i_jnt){
        broadcastTfEntity(tf_br,agent.skeletonJoint[i_jnt].meEntity,prefix+"/"+agent.meEntity.id,stamp);
    }
}

int main(int argc, char** argv) {
    unsigned int seq = 0;
    ros::init(argc, argv, "pdg");
    ros::NodeHandle node;

    tf::TransformBroadcaster tf_br;

    //Data reading
    vector<HumanReader*> humanReaders;
    groupHumanRd.init(&node, "/spencer/perception/tracked_groups", "/pdg/groupHuman");
    humanReaders.push_back(&groupHumanRd);
    morseHumanRd.init(&node, "/pdg/morseHuman");
    humanReaders.push_back(&morseHumanRd);
    //niutHumanRd.init(&node, "/niut/Human", "/pdg/niutHuman")
    mocapHumanRd.init(&node, "/optitrack_person/tracked_persons", "/pdg/mocapHuman");
    humanReaders.push_back(&mocapHumanRd);
    adreamMocapHumanRd.init(&node, "/optitrack/bodies/Rigid_Body_3", "/optitrack/bodies/Rigid_Body_1", "/optitrack/bodies/Rigid_Body_2", "/pdg/adreamMocapHuman");
    humanReaders.push_back(&adreamMocapHumanRd);
    toasterSimuHumanRd.init(&node, "/pdg/toasterSimuHuman");
    humanReaders.push_back(&toasterSimuHumanRd);

    vector<RobotReader*> robotReaders;
    pr2RobotRd.init(&node, "/pdg/pr2Robot");
    robotReaders.push_back(&pr2RobotRd);
    spencerRobotRd.init(&node, "/pdg/spencerRobot");
    robotReaders.push_back(&spencerRobotRd);
    toasterSimuRobotRd.init(&node, "/pdg/toasterSimuRobot");
    robotReaders.push_back(&toasterSimuRobotRd);

    vector<ObjectReader*> objectReaders;
    arObjectRd.init(&node, "ar_visualization_marker", "/pdg/arObjectReader");
    objectReaders.push_back(&arObjectRd);
    om2mObjectRd.init(&node, "/iot2pdg_updates", "/pdg/OM2MObjectReader");
    objectReaders.push_back(&om2mObjectRd);
    gazeboRd.init(&node, "/gazebo/model_states", "/pdg/gazeboObjectReader");
    objectReaders.push_back(&gazeboRd);
    mocapObjectRd.init(&node, "/optitrack/bodies/Rigid_Body_1", "object", "/pdg/mocapObjectReader");
    objectReaders.push_back(&mocapObjectRd);
    toasterSimuObjectRd.init(&node, "/pdg/toasterSimuObject");
    objectReaders.push_back(&toasterSimuObjectRd);

    //Services
    ros::ServiceServer addStreamServ = node.advertiseService("pdg/manage_stream", addStream);
    ROS_INFO("Ready to manage stream.");

    ros::ServiceServer servicePutInHand = node.advertiseService("pdg/put_in_hand", putInHand);
    ROS_INFO("[Request] Ready to put object in hand.");

    ros::ServiceServer serviceRemoveFromHand = node.advertiseService("pdg/remove_from_hand", removeFromHand);
    ROS_INFO("[Request] Ready to remove object from hand.");

    ros::ServiceServer setEntPose = node.advertiseService("pdg/set_entity_pose", setEntityPose);
    ROS_INFO("Ready to set Entity position.");

    //Data writing
    ros::Publisher object_pub = node.advertise<toaster_msgs::ObjectListStamped>("pdg/objectList", 1000);
    ros::Publisher human_pub = node.advertise<toaster_msgs::HumanListStamped>("pdg/humanList", 1000);
    ros::Publisher robot_pub = node.advertise<toaster_msgs::RobotListStamped>("pdg/robotList", 1000);
    ros::Publisher fact_pub = node.advertise<toaster_msgs::FactList>("pdg/factList", 1000);

    ros::ServiceClient setPoseClient = node.serviceClient<toaster_msgs::SetEntityPose>("/toaster_simu/set_entity_pose", true);
    EntityUtility_setClient(&setPoseClient);

    ros::Rate loop_rate(30);

    tf::TransformListener listener;
    ROS_INFO("[PDG] initializing\n");


    while (node.ok()) {
      toasterList_t list_msg;

        //update data
        morseHumanRd.updateHumans(listener);

        pr2RobotRd.updateRobot(listener);
        spencerRobotRd.updateRobot(listener); // /!\ full config always at false !!

        ///////////////////////////////////////////////////////////////////////

        //////////////////
        // publish data //
        //////////////////

        for(vector<HumanReader*>::iterator it = humanReaders.begin(); it != humanReaders.end(); ++it)
        {
          (*it)->updateEntityPose(newPoseEnt_);
          (*it)->Publish(list_msg);
        }

        for(vector<RobotReader*>::iterator it = robotReaders.begin(); it != robotReaders.end(); ++it)
        {
          (*it)->updateEntityPose(newPoseEnt_);
          (*it)->Publish(list_msg);
        }

        for(vector<ObjectReader*>::iterator it = objectReaders.begin(); it != objectReaders.end(); ++it)
          (*it)->updateEntityPose(newPoseEnt_);

        //do publication for all objects
        ObjectReader tmp;
        tmp.Publish(list_msg, objectIn);

        ////////////////////////////////////////////////////////////////////////

        //header of messages
        seq++;
        list_msg.object_msg.header.stamp = ros::Time::now();
        list_msg.object_msg.header.seq = seq;
        list_msg.object_msg.header.frame_id = "/map";

        list_msg.human_msg.header = list_msg.object_msg.header;
        list_msg.robot_msg.header = list_msg.object_msg.header;

        for(typeof(list_msg.object_msg.objectList.begin()) it=list_msg.object_msg.objectList.begin(); it!= list_msg.object_msg.objectList.end();++it){
            broadcastTfEntity(tf_br,it->meEntity,"",list_msg.object_msg.header.stamp);
        }

        for(typeof(list_msg.human_msg.humanList.begin()) it=list_msg.human_msg.humanList.begin(); it!= list_msg.human_msg.humanList.end();++it){
            broadcastTfAgent(tf_br,it->meAgent,"",list_msg.human_msg.header.stamp);
        }
        for(typeof(list_msg.robot_msg.robotList.begin()) it=list_msg.robot_msg.robotList.begin(); it!= list_msg.robot_msg.robotList.end();++it){
            broadcastTfAgent(tf_br,it->meAgent,"",list_msg.robot_msg.header.stamp);
        }


        //ROS_INFO("%s", msg.data.c_str());

        object_pub.publish(list_msg.object_msg);
        human_pub.publish(list_msg.human_msg);
        robot_pub.publish(list_msg.robot_msg);
        fact_pub.publish(list_msg.fact_msg);

        ros::spinOnce();

        loop_rate.sleep();

    }
    return 0;
}
