#include <ros/ros.h>
#include "tf/transform_datatypes.h"

// Message generated class
#include <toaster_msgs/Entity.h>
#include <toaster_msgs/Agent.h>
#include <toaster_msgs/Joint.h>
#include <toaster_msgs/Robot.h>
#include <toaster_msgs/Human.h>
#include <toaster_msgs/Object.h>
#include <toaster_msgs/RobotListStamped.h>
#include <toaster_msgs/HumanListStamped.h>
#include <toaster_msgs/ObjectListStamped.h>
#include <boost/algorithm/string/predicate.hpp>


//Services
#include <toaster_msgs/SetEntityPose.h>
#include <toaster_msgs/AddEntity.h>
#include <toaster_msgs/RemoveEntity.h>
#include <toaster_msgs/AddAgent.h>

#include "toaster_simu/Keyboard.h"


std::map<std::string, toaster_msgs::Object> object_map;
std::map<std::string, toaster_msgs::Human> human_map;
std::map<std::string, toaster_msgs::Robot> robot_map;

std::string keyboardControlled_ = "";

bool setEntityPose(std::string id, std::string type, std::string ownerId, geometry_msgs::Pose pose) {
    // Is it an object?
    if (boost::iequals(type, "object")) {
        std::map<std::string, toaster_msgs::Object>::const_iterator it = object_map.find(id);
        if (it != object_map.end()) {
            toaster_msgs::Object obj;
            obj.meEntity = it->second.meEntity;
            obj.meEntity.pose = pose;

            object_map[id] = obj;
        } else {
            return false;
        }

        // Ok, not an object, mb a human?
    } else if (boost::iequals(type, "human")) {
        std::map<std::string, toaster_msgs::Human>::const_iterator itH = human_map.find(id);
        if (itH != human_map.end()) {
            toaster_msgs::Human hum;
            hum.meAgent.meEntity = itH->second.meAgent.meEntity;
            hum.meAgent.meEntity.pose = pose;

            // TODO: update joints too

            human_map[id] = hum;
        } else {
            return false;
        }

        // Ok, not a human, mb a robot??
    } else if (boost::iequals(type, "robot")) {
        std::map<std::string, toaster_msgs::Robot>::const_iterator itR = robot_map.find(id);
        if (itR != robot_map.end()) {
            toaster_msgs::Robot rob;
            rob.meAgent.meEntity = itR->second.meAgent.meEntity;
            rob.meAgent.meEntity.pose = pose;

            robot_map[id] = rob;
        } else {
            return false;
        }
    } else if (boost::iequals(type, "joint")) {
        ROS_DEBUG("Changing joint position");
        std::map<std::string, toaster_msgs::Human>::const_iterator itH = human_map.find(ownerId);
        if (itH != human_map.end()) {

            ROS_DEBUG("set joint pose: owner is human");
            toaster_msgs::Joint joint;
            // find the joint
            int index = -1;
            for (int i = 0; i < itH->second.meAgent.skeletonJoint.size(); i++) {
                ROS_INFO("we compare %s and %s", itH->second.meAgent.skeletonJoint[i].meEntity.id.c_str(), id.c_str());
                if (boost::iequals(itH->second.meAgent.skeletonJoint[i].meEntity.id, id)) {
                    ROS_INFO("true");
                    index = i;
                }
            }

            if (index == -1)
                return false;

            toaster_msgs::Human hum;
            hum = itH->second;

            // Update position
            joint = itH->second.meAgent.skeletonJoint[index];
            joint.meEntity.pose = pose;
            hum.meAgent.skeletonJoint[index] = joint;
            human_map[itH->second.meAgent.meEntity.id] = hum;

        } else {
            std::map<std::string, toaster_msgs::Robot>::const_iterator itR = robot_map.find(ownerId);
            if (itR != robot_map.end()) {

                ROS_DEBUG("set joint pose: owner is robot");
                toaster_msgs::Joint joint;

                // find the joint
                int index = -1;
                for (int i = 0; i < itR->second.meAgent.skeletonJoint.size(); i++) {
                    if (boost::iequals(itR->second.meAgent.skeletonJoint[i].meEntity.id, id))
                        index = i;
                }

                if (index == -1)
                    return false;

                toaster_msgs::Robot rob;
                rob = itR->second;

                // Update position
                joint = itR->second.meAgent.skeletonJoint[index];
                joint.meEntity.pose = pose;

                rob.meAgent.skeletonJoint[index] = joint;
                robot_map[itR->second.meAgent.meEntity.id] = rob;

            } else {
                return false;
            }
        }
    }
    return true;
}

bool updateEntityPose(std::string id, double x, double y, double z, double roll, double pitch, double yaw) {
    // Is it an object?
    std::map<std::string, toaster_msgs::Object>::const_iterator it = object_map.find(id);
    if (it != object_map.end()) {
        double rollEnt, pitchEnt, yawEnt;
        toaster_msgs::Object obj;
        obj.meEntity = it->second.meEntity;
        obj.meEntity.pose.position.x += x;
        obj.meEntity.pose.position.y += y;
        obj.meEntity.pose.position.z += z;


        tf::Quaternion q(obj.meEntity.pose.orientation.x, obj.meEntity.pose.orientation.y,
                obj.meEntity.pose.orientation.z, obj.meEntity.pose.orientation.w);
        tf::Matrix3x3(q).getRPY(rollEnt, pitchEnt, yawEnt);

        rollEnt += roll;
        pitchEnt += pitch;
        yawEnt += yaw;

        obj.meEntity.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rollEnt, pitchEnt, yawEnt);

        object_map[id] = obj;
    } else {
        // Ok, not an object, mb a human?
        std::map<std::string, toaster_msgs::Human>::const_iterator itH = human_map.find(id);
        if (itH != human_map.end()) {
            toaster_msgs::Human hum;
            double rollEnt, pitchEnt, yawEnt;
            hum.meAgent.meEntity = itH->second.meAgent.meEntity;
            hum.meAgent.meEntity.pose.position.x += x;
            hum.meAgent.meEntity.pose.position.y += y;
            hum.meAgent.meEntity.pose.position.z += z;

            tf::Quaternion q(hum.meAgent.meEntity.pose.orientation.x, hum.meAgent.meEntity.pose.orientation.y,
                    hum.meAgent.meEntity.pose.orientation.z, hum.meAgent.meEntity.pose.orientation.w);
            tf::Matrix3x3(q).getRPY(rollEnt, pitchEnt, yawEnt);

            rollEnt += roll;
            pitchEnt += pitch;
            yawEnt += yaw;

            hum.meAgent.meEntity.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rollEnt, pitchEnt, yawEnt);

            // Update joints
            for (int i = 0; i < itH->second.meAgent.skeletonJoint.size(); i++) {
                toaster_msgs::Joint joint;
                double rollJnt, pitchJnt, yawJnt;
                joint = itH->second.meAgent.skeletonJoint[i];
                joint.meEntity.pose.position.x += x;
                joint.meEntity.pose.position.y += y;
                joint.meEntity.pose.position.z += z;


                tf::Quaternion q(joint.meEntity.pose.orientation.x, joint.meEntity.pose.orientation.y,
                        joint.meEntity.pose.orientation.z, joint.meEntity.pose.orientation.w);
                tf::Matrix3x3(q).getRPY(rollJnt, pitchJnt, yawJnt);

                rollJnt += roll;
                pitchJnt += pitch;
                yawJnt += yaw;

                joint.meEntity.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rollJnt, pitchJnt, yawJnt);

                hum.meAgent.skeletonJoint.push_back(joint);
                hum.meAgent.skeletonNames.push_back(joint.meEntity.name);
            }

            human_map[id] = hum;
        } else {
            // Ok, not a human, mb a robot??
            std::map<std::string, toaster_msgs::Robot>::const_iterator itR = robot_map.find(id);
            if (itR != robot_map.end()) {
                toaster_msgs::Robot rob;
                double rollEnt, pitchEnt, yawEnt;
                rob.meAgent.meEntity = itR->second.meAgent.meEntity;
                rob.meAgent.meEntity.pose.position.x += x;
                rob.meAgent.meEntity.pose.position.y += y;
                rob.meAgent.meEntity.pose.position.z += z;

                tf::Quaternion q(rob.meAgent.meEntity.pose.orientation.x, rob.meAgent.meEntity.pose.orientation.y,
                        rob.meAgent.meEntity.pose.orientation.z, rob.meAgent.meEntity.pose.orientation.w);
                tf::Matrix3x3(q).getRPY(rollEnt, pitchEnt, yawEnt);

                rollEnt += roll;
                pitchEnt += pitch;
                yawEnt += yaw;

                rob.meAgent.meEntity.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rollEnt, pitchEnt, yawEnt);

                // Update joints
                for (int i = 0; i < itR->second.meAgent.skeletonJoint.size(); i++) {
                    toaster_msgs::Joint joint;

                    double rollJnt, pitchJnt, yawJnt;
                    joint = itR->second.meAgent.skeletonJoint[i];
                    joint.meEntity.pose.position.x += x;
                    joint.meEntity.pose.position.y += y;
                    joint.meEntity.pose.position.z += z;


                    tf::Quaternion q(joint.meEntity.pose.orientation.x, joint.meEntity.pose.orientation.y,
                            joint.meEntity.pose.orientation.z, joint.meEntity.pose.orientation.w);
                    tf::Matrix3x3(q).getRPY(rollJnt, pitchJnt, yawJnt);

                    rollJnt += roll;
                    pitchJnt += pitch;
                    yawJnt += yaw;

                    joint.meEntity.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rollJnt, pitchJnt, yawJnt);

                    rob.meAgent.skeletonJoint.push_back(joint);
                    rob.meAgent.skeletonNames.push_back(joint.meEntity.name);
                }

                robot_map[id] = rob;
            } else
                return false;
        }
    }
    return true;
}




///////////////////////////
//   Service functions   //
///////////////////////////

bool setEntityPose(toaster_msgs::SetEntityPose::Request &req,
        toaster_msgs::SetEntityPose::Response & res) {

    if (req.id != "") {

        if (setEntityPose(req.id, req.type, req.ownerId, req.pose)) {
            ROS_INFO("[toaster_simu][Request][INFO] request to set entity pose with "
                    "id %s successful", req.id.c_str());
            res.answer = true;
        } else {
            ROS_WARN("[toaster_simu][Request] request to set entity pose with "
                    "id %s, we don't know this entity, please add it first", req.id.c_str());
            res.answer = false;
        }
    } else {
        ROS_WARN("[toaster_simu][Request] request to set entity pose with "
                "no id specified, sending back response: false");
        res.answer = false;
    }
    return true;
}

bool addEntity(toaster_msgs::AddEntity::Request &req,
        toaster_msgs::AddEntity::Response & res) {

    ros::Time now = ros::Time::now();

    if (req.type == "" || req.id == "") {
        ROS_WARN("[toaster_simu][Request] to add entity you need to specify "
                "at least the entity type and id");
        res.answer = false;
    } else {
        // Is it an object?
        if (boost::iequals(req.type, "object")) {
            toaster_msgs::Object obj;
            obj.meEntity.id = req.id;
            if (req.name == "")
                obj.meEntity.name = req.id;
            else
                obj.meEntity.name = req.name;

            obj.meEntity.pose.orientation.w = 1.0;
            obj.meEntity.time = now.toNSec();
            object_map[req.id] = obj;

        }// Ok, not an object, mb a human?

        else if (boost::iequals(req.type, "human")) {
            toaster_msgs::Human hum;
            hum.meAgent.meEntity.id = req.id;
            if (req.name == "")
                hum.meAgent.meEntity.name = req.id;
            else
                hum.meAgent.meEntity.name = req.name;

            hum.meAgent.meEntity.pose.orientation.w = 1.0;
            hum.meAgent.meEntity.time = now.toNSec();
            human_map[req.id] = hum;

        }// Ok, not a human, mb a robot??

        else if (boost::iequals(req.type, "robot")) {
            toaster_msgs::Robot rob;
            rob.meAgent.meEntity.id = req.id;
            if (req.name == "")
                rob.meAgent.meEntity.name = req.id;
            else
                rob.meAgent.meEntity.name = req.name;

            rob.meAgent.meEntity.pose.orientation.w = 1.0;
            rob.meAgent.meEntity.time = now.toNSec();
            robot_map[req.id] = rob;

        }// So I guess it's a joint...

        else if (boost::iequals(req.type, "joint")) {
            toaster_msgs::Joint joint;
            joint.meEntity.id = req.id;
            if (req.name == "")
                joint.meEntity.name = req.id;
            else
                joint.meEntity.name = req.name;

            joint.meEntity.pose.orientation.w = 1.0;
            joint.meEntity.time = now.toNSec();
            joint.jointOwner = req.ownerId;


            //Who is the owner?
            std::map<std::string, toaster_msgs::Human>::const_iterator itH = human_map.find(req.ownerId);
            if (itH != human_map.end()) {

                toaster_msgs::Human hum;
                hum = itH->second;

                hum.meAgent.skeletonJoint = itH->second.meAgent.skeletonJoint;
                hum.meAgent.skeletonNames = itH->second.meAgent.skeletonNames;
                hum.meAgent.skeletonNames.push_back(req.name);
                hum.meAgent.skeletonJoint.push_back(joint);
                human_map[req.ownerId] = hum;

            } else {
                // Ok, not a human, mb a robot??
                std::map<std::string, toaster_msgs::Robot>::const_iterator itR = robot_map.find(req.ownerId);
                if (itR != robot_map.end()) {

                    toaster_msgs::Robot rob;
                    rob = itR->second;

                    rob.meAgent.skeletonJoint = itR->second.meAgent.skeletonJoint;
                    rob.meAgent.skeletonNames = itR->second.meAgent.skeletonNames;
                    rob.meAgent.skeletonNames.push_back(req.name);
                    rob.meAgent.skeletonJoint.push_back(joint);
                    robot_map[req.ownerId] = rob;

                } else {
                    ROS_WARN("[toaster_simu][Request] request to set entity pose with "
                            "type %s and owner %s, we don't know this agent...", req.type.c_str(), req.ownerId.c_str());
                    res.answer = false;
                    return true;
                }

            }
        } else {
            ROS_WARN("[toaster_simu][Request] request to set entity pose with "
                    "type %s, please use \"object\", \"human\", \"robot\" or \"joint\"as a type", req.type.c_str());
            res.answer = false;
            return true;
        }
    }

    res.answer = true;
    return true;

}

bool removeEntity(toaster_msgs::RemoveEntity::Request &req,
        toaster_msgs::RemoveEntity::Response & res) {
    if (req.type == "" || req.id == "") {
        ROS_WARN("[toaster_simu][Request] to remove entity you need to specify "
                "the entity type and id");
        res.answer = false;
    } else {
        // Is it an object?
        if (boost::iequals(req.type, "object")) {
            std::map<std::string, toaster_msgs::Object>::iterator it = object_map.find(req.id);
            if (it != object_map.end()) {
                object_map.erase(it);
            } else {
                ROS_WARN("[toaster_simu][Request] We couldn't find the object %s", req.id.c_str());
                res.answer = false;
                return false;
            }

            // Ok, not an object, mb a human?
        } else if (boost::iequals(req.type, "human")) {
            std::map<std::string, toaster_msgs::Human>::iterator itH = human_map.find(req.id);
            if (itH != human_map.end()) {
                human_map.erase(itH);
            } else {
                ROS_WARN("[toaster_simu][Request] We couldn't find the human %s", req.id.c_str());
                res.answer = false;
                return false;
            }

            // Ok, not a human, mb a robot??
        } else if (boost::iequals(req.type, "robot")) {
            std::map<std::string, toaster_msgs::Robot>::iterator itR = robot_map.find(req.id);
            if (itR != robot_map.end()) {
                robot_map.erase(itR);
            } else {
                ROS_WARN("[toaster_simu][Request] We couldn't find the robot %s", req.id.c_str());
                res.answer = false;
                return false;
            }

        } else if (boost::iequals(req.type, "joint")) {
            ROS_DEBUG("Changing joint position");
            std::map<std::string, toaster_msgs::Human>::iterator itH = human_map.find(req.ownerId);
            if (itH != human_map.end()) {

                ROS_DEBUG("remove joint: owner is a human");
                toaster_msgs::Joint joint;

                // find the joint
                int index = -1;
                for (int i = 0; i < itH->second.meAgent.skeletonJoint.size(); i++) {
                    ROS_DEBUG("we compare %s and %s", itH->second.meAgent.skeletonJoint[i].meEntity.id.c_str(), req.id.c_str());
                    if (boost::iequals(itH->second.meAgent.skeletonJoint[i].meEntity.id, req.id)) {
                        index = i;
                    }
                }

                if (index == -1) {
                    ROS_WARN("We found the owner %s but he doesn't have a joint %s", req.ownerId.c_str(), req.id.c_str());
                    res.answer = false;
                    return false;
                }

                itH->second.meAgent.skeletonJoint.erase(itH->second.meAgent.skeletonJoint.begin() + index);
                itH->second.meAgent.skeletonNames.erase(itH->second.meAgent.skeletonNames.begin() + index);


            } else {
                std::map<std::string, toaster_msgs::Robot>::iterator itR = robot_map.find(req.ownerId);
                if (itR != robot_map.end()) {

                    ROS_DEBUG("remove joint owner: is a robot");
                    toaster_msgs::Joint joint;

                    // find the joint
                    int index = -1;
                    for (int i = 0; i < itR->second.meAgent.skeletonJoint.size(); i++) {
                        if (boost::iequals(itR->second.meAgent.skeletonJoint[i].meEntity.id, req.id))
                            index = i;
                    }

                    if (index == -1) {

                        ROS_WARN("We found the owner %s but he doesn't have a joint %s", req.ownerId.c_str(), req.id.c_str());
                        res.answer = false;
                        return false;
                    }

                    itR->second.meAgent.skeletonJoint.erase(itR->second.meAgent.skeletonJoint.begin() + index);
                    itR->second.meAgent.skeletonNames.erase(itR->second.meAgent.skeletonNames.begin() + index);

                } else {
                    ROS_WARN("We did not found the owner %s ", req.ownerId.c_str());
                    res.answer = false;
                    return false;
                }
            }
        }//joint
    }

    res.answer = true;
    return true;
}

bool setEntityKeyboard(toaster_msgs::AddAgent::Request &req,
        toaster_msgs::AddAgent::Response & res) {

    if (req.id != "") {
        keyboardControlled_ = req.id;
        res.answer = true;
    } else {
        ROS_WARN("[toaster_simu][Request] request to set keyboard entity with "
                " no id, please enter an id");
        res.answer = false;
    }
    return true;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "toaster_simu");
    ros::NodeHandle node;


    //Services
    ros::ServiceServer setEntPose = node.advertiseService("toaster_simu/set_entity_pose", setEntityPose);
    ROS_INFO("Ready to set Entity position.");

    ros::ServiceServer serviceAddEnt = node.advertiseService("toaster_simu/add_entity", addEntity);
    ROS_INFO("[Request] Ready to add entities.");


    ros::ServiceServer serviceRmEnt = node.advertiseService("toaster_simu/remove_entity", removeEntity);
    ROS_INFO("[Request] Ready to remove entities.");

    ros::ServiceServer serviceSetKeyb = node.advertiseService("toaster_simu/set_entity_keyboard", setEntityKeyboard);
    ROS_INFO("[Request] Ready to control entity with keyboard.");

    //Data writing
    ros::Publisher object_pub = node.advertise<toaster_msgs::ObjectListStamped>("toaster_simu/objectList", 1000);
    ros::Publisher human_pub = node.advertise<toaster_msgs::HumanListStamped>("toaster_simu/humanList", 1000);
    ros::Publisher robot_pub = node.advertise<toaster_msgs::RobotListStamped>("toaster_simu/robotList", 1000);

    ros::Rate loop_rate(30);

    ROS_INFO("[toaster_simu] initializing\n");

    keyboard::Keyboard kbd;

    uint16_t k;
    uint16_t mod;
    bool pressed, new_event;
    double inc = 0.1;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    while (node.ok()) {

        toaster_msgs::ObjectListStamped objectList_msg;
        toaster_msgs::HumanListStamped humanList_msg;
        toaster_msgs::RobotListStamped robotList_msg;



        if (keyboardControlled_ != "") {

            if (kbd.get_key(new_event, pressed, k, mod))
                if (new_event) {
                    switch (k) {
                        case (SDLK_UP):
                        {
                            if (pressed)
                                x = 1.0;
                            else x = 0.0;
                            break;
                        }
                        case (SDLK_DOWN):
                        {
                            if (pressed)
                                x = -1.0;
                            else x = 0.0;
                            break;
                        }
                        case (SDLK_LEFT):
                        {
                            if (pressed)
                                y = 1.0;
                            else y = 0.0;
                            break;
                        }
                        case (SDLK_RIGHT):
                        {
                            if (pressed)
                                y = -1.0;
                            else y = 0.0;
                            break;
                        }
                        case (SDLK_PAGEUP):
                        {
                            if (pressed)
                                z = 1.0;
                            else z = 0.0;
                            break;
                        }
                        case (SDLK_PAGEDOWN):
                        {
                            if (pressed)
                                z = -1.0;
                            else z = 0.0;
                            break;
                        }
                        case (SDLK_w):
                        {
                            if (pressed)
                                roll = 1.0;
                            else roll = 0.0;
                            break;
                        }
                        case (SDLK_s):
                        {
                            if (pressed)
                                roll = -1.0;
                            else roll = 0.0;
                            break;
                        }
                        case (SDLK_a):
                        {
                            if (pressed)
                                pitch = 1.0;
                            else pitch = 0.0;
                            break;
                        }
                        case (SDLK_d):
                        {
                            if (pressed)
                                pitch = -1.0;
                            else pitch = 0.0;
                            break;
                        }
                        case (SDLK_q):
                        {
                            if (pressed)
                                yaw = 1.0;
                            else yaw = 0.0;
                            break;
                        }
                        case (SDLK_e):
                        {
                            if (pressed)
                                yaw = -1.0;
                            else yaw = 0.0;
                            break;
                        }
                        case (SDLK_KP_MINUS):
                        {
                            inc /= 1.2;
                            break;
                        }
                        case (SDLK_KP_PLUS):
                        {
                            inc *= 1.2;
                            break;
                        }
                    }
                }
            updateEntityPose(keyboardControlled_, x*inc, y*inc, z*inc, roll*inc, pitch*inc, yaw * inc);
        }
        ros::Time now = ros::Time::now();

        //update time
        for (std::map<std::string, toaster_msgs::Object>::const_iterator it = object_map.begin(); it != object_map.end(); ++it) {
            toaster_msgs::Object obj;
            obj.meEntity = it->second.meEntity;
            obj.meEntity.time = now.toNSec();

            object_map[it->second.meEntity.id] = obj;
        }

        for (std::map<std::string, toaster_msgs::Human>::const_iterator it = human_map.begin(); it != human_map.end(); ++it) {
            toaster_msgs::Human hum;
            hum.meAgent = it->second.meAgent;
            hum.meAgent.meEntity.time = now.toNSec();

            human_map[it->second.meAgent.meEntity.id] = hum;
        }

        for (std::map<std::string, toaster_msgs::Robot>::const_iterator it = robot_map.begin(); it != robot_map.end(); ++it) {
            toaster_msgs::Robot rob;
            rob.meAgent = it->second.meAgent;
            rob.meAgent.meEntity.time = now.toNSec();

            robot_map[it->second.meAgent.meEntity.id] = rob;
        }
        //update msg list
        for (std::map<std::string, toaster_msgs::Object>::const_iterator it = object_map.begin(); it != object_map.end(); ++it) {
            objectList_msg.objectList.push_back(it->second);
        }

        for (std::map<std::string, toaster_msgs::Human>::const_iterator it = human_map.begin(); it != human_map.end(); ++it) {
            humanList_msg.humanList.push_back(it->second);
        }

        for (std::map<std::string, toaster_msgs::Robot>::const_iterator it = robot_map.begin(); it != robot_map.end(); ++it) {
            robotList_msg.robotList.push_back(it->second);
        }

        object_pub.publish(objectList_msg);
        human_pub.publish(humanList_msg);
        robot_pub.publish(robotList_msg);

        ros::spinOnce();

        loop_rate.sleep();

    }
    return 0;
}
