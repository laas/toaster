#include <ros/ros.h>

// Message generated class
#include <toaster_msgs/Entity.h>
#include <toaster_msgs/Agent.h>
#include <toaster_msgs/Joint.h>
#include <toaster_msgs/Robot.h>
#include <toaster_msgs/Human.h>
#include <toaster_msgs/Object.h>
#include <toaster_msgs/RobotList.h>
#include <toaster_msgs/HumanList.h>
#include <toaster_msgs/ObjectList.h>
#include <boost/algorithm/string/predicate.hpp>


//Services
#include <toaster_msgs/SetEntityPose.h>
#include <toaster_msgs/AddEntity.h>
#include <toaster_msgs/AddAgent.h>

#include "toaster_simu/Keyboard.h"


std::map<std::string, toaster_msgs::Object> object_map;
std::map<std::string, toaster_msgs::Human> human_map;
std::map<std::string, toaster_msgs::Robot> robot_map;

std::string keyboardControlled_ = "";

bool setEntityPose(std::string id, std::string type, std::string ownerId, double x, double y, double z, double roll, double pitch, double yaw) {
    // Is it an object?
    if (boost::iequals(type, "object")) {
        std::map<std::string, toaster_msgs::Object>::const_iterator it = object_map.find(id);
        if (it != object_map.end()) {
            toaster_msgs::Object obj;
            obj.meEntity = it->second.meEntity;
            obj.meEntity.positionX = x;
            obj.meEntity.positionY = y;
            obj.meEntity.positionZ = z;
            obj.meEntity.orientationRoll = roll;
            obj.meEntity.orientationPitch = pitch;
            obj.meEntity.orientationYaw = yaw;

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
            hum.meAgent.meEntity.positionX = x;
            hum.meAgent.meEntity.positionY = y;
            hum.meAgent.meEntity.positionZ = z;
            hum.meAgent.meEntity.orientationRoll = roll;
            hum.meAgent.meEntity.orientationPitch = pitch;
            hum.meAgent.meEntity.orientationYaw = yaw;

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
            rob.meAgent.meEntity.positionX = x;
            rob.meAgent.meEntity.positionY = y;
            rob.meAgent.meEntity.positionZ = z;
            rob.meAgent.meEntity.orientationRoll = roll;
            rob.meAgent.meEntity.orientationPitch = pitch;
            rob.meAgent.meEntity.orientationYaw = yaw;

            robot_map[id] = rob;
        } else {
            return false;
        }
    } else if (boost::iequals(type, "joint")) {
        std::map<std::string, toaster_msgs::Human>::const_iterator itH = human_map.find(ownerId);
        if (itH != human_map.end()) {
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
            joint.meEntity.positionX = x;
            joint.meEntity.positionY = y;
            joint.meEntity.positionZ = z;
            joint.meEntity.orientationRoll = roll;
            joint.meEntity.orientationPitch = pitch;
            joint.meEntity.orientationYaw = yaw;

            hum.meAgent.skeletonJoint[index] = joint;
            human_map[id] = hum;

        } else {
            std::map<std::string, toaster_msgs::Robot>::const_iterator itR = robot_map.find(ownerId);
            if (itR != robot_map.end()) {
                toaster_msgs::Joint joint;

                // find the joint
                int index = -1;
                for (int i = 0; i < itR->second.meAgent.skeletonJoint.size(); i++) {
                    if (boost::iequals(itH->second.meAgent.skeletonJoint[i].meEntity.id, id) )
                        index = i;
                }

                if (index == -1)
                    return false;

                toaster_msgs::Robot rob;
                rob = itR->second;

                // Update position
                joint = itR->second.meAgent.skeletonJoint[index];
                joint.meEntity.positionX = x;
                joint.meEntity.positionY = y;
                joint.meEntity.positionZ = z;
                joint.meEntity.orientationRoll = roll;
                joint.meEntity.orientationPitch = pitch;
                joint.meEntity.orientationYaw = yaw;

                rob.meAgent.skeletonJoint[index] = joint;
                robot_map[id] = rob;

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
        toaster_msgs::Object obj;
        obj.meEntity = it->second.meEntity;
        obj.meEntity.positionX += x;
        obj.meEntity.positionY += y;
        obj.meEntity.positionZ += z;
        obj.meEntity.orientationRoll += roll;
        obj.meEntity.orientationPitch += pitch;
        obj.meEntity.orientationYaw += yaw;

        object_map[id] = obj;
    } else {
        // Ok, not an object, mb a human?
        std::map<std::string, toaster_msgs::Human>::const_iterator itH = human_map.find(id);
        if (itH != human_map.end()) {
            toaster_msgs::Human hum;
            hum.meAgent.meEntity = itH->second.meAgent.meEntity;
            hum.meAgent.meEntity.positionX += x;
            hum.meAgent.meEntity.positionY += y;
            hum.meAgent.meEntity.positionZ += z;
            hum.meAgent.meEntity.orientationRoll += roll;
            hum.meAgent.meEntity.orientationPitch += pitch;
            hum.meAgent.meEntity.orientationYaw += yaw;

            // Update joints
            for (int i = 0; i < itH->second.meAgent.skeletonJoint.size(); i++) {
                toaster_msgs::Joint joint;
                joint = itH->second.meAgent.skeletonJoint[i];
                joint.meEntity.positionX += x;
                joint.meEntity.positionY += y;
                joint.meEntity.positionZ += z;
                joint.meEntity.orientationRoll += roll;
                joint.meEntity.orientationPitch += pitch;
                joint.meEntity.orientationYaw += yaw;

                hum.meAgent.skeletonJoint.push_back(joint);
                hum.meAgent.skeletonNames.push_back(joint.meEntity.name);
            }

            human_map[id] = hum;
        } else {
            // Ok, not a human, mb a robot??
            std::map<std::string, toaster_msgs::Robot>::const_iterator itR = robot_map.find(id);
            if (itR != robot_map.end()) {
                toaster_msgs::Robot rob;
                rob.meAgent.meEntity = itR->second.meAgent.meEntity;
                rob.meAgent.meEntity.positionX += x;
                rob.meAgent.meEntity.positionY += y;
                rob.meAgent.meEntity.positionZ += z;
                rob.meAgent.meEntity.orientationRoll += roll;
                rob.meAgent.meEntity.orientationPitch += pitch;
                rob.meAgent.meEntity.orientationYaw += yaw;

                // Update joints
                for (int i = 0; i < itR->second.meAgent.skeletonJoint.size(); i++) {
                    toaster_msgs::Joint joint;
                    joint = itR->second.meAgent.skeletonJoint[i];
                    joint.meEntity.positionX += x;
                    joint.meEntity.positionY += y;
                    joint.meEntity.positionZ += z;
                    joint.meEntity.orientationRoll += roll;
                    joint.meEntity.orientationPitch += pitch;
                    joint.meEntity.orientationYaw += yaw;

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

        if (setEntityPose(req.id, req.type, req.ownerId, req.x, req.y, req.z, req.roll, req.pitch, req.yaw)) {
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

            object_map[req.id] = obj;

        }// Ok, not an object, mb a human?

        else if (boost::iequals(req.type, "human")) {
            toaster_msgs::Human hum;
            hum.meAgent.meEntity.id = req.id;
            if (req.name == "")
                hum.meAgent.meEntity.name = req.id;
            else
                hum.meAgent.meEntity.name = req.name;

            human_map[req.id] = hum;

        }// Ok, not a human, mb a robot??

        else if (boost::iequals(req.type, "robot")) {
            toaster_msgs::Robot rob;
            rob.meAgent.meEntity.id = req.id;
            if (req.name == "")
                rob.meAgent.meEntity.name = req.id;
            else
                rob.meAgent.meEntity.name = req.name;

            robot_map[req.id] = rob;

        }// So I guess it's a joint...

        else if (boost::iequals(req.type, "joint")) {
            toaster_msgs::Joint joint;
            joint.meEntity.id = req.id;
            if (req.name == "")
                joint.meEntity.name = req.id;
            else
                joint.meEntity.name = req.name;

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
                    "type %s, please use \"object\", \"human\" or \"robot\" as a type", req.type.c_str());
            res.answer = false;
            return true;
        }
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

    ros::ServiceServer serviceSetKeyb = node.advertiseService("toaster_simu/set_entity_keyboard", setEntityKeyboard);
    ROS_INFO("[Request] Ready to control entity with keyboard.");

    //Data writing
    ros::Publisher object_pub = node.advertise<toaster_msgs::ObjectList>("pdg/objectList", 1000);
    ros::Publisher human_pub = node.advertise<toaster_msgs::HumanList>("pdg/humanList", 1000);
    ros::Publisher robot_pub = node.advertise<toaster_msgs::RobotList>("pdg/robotList", 1000);

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

        toaster_msgs::ObjectList objectList_msg;
        toaster_msgs::HumanList humanList_msg;
        toaster_msgs::RobotList robotList_msg;



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
