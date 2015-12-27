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



std::map<std::string, toaster_msgs::Object> object_map;
std::map<std::string, toaster_msgs::Human> human_map;
std::map<std::string, toaster_msgs::Robot> robot_map;

///////////////////////////
//   Service functions   //
///////////////////////////

bool setEntityPose(toaster_msgs::SetEntityPose::Request &req,
        toaster_msgs::SetEntityPose::Response & res) {

    if (req.id != "") {

        // Is it an object?
        std::map<std::string, toaster_msgs::Object>::const_iterator it = object_map.find(req.id);
        if (it != object_map.end()) {
            toaster_msgs::Object obj;
            obj.meEntity = it->second.meEntity;
            obj.meEntity.positionX = req.x;
            obj.meEntity.positionY = req.y;
            obj.meEntity.positionZ = req.z;
            obj.meEntity.orientationRoll = req.roll;
            obj.meEntity.orientationPitch = req.pitch;
            obj.meEntity.orientationYaw = req.yaw;

            object_map[req.id] = obj;
        } else {
            // Ok, not an object, mb a human?
            std::map<std::string, toaster_msgs::Human>::const_iterator itH = human_map.find(req.id);
            if (itH != human_map.end()) {
                toaster_msgs::Human hum;
                hum.meAgent.meEntity = itH->second.meAgent.meEntity;
                hum.meAgent.meEntity.positionX = req.x;
                hum.meAgent.meEntity.positionY = req.y;
                hum.meAgent.meEntity.positionZ = req.z;
                hum.meAgent.meEntity.orientationRoll = req.roll;
                hum.meAgent.meEntity.orientationPitch = req.pitch;
                hum.meAgent.meEntity.orientationYaw = req.yaw;

                human_map[req.id] = hum;
            } else {
                // Ok, not a human, mb a robot??
                std::map<std::string, toaster_msgs::Robot>::const_iterator itR = robot_map.find(req.id);
                if (itR != robot_map.end()) {
                    toaster_msgs::Robot rob;
                    rob.meAgent.meEntity = itR->second.meAgent.meEntity;
                    rob.meAgent.meEntity.positionX = req.x;
                    rob.meAgent.meEntity.positionY = req.y;
                    rob.meAgent.meEntity.positionZ = req.z;
                    rob.meAgent.meEntity.orientationRoll = req.roll;
                    rob.meAgent.meEntity.orientationPitch = req.pitch;
                    rob.meAgent.meEntity.orientationYaw = req.yaw;

                    robot_map[req.id] = rob;

                    //TODO: add Joint
                } else {
                    ROS_WARN("[toaster_simu][Request] request to set entity pose with "
                            "id %s, we don't know this entity, please add it first", req.id.c_str());
                    res.answer = false;
                    return true;
                }
            }

        }
        ROS_INFO("[toaster_simu][Request][INFO] request to set entity pose with "
                "id %s successful", req.id.c_str());
    } else {
        ROS_WARN("[toaster_simu][Request] request to set entity pose with "
                "no id specified, sending back response: false");
        res.answer = false;
    }
    res.answer = true;
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

            //TODO: add Joint

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

int main(int argc, char** argv) {

    ros::init(argc, argv, "toaster_simu");
    ros::NodeHandle node;


    //Services
    ros::ServiceServer setEntPose = node.advertiseService("toaster_simu/set_entity_pose", setEntityPose);
    ROS_INFO("Ready to set Entity position.");

    ros::ServiceServer serviceAddEnt = node.advertiseService("toaster_simu/add_entity", addEntity);
    ROS_INFO("[Request] Ready to add entities.");

    //Data writing
    ros::Publisher object_pub = node.advertise<toaster_msgs::ObjectList>("pdg/objectList", 1000);
    ros::Publisher human_pub = node.advertise<toaster_msgs::HumanList>("pdg/humanList", 1000);
    ros::Publisher robot_pub = node.advertise<toaster_msgs::RobotList>("pdg/robotList", 1000);



    ros::Rate loop_rate(30);

    ROS_INFO("[toaster_simu] initializing\n");



    while (node.ok()) {

        toaster_msgs::ObjectList objectList_msg;
        toaster_msgs::HumanList humanList_msg;
        toaster_msgs::RobotList robotList_msg;

        for (std::map<std::string, toaster_msgs::Object>::const_iterator it = object_map.begin(); it != object_map.end(); ++it)
            objectList_msg.objectList.push_back(it->second);


        for (std::map<std::string, toaster_msgs::Human>::const_iterator it = human_map.begin(); it != human_map.end(); ++it)
            humanList_msg.humanList.push_back(it->second);


        for (std::map<std::string, toaster_msgs::Robot>::const_iterator it = robot_map.begin(); it != robot_map.end(); ++it)
            robotList_msg.robotList.push_back(it->second);

        object_pub.publish(objectList_msg);
        human_pub.publish(humanList_msg);
        robot_pub.publish(robotList_msg);

        ros::spinOnce();

        loop_rate.sleep();

    }
    return 0;
}
