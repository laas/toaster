/* 
 * File:   GroupHumanReader.cpp
 * Author: gmilliez
 * 
 * Created on April 21, 2015, 12:51 AM
 */

#include "pdg/GroupHumanReader.h"

GroupHumanReader::GroupHumanReader(ros::NodeHandle& node, std::string topic) {
    std::cout << "Initializing GroupHumanReader" << std::endl;
    // ******************************************
    // Starts listening to the joint_states
    sub_ = node.subscribe(topic, 1, &GroupHumanReader::groupTrackCallback, this);
    fullHuman_ = false;
    std::cout << "Done\n";
}

/*
  Gets data from a TrackedPersons msg in the human map. This msg contains a list of agens with
  their positions and orientations.
 */
void GroupHumanReader::groupTrackCallback(const spencer_tracking_msgs::TrackedGroups::ConstPtr& msg) {
    tf::StampedTransform transform;
    ros::Time now = ros::Time::now();


    try {
        std::string frame;
        frame = msg->header.frame_id;

        //transform from the groupTrack frame to map
        listener_.waitForTransform("/map", frame,
                msg->header.stamp, ros::Duration(3.0));
        listener_.lookupTransform("/map", frame,
                msg->header.stamp, transform);

        //for every group present in the tracking message
        for (int i = 0; i < msg->groups.size(); i++) {
            spencer_tracking_msgs::TrackedGroup group = msg->groups[i];
            int humId = group.group_id;
            //create a new human with the same id as the message
            Human* curHuman = new Human(10000 + humId);

            //get the pose of the agent in the groupTrack frame and transform it to the map frame
            geometry_msgs::PoseStamped groupTrackPose, mapPose;
            //geometry_msgs::PoseStamped optitrackPose, mapPose;

            groupTrackPose.pose.position = group.centerOfGravity.pose.position;
            groupTrackPose.pose.orientation = group.centerOfGravity.pose.orientation;
            groupTrackPose.header.stamp = msg->header.stamp;
            groupTrackPose.header.frame_id = frame;
            listener_.transformPose("/map", groupTrackPose, mapPose);

            //set human position
            bg::model::point<double, 3, bg::cs::cartesian> humanPosition;
            humanPosition.set<0>(mapPose.pose.position.x);
            humanPosition.set<1>(mapPose.pose.position.y);
            humanPosition.set<2>(mapPose.pose.position.z);

            //set the human orientation
            std::vector<double> humanOrientation;

            //transform the pose message
            humanOrientation.push_back(0.0);
            humanOrientation.push_back(0.0);
            humanOrientation.push_back(tf::getYaw(mapPose.pose.orientation));

            //put the data in the human
            curHuman->setOrientation(humanOrientation);
            curHuman->setPosition(humanPosition);
            curHuman->setTime(now.toNSec());

            lastConfig_[humId] = curHuman;
        }
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());


    }
}