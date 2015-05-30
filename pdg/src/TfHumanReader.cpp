/*
 * TfHumanReader.cpp
 *
 *  Created on: May 30, 2015
 *      Author: mfiore
 */

#include <pdg/TfHumanReader.h>



TfHumanReader::TfHumanReader(ros::NodeHandle& node) {
    std::cout << "Initializing TfHumanReader" << std::endl;
    // ******************************************
    // Starts listening to the joint_states
    fullHuman_ = false;


    std::cout << "Done\n";
}

/*
  Gets data from a TrackedPersons msg in the human map. This msg contains a list of agens with
  their positions and orientations.
 */
void TfHumanReader::readTf() {
	tf::StampedTransform transform;
	ros::Time now = ros::Time::now();
	Human* curHuman;



		try {
			listener_.waitForTransform("/map","person_1",ros::Time(0),ros::Duration(3.0));
			listener_.lookupTransform("/map", "person_1",ros::Time(0), transform);
            int humId=humanIdOffset_;
			//set human position
            bg::model::point<double, 3, bg::cs::cartesian> humanPosition;
			humanPosition.set<0>(transform.getOrigin().getX());
			humanPosition.set<1>(transform.getOrigin().getY());
			humanPosition.set<2>(transform.getOrigin().getZ());
			//set the human orientation
			std::vector<double> humanOrientation;
			//transform the pose message
			humanOrientation.push_back(0.0);
			humanOrientation.push_back(0.0);
			humanOrientation.push_back(tf::getYaw(transform.getRotation()));
			//put the data in the human
			curHuman->setOrientation(humanOrientation);
			curHuman->setPosition(humanPosition);
			curHuman->setTime(now.toNSec());
			lastConfig_[humId] = curHuman;

		} catch (tf::TransformException ex) {
			ROS_ERROR("%s", ex.what());





	}

}


