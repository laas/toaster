#include "visualizer.h"

Visualizer::Visualizer(ros::NodeHandle* node) : node_(node)
{
  nameList_ = std::vector<std::string>();
  idCpt_ = 1;
  printNames_ = true;

  objectNameScale_ = 0.2;
  humanNameScale_ = 0.3;
  robotNameScale_ = 0.3;
  areaNameScale_ = 0.3;

  ros::ServiceServer switch_name_print;
  switch_name_print = node_->advertiseService("toaster_visualizer/switch_name_print", &Visualizer::switchNamePrint, this);

  ros::ServiceServer name_scale;
  name_scale = node_->advertiseService("toaster_visualizer/scale_name", &Visualizer::scaleName, this);
}

bool Visualizer::switchNamePrint(toaster_msgs::Empty::Request &req, toaster_msgs::Empty::Response &res) {
    ROS_INFO("[toaster_visu] switching name print");
    printNames_ = !printNames_;

    return true;
}

bool Visualizer::scaleName(toaster_msgs::Scale::Request &req, toaster_msgs::Scale::Response &res) {
    ROS_INFO("[toaster_visu] scaling printed names");

    objectNameScale_ = req.objectScale;
    humanNameScale_ = req.humanScale;
    robotNameScale_ = req.robotScale;
    areaNameScale_ = req.areaScale;

    return true;
}

int Visualizer::id_generator(std::string name) {
    if (std::find(nameList_.begin(), nameList_.end(), name) != nameList_.end()) {
        return std::find(nameList_.begin(), nameList_.end(), name) - nameList_.begin() + 1;
    } else {
        nameList_.push_back(name);
        return idCpt_++;
    }
}
