#include "fetch_pbd_interaction/world_landmark.h"
#include "fetch_pbd_interaction/world.h"

namespace fetch_pbd_interaction {

WorldLandmark::WorldLandmark(geometry_msgs::Pose pose, int ind, geometry_msgs::Vector3 dimensions,
                 sensor_msgs::PointCloud2 pc){// World *world, void (World::*removeObject)(std::string name), void (World::*generateGrasps)(std::string name)){
  
  // _removeObjectCallback = removeObject;
  // _generateGraspsCallback = generateGrasps;
  point_cloud = pc;

  assigned_name = "";

  index = ind;

  object = fetch_pbd_interaction::Landmark();
  object.name = getName();
  object.pose = pose;
  object.dimensions = dimensions;
  object.point_cloud = point_cloud;
  // menu_handler = interactive_markers::MenuHandler();
  int_marker = visualization_msgs::InteractiveMarker();
  is_removed = false;
  // this->world = world;
  // menu_handler.insert("Remove from scene", &WorldLandmark::remove, this);
  // menu_handler.insert("Generate grasps for object", &WorldLandmark::getGrasps, this);
  // menu_handler.insert("Remove from scene", boost::bind(&WorldLandmark::remove, this, _1));
  // menu_handler.insert("Generate grasps for object", boost::bind(&WorldLandmark::getGrasps, this, _1));

}

WorldLandmark::~WorldLandmark() {}

std::string WorldLandmark::getName(){
  if (assigned_name == "") {
    std::stringstream sstm;
    sstm << "thing " << index;
    return sstm.str();
  }
  else {
    return assigned_name;
  } 
}
void WorldLandmark::remove(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  ROS_INFO("Will remove object"); //, getName().c_str());
  // ((world)->*_removeObjectCallback)(object.name);

}
void WorldLandmark::getGrasps(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  ROS_INFO("Generating grasps for object"); //: %s", getName().c_str());
  // ((world)->*_generateGraspsCallback)(object.name);
}


}