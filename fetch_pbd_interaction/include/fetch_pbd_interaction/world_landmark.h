#ifndef PBD_WORLD_LANDMARK
#define PBD_WORLD_LANDMARK


// System includes
#include <sstream>

// ROS builtins
#include "ros/ros.h"
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/Pose.h>
#include "visualization_msgs/InteractiveMarkerFeedback.h"
#include "visualization_msgs/InteractiveMarker.h"

// Local
#include "fetch_pbd_interaction/Landmark.h"

namespace fetch_pbd_interaction {

class World;

class WorldLandmark {

private:
	//void (World::*_removeObjectCallback)(std::string name);
	//void (World::*_generateGraspsCallback)(std::string name);
	sensor_msgs::PointCloud2 point_cloud;
	int index;
	std::string assigned_name;
    bool is_removed;
    fetch_pbd_interaction::World *world;

public:
	WorldLandmark(geometry_msgs::Pose pose, int ind, geometry_msgs::Vector3 dims,
                 sensor_msgs::PointCloud2 pc); // World *world, void (World::*removeObject)(std::string name), void (World::*generateGrasps)(std::string name));
	~WorldLandmark();
	std::string getName();
	void remove(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	void getGrasps(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    visualization_msgs::InteractiveMarker int_marker;
	interactive_markers::MenuHandler menu_handler;

	fetch_pbd_interaction::Landmark object;


};

};

#endif
