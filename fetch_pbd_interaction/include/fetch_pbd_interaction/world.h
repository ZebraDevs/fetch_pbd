#ifndef PBD_WORLD
#define PBD_WORLD

#include <vector>
#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/InteractiveMarker.h"
#include "visualization_msgs/InteractiveMarkerControl.h"
#include "visualization_msgs/InteractiveMarkerFeedback.h"
#include <interactive_markers/interactive_marker_server.h>
#include "rail_manipulation_msgs/SegmentedObject.h"
#include "rail_manipulation_msgs/SegmentedObjectList.h"
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include "fetch_pbd_interaction/WorldState.h"
#include "fetch_pbd_interaction/Landmark.h"
#include "fetch_pbd_interaction/GetObjectList.h"
#include "fetch_pbd_interaction/GetMostSimilarObject.h"
#include "fetch_pbd_interaction/GetObjectFromName.h"
#include "fetch_pbd_interaction/GetNearestObject.h"
#include "std_msgs/ColorRGBA.h"
#include "std_srvs/Empty.h"
#include "moveit_msgs/CollisionObject.h"
#include "moveit_msgs/PlanningScene.h"
#include "shape_msgs/SolidPrimitive.h"

// #include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/filters/filter.h"
#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "pcl/common/distances.h"
#include "pcl/common/pca.h"
#include "pcl/filters/filter.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/conditional_euclidean_clustering.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/region_growing_rgb.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/point_types_conversion.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/console/parse.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <algorithm>
#include <math.h>

#include "fetch_pbd_interaction/world_landmark.h"

namespace fetch_pbd_interaction {

class World {

private:
    boost::mutex mutex;
    std::vector<fetch_pbd_interaction::WorldLandmark> objects;
    interactive_markers::InteractiveMarkerServer im_server;
    ros::Publisher add_grasp_pub;
    ros::Publisher world_update_pub;
    ros::Publisher planning_scene_diff_publisher;
    ros::Subscriber table_subscriber;
    ros::Subscriber object_subscriber;
    ros::ServiceClient segmentation_service_client;
    std::string segmented_objects_topic;
    bool has_surface;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    ros::ServiceServer nearest_object_service;
    ros::ServiceServer object_list_service;
    ros::ServiceServer similar_object_service;
    ros::ServiceServer object_from_name_service;
    ros::ServiceServer clear_world_objects_service;
    ros::ServiceServer update_world_service;
    interactive_markers::MenuHandler menu_handler;
    // Two objects must be closer than this to be considered 'the same'.
    float obj_similar_dist_threshold;

    // When adding objects, if they are closer than this they'll replace one
    // another.
    float obj_add_dist_threshold;

    // How close to 'nearest' object something must be to be counted as
    // 'near' it.     
    float obj_nearest_dist_threshold;

    // Landmark distances below this will be clamped to zero.
    float obj_dist_zero_clamp;

    // Scales
    float text_height;
    float surface_height;  // 0.01 == 1cm (i think)
    float text_offset;  // how high objects' labels are above them.

    // colors
    std_msgs::ColorRGBA color_obj; // not yet a param
    std_msgs::ColorRGBA color_surface; // not yet a param
    std_msgs::ColorRGBA color_text; // not yet a param

    // frames
    std::string base_frame;
    ros::Duration marker_duration;

    static visualization_msgs::Marker pc2ToMarker(sensor_msgs::PointCloud2, int index, ros::Duration duration, std::string output_frame);

    static float objectDissimilarity(fetch_pbd_interaction::Landmark obj1, fetch_pbd_interaction::Landmark obj2);

    static float poseDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, bool is_on_table=true);
    
    visualization_msgs::InteractiveMarker getSurfaceMarker(geometry_msgs::Pose pose, geometry_msgs::Vector3 dimensions);

    void worldChanged();

    bool getMostSimilarObjectCallback(fetch_pbd_interaction::GetMostSimilarObject::Request& req,
                     fetch_pbd_interaction::GetMostSimilarObject::Response& res);

    std::vector<fetch_pbd_interaction::Landmark> getObjectList();

    bool getObjectListCallback(fetch_pbd_interaction::GetObjectList::Request& req, 
                      fetch_pbd_interaction::GetObjectList::Response& resp);

    bool updateWorldCallback(fetch_pbd_interaction::GetObjectList::Request& req, 
                      fetch_pbd_interaction::GetObjectList::Response& resp);

    bool getObjectFromNameCallback(fetch_pbd_interaction::GetObjectFromName::Request& req, 
                      fetch_pbd_interaction::GetObjectFromName::Response& resp);

    bool hasObjects();

    void recordObjectPose();

    void objectsUpdateCallback(const rail_manipulation_msgs::SegmentedObjectListConstPtr& msg);
    
    void getBoundingBox(sensor_msgs::PointCloud2 pc2, geometry_msgs::Vector3* dimensions, geometry_msgs::Pose* pose);


    void tablePositionUpdateCallback(const rail_manipulation_msgs::SegmentedObjectPtr& msg);

    bool clearObjectsCallback(std_srvs::Empty::Request& req, 
                      std_srvs::Empty::Response& resp);

    void clearObjects();

    bool getNearestObjectCallback(fetch_pbd_interaction::GetNearestObject::Request& req, 
                      fetch_pbd_interaction::GetNearestObject::Response& resp);

    void markerFeedbackCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void resetObjects();

    bool addNewObject(geometry_msgs::Pose pose, geometry_msgs::Vector3 dimensions, 
                        sensor_msgs::PointCloud2 point_cloud);


    void removeSurfaceMarker();

    visualization_msgs::InteractiveMarker getObjectMarker(int index);

    void addSurfaceToPlanningScene(geometry_msgs::Pose pose, geometry_msgs::Vector3 dimensions);

    void removeSurfaceFromPlanningScene();
  
    void publishTfPose(geometry_msgs::Pose pose, std::string name, std::string parent);

    void removeObject(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void addGrasp(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);


public:
    World(ros::NodeHandle n, ros::NodeHandle pn, const std::string& im_topic, 
              const std::string& add_grasp_topic, const std::string& world_update_topic,
              const std::string& segmentation_service_name, const std::string& segmented_objects_topic_name,
              const std::string& segmented_table_topic_name, const std::string& planning_scene_topic, 
              const float&  obj_similar_distance_threshold, const float&  obj_add_distance_threshold,
              const float&  obj_nearest_distance_threshold, 
              const float& obj_distance_zero_clamp, const float& text_h, 
              const float&  surface_h, const float&  text_off, 
              const std::string& base_frame_name);
    ~World();

    void update();

};

};

#endif

