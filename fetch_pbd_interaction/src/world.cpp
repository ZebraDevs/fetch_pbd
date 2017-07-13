#include "fetch_pbd_interaction/world.h"

namespace fetch_pbd_interaction {

World::World(ros::NodeHandle n, ros::NodeHandle pn, const std::string& im_topic, 
              const std::string& add_grasp_topic, const std::string& world_update_topic,
              const std::string& segmentation_service_name, const std::string& segmented_objects_topic_name,
              const std::string& segmented_table_topic_name, const std::string& planning_scene_topic, 
              const float&  obj_similar_distance_threshold, const float&  obj_add_distance_threshold,
              const float&  obj_nearest_distance_threshold, 
              const float& obj_distance_zero_clamp, const float& text_h, 
              const float&  surface_h, const float&  text_off, 
              const std::string& base_frame_name)
              : im_server(im_topic){
  // Two objects must be closer than this to be considered 'the same'.
  obj_similar_dist_threshold = obj_similar_distance_threshold;

  // When adding objects, if they are closer than this they'll replace one
  // another.
  obj_add_dist_threshold = obj_add_distance_threshold;

  // How close to 'nearest' object something must be to be counted as
  // 'near' it.
  obj_nearest_dist_threshold = obj_nearest_distance_threshold;

  // Landmark distances below this will be clamped to zero.
  obj_dist_zero_clamp = obj_distance_zero_clamp;

  // Scales
  text_height = text_h;
  surface_height = surface_h;  // 0.01 == 1cm (i think)
  text_offset = text_off;  // how high objects' labels are above them.

  // colors
  color_obj = std_msgs::ColorRGBA();
  color_obj.r = 0.2;
  color_obj.g = 0.8;
  color_obj.b = 0.0;
  color_obj.a = 0.6; // not yet a param
  std_msgs::ColorRGBA color_surface = std_msgs::ColorRGBA(); // not yet a param
  color_surface.r = 0.8;
  color_surface.g = 0.0;
  color_surface.b = 0.4;
  color_surface.a = 0.4; // not yet a param
  std_msgs::ColorRGBA color_text = std_msgs::ColorRGBA(); // not yet a param
  color_text.r = 0.0;
  color_text.g = 0.0;
  color_text.b = 0.0;
  color_text.a = 0.5; // not yet a param

  base_frame = base_frame_name;
  marker_duration = ros::Duration(0.0);
  segmented_objects_topic = segmented_objects_topic_name;
  
  geometry_msgs::Vector3 scale_text = geometry_msgs::Vector3();
  scale_text.z = text_height;

  add_grasp_pub = n.advertise<fetch_pbd_interaction::Landmark>(add_grasp_topic, 100);
  world_update_pub = n.advertise<fetch_pbd_interaction::WorldState>(world_update_topic, 100);
  segmentation_service_client =  n.serviceClient<std_srvs::Empty>(segmentation_service_name);
  nearest_object_service = n.advertiseService("get_nearest_object", &World::getNearestObjectCallback, this);
  object_list_service = n.advertiseService("get_object_list", &World::getObjectListCallback, this);
  similar_object_service = n.advertiseService("get_most_similar_object", &World::getMostSimilarObjectCallback, this);
  object_from_name_service = n.advertiseService("get_object_from_name", &World::getObjectFromNameCallback, this);
  clear_world_objects_service = n.advertiseService("clear_world_objects", &World::clearObjectsCallback, this);
  update_world_service = n.advertiseService("update_world", &World::updateWorldCallback, this);
  // segmented_objects_topic = segmented_objects_topic_name;
  table_subscriber = n.subscribe(segmented_table_topic_name, 1, &World::tablePositionUpdateCallback, this);
  // object_subscriber = n.subscribe(segmented_objects_topic_name, 1, &World::objectsUpdateCallback, this);
  has_surface = false;
  planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>(planning_scene_topic, 1);
  menu_handler = interactive_markers::MenuHandler();
  menu_handler.insert("Remove from scene", boost::bind(&World::removeObject, this, _1));
  menu_handler.insert("Add grasp", boost::bind(&World::addGrasp, this, _1));

}

World::~World() {}

// Public instance methods
void World::update() {
  mutex.lock();
  if (hasObjects()) {
    for (int i = 0; i < objects.size(); i++){
      publishTfPose(objects[i].object.pose,
                    objects[i].getName(),
                    base_frame);
    }
  }
  mutex.unlock();
}

// Private static methods
visualization_msgs::Marker World::pc2ToMarker(sensor_msgs::PointCloud2 pc2, int index, ros::Duration duration, std::string output_frame){
  visualization_msgs::Marker pc2_marker;
  pc2_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  pc2_marker.id = index;
  pc2_marker.lifetime=duration;
  pc2_marker.scale.x = 0.005;
  pc2_marker.scale.y = 0.005;
  pc2_marker.scale.z = 0.005;
  pc2_marker.header.frame_id = pc2.header.frame_id;
  pc2_marker.pose.orientation.w = 1.0;

  pcl::PointCloud<pcl::PointXYZRGB> pc;
  // sensor_msgs::PointCloud2 transformed_cloud_msg;
  // tf::TransformListener tf_listener;
  // tf::StampedTransform transform;
  // tf_listener.waitForTransform(output_frame, pc2.header.frame_id, ros::Time(0), ros::Duration(3.0));
  // ROS_INFO("Got transform for point cloud.");
  // tf_listener.lookupTransform(output_frame, pc2.header.frame_id, ros::Time(0), transform);
  // pcl_ros::transformPointCloud(output_frame, transform, pc2, transformed_cloud_msg);
 
  pcl::fromROSMsg(pc2, pc);
  std::vector<geometry_msgs::Point> points_msgs;
  std::vector<std_msgs::ColorRGBA> colors_msgs;

  for (int i=0; i < pc.points.size(); i++){
    pcl::PointXYZRGB point = pc.points[i];
    geometry_msgs::Point point_msg;
    std_msgs::ColorRGBA color_msg;

    point_msg.x = point.x;
    point_msg.y = point.y;
    point_msg.z = point.z;

    color_msg.r = point.r/255.0;
    color_msg.g = point.g/255.0;
    color_msg.b = point.b/255.0;
    color_msg.a = 1.0;

    points_msgs.push_back(point_msg);
    colors_msgs.push_back(color_msg);
  }

  pc2_marker.colors = colors_msgs;
  pc2_marker.points = points_msgs;

  return pc2_marker;
}

float World::objectDissimilarity(fetch_pbd_interaction::Landmark obj1, fetch_pbd_interaction::Landmark obj2){
  float dis;

  geometry_msgs::Vector3 d1 = obj1.dimensions;
  geometry_msgs::Vector3 d2 = obj2.dimensions;
  dis = sqrt(pow(d1.x - d2.x,2) + pow(d1.y - d2.y,2) + pow(d1.z - d2.z,2));

  return dis;
}

float World::poseDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, bool is_on_table /*=true*/){
  float dist;
  geometry_msgs::Point p1 = pose1.position;
  geometry_msgs::Point p2 = pose2.position;
  if (is_on_table){
    dist = sqrt(pow(p1.x - p2.x,2) + pow(p1.y - p2.y,2));
  }
  else {
    dist = sqrt(pow(p1.x - p2.x,2) + pow(p1.y - p2.y,2) + pow(p1.z - p2.z,2));
  }
  return dist;
}

visualization_msgs::InteractiveMarker World::getSurfaceMarker(geometry_msgs::Pose pose, geometry_msgs::Vector3 dimensions){
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.name = "surface";
  int_marker.header.frame_id = base_frame;
  int_marker.pose = pose;
  int_marker.scale = 1.0;
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.id = 2000;
  marker.lifetime = marker_duration;
  marker.scale = dimensions;
  marker.color = color_surface;


  visualization_msgs::InteractiveMarkerControl button_control;
  button_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  button_control.always_visible = true;
  button_control.markers.push_back(marker);
  int_marker.controls.push_back(button_control);
  return int_marker;
}

// Private instance methods
void World::worldChanged(){
  fetch_pbd_interaction::WorldState new_state;
  new_state.object_list = getObjectList();
  update();
  world_update_pub.publish(new_state);
}

std::vector<fetch_pbd_interaction::Landmark> World::getObjectList(){
  std::vector<fetch_pbd_interaction::Landmark> object_list;
  for (int i=0; i < objects.size(); i++){
    object_list.push_back(objects[i].object);
  }
  if (object_list.size() == 0){
    ROS_WARN("No objects detected");
  }
  return object_list;
}

bool World::getMostSimilarObjectCallback(fetch_pbd_interaction::GetMostSimilarObject::Request& req,
                 fetch_pbd_interaction::GetMostSimilarObject::Response& resp){
  mutex.lock();
  float best_dist = 10000.0; //just a big number
  fetch_pbd_interaction::Landmark chosen_object;
  bool success = false;
  std::vector<fetch_pbd_interaction::Landmark> object_list = getObjectList();
  for (int i=0; i < object_list.size(); i++){
    float dist = World::objectDissimilarity(object_list[i], req.original_object);
    if (dist < best_dist){
      best_dist = dist;
      chosen_object = object_list[i];
      success = true;
    }
  }
  if (!success){
    ROS_INFO("Did not find a similar object.");
    resp.has_similar = false;
  }
  else{
    ROS_INFO("Landmark dissimilarity is --- %f", best_dist);
    if (best_dist > obj_similar_dist_threshold){
      ROS_INFO("Found an object but it is not similar enough.");
      resp.has_similar = false;
    }
    else {
      ROS_INFO("Most similar object is: %s", chosen_object.name.c_str());
      resp.has_similar = true;
      resp.similar_object = chosen_object;
    }
  }
  mutex.unlock();
  return true;
}

bool World::getObjectListCallback(fetch_pbd_interaction::GetObjectList::Request& req, 
                  fetch_pbd_interaction::GetObjectList::Response& resp){
  update();
  resp.object_list = getObjectList();
  return true;
}

bool World::updateWorldCallback(fetch_pbd_interaction::GetObjectList::Request& req, 
                  fetch_pbd_interaction::GetObjectList::Response& resp){
  recordObjectPose();
  resp.object_list = getObjectList();
  return true;
}

bool World::getObjectFromNameCallback(fetch_pbd_interaction::GetObjectFromName::Request& req, 
                  fetch_pbd_interaction::GetObjectFromName::Response& resp){
  std::vector<fetch_pbd_interaction::Landmark> object_list = getObjectList();
  for (int i=0; i < object_list.size(); i++){
    if (object_list[i].name == req.ref_name){
      resp.has_object = true;
      resp.obj = object_list[i];
      return true;
    }
  }
  resp.has_object = false;
  return true;
}

bool World::hasObjects(){
  if (objects.size() > 0){
    return true; 
  }
  else {
    return false;
  }
}

void World::recordObjectPose(){
  ROS_INFO("Waiting for segmentation service");
  try
  {
    std_srvs::Empty srv;
    segmentation_service_client.call(srv);
    rail_manipulation_msgs::SegmentedObjectListConstPtr msg = ros::topic::waitForMessage<rail_manipulation_msgs::SegmentedObjectList>(segmented_objects_topic);
    objectsUpdateCallback(msg);
  }
  catch ( ros::Exception &e )
  {
    ROS_ERROR("Error occured: %s ", e.what());
  }
}

void World::objectsUpdateCallback(const rail_manipulation_msgs::SegmentedObjectListConstPtr& msg){
  // resetObjects();
  for (int i=0; i < objects.size(); i++){
    im_server.erase(objects[i].int_marker.name);
    im_server.applyChanges();
  }
  objects.clear();
  mutex.lock();
  bool added = false;
  for (int i=0; i < msg->objects.size(); i++){
    if (msg->objects[i].point_cloud.height == 0 || msg->objects[i].point_cloud.width == 0){
      ROS_WARN("Empty point cloud");
    }
    sensor_msgs::PointCloud2 pc2 = msg->objects[i].point_cloud;
    geometry_msgs::Vector3 dimensions;
    geometry_msgs::Pose pose;
    getBoundingBox(pc2, &dimensions, &pose);
    added = addNewObject(pose, dimensions, pc2);
  }
  if (!added){
    ROS_WARN("Failed to add object");
  }

  mutex.unlock();
  worldChanged();
}

void World::getBoundingBox(sensor_msgs::PointCloud2 pc2, geometry_msgs::Vector3* dimensions, geometry_msgs::Pose* pose){
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  // sensor_msgs::PointCloud2 transformed_cloud_msg;
  // tf::TransformListener tf_listener;
  // tf::StampedTransform transform;
  // tf_listener.waitForTransform(base_frame, pc2.header.frame_id, ros::Time(0), ros::Duration(3.0));
  // ROS_INFO("Got transform for point cloud.");
  // tf_listener.lookupTransform(base_frame, pc2.header.frame_id, ros::Time(0), transform);
  // pcl_ros::transformPointCloud(base_frame, transform, pc2, transformed_cloud_msg);
 
  pcl::fromROSMsg(pc2, cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));
  for (size_t i = 0; i < projected->points.size(); ++i) {
    pcl::PointXYZRGB& point = projected->at(i);
    point.z = 0;
  }
  // Compute PCA.
  pcl::PCA<pcl::PointXYZRGB> pca(true);
  pca.setInputCloud(projected);
  // Get eigenvectors.
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
  // Because we projected points on the XY plane, we add in the Z vector as the
  // 3rd eigenvector.
  eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));
  // ROS_INFO("eigenvectors z: %f, %f, %f", eigenvectors.col(2)(0), eigenvectors.col(2)(1), eigenvectors.col(2)(2));
  if(eigenvectors.col(2)(2) < 0.0){
    //switch axes
    eigenvectors.col(2)(2) = 1.0;
  }

  Eigen::Quaternionf q1(eigenvectors);
  // Find min/max x and y, based on the points in eigenspace.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr eigen_projected(
      new pcl::PointCloud<pcl::PointXYZRGB>(cloud));
  pca.project(cloud, *eigen_projected);
  pcl::PointXYZRGB eigen_min;
  pcl::PointXYZRGB eigen_max;
  pcl::getMinMax3D(*eigen_projected, eigen_min, eigen_max);
  double x_length = eigen_max.x - eigen_min.x;
  double y_length = eigen_max.y - eigen_min.y;
  // The points in eigenspace all have z values of 0. Get min/max z from the
  // original point cloud data.
  pcl::PointXYZRGB cloud_min;
  pcl::PointXYZRGB cloud_max;
  pcl::getMinMax3D(cloud, cloud_min, cloud_max);
  double z_length = cloud_max.z - cloud_min.z;
  // Compute midpoint, defined as the midpoint between the minimum and maximum
  // points, in x, y, and z directions. The centroid is an average that depends
  // on the density of points, which doesn't give you the geometric center of
  // the point cloud.
  pcl::PointXYZRGB eigen_center;
  eigen_center.x = eigen_min.x + x_length / 2;
  eigen_center.y = eigen_min.y + y_length / 2;
  eigen_center.z = 0;
  pcl::PointXYZRGB center;
  pca.reconstruct(eigen_center, center);
  center.z = z_length / 2 + cloud_min.z;
  // Output midpoint.
  pose->position.x = center.x;
  pose->position.y = center.y;
  pose->position.z = center.z;
  pose->orientation.w = q1.w();
  pose->orientation.x = q1.x();
  pose->orientation.y = q1.y();
  pose->orientation.z = q1.z();
  // Output dimensions.
  dimensions->x = x_length;
  dimensions->y = y_length;
  dimensions->z = z_length;
}

void World::tablePositionUpdateCallback(const rail_manipulation_msgs::SegmentedObjectPtr& msg){
  // resetObjects();
  removeSurfaceMarker();
  // objects.clear();
  if (msg->point_cloud.height == 0 || msg->point_cloud.width == 0){
    ROS_WARN("No table detected");
    has_surface = false;
    return;
  }
  ROS_INFO("Table detected");
  has_surface = true;
  sensor_msgs::PointCloud2 pc2 = msg->point_cloud;
  geometry_msgs::Vector3 dimensions;
  geometry_msgs::Pose pose;
  pose.position.x = msg->center.x;
  pose.position.y = msg->center.y;
  pose.position.z = msg->center.z;
  pose.orientation.w = 1.0;
  dimensions.x = msg->depth;
  dimensions.y = msg->width;
  dimensions.z = msg->height;
  // getBoundingBox(pc2, &dimensions, &pose);
  ROS_INFO("Adding table marker");
  visualization_msgs::InteractiveMarker surface = getSurfaceMarker(pose, dimensions);
  im_server.insert(surface);
  im_server.setCallback(surface.name, boost::bind(&World::markerFeedbackCallback, this, _1));
  im_server.applyChanges();
  ROS_INFO("Adding surface to planning scene");
  addSurfaceToPlanningScene(pose, dimensions);
  ROS_INFO("Added surface to planning scene");
}

bool World::clearObjectsCallback(std_srvs::Empty::Request& req, 
                  std_srvs::Empty::Response& resp){
  clearObjects();
  return true;
}

void World::clearObjects(){
  ROS_INFO("Clearing objects");
  resetObjects();
  removeSurfaceMarker();
  worldChanged();
  removeSurfaceFromPlanningScene();
}

bool World::getNearestObjectCallback(fetch_pbd_interaction::GetNearestObject::Request& req, 
                  fetch_pbd_interaction::GetNearestObject::Response& resp){
  geometry_msgs::PoseStamped arm_pose;
  ROS_INFO("Transform from frame: %s to %s", base_frame.c_str(), req.pose.header.frame_id.c_str());
  tf_listener.waitForTransform(base_frame, req.pose.header.frame_id,
                              ros::Time::now(), ros::Duration(5.0));
  tf_listener.transformPose(base_frame, req.pose, arm_pose);
  std::vector<float> distances;
  for (int i=0; i < objects.size(); i++){
    float dist = poseDistance(objects[i].object.pose, arm_pose.pose);
    distances.push_back(dist);
  }

  if (distances.size() > 0){
    int min = *std::min_element(distances.begin(), distances.end());
    if (min < obj_nearest_dist_threshold){
      for (int i=0; i < distances.size(); i++){
        if (distances[i] == min) {
          resp.has_nearest = true;
          resp.nearest_object = objects[i].object;
          return true;
        }
      }
    }
  }
  resp.has_nearest = false;
  return true;
}

void World::markerFeedbackCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  ;
  //TODO(sarah): why do we need this? 
}

void World::resetObjects(){
  mutex.lock();
  ROS_INFO("Resetting objects");
  for (int i=0; i < objects.size(); i++){
    im_server.erase(objects[i].int_marker.name);
    im_server.applyChanges();
  }
  if (has_surface){
    removeSurfaceMarker();
  }
  im_server.clear();
  im_server.applyChanges();
  objects.clear();
  mutex.unlock();
  worldChanged();
}

bool World::addNewObject(geometry_msgs::Pose pose, geometry_msgs::Vector3 dimensions, 
                          sensor_msgs::PointCloud2 point_cloud){
  for (int i=0; i < objects.size(); i++){
    if (World::poseDistance(objects[i].object.pose, pose) < obj_add_dist_threshold){
      ROS_INFO("Previously detected object at the same location. Will not add this object.");
      return false;
    }
  }
  int n_objects = objects.size();
  objects.push_back(fetch_pbd_interaction::WorldLandmark( 
                    pose, n_objects, dimensions, point_cloud));//,
                    // this,
                    // &fetch_pbd_interaction::World::removeObject, 
                    // &fetch_pbd_interaction::World::generateGrasps));

  visualization_msgs::InteractiveMarker int_marker = getObjectMarker(n_objects);
  objects[n_objects].int_marker = int_marker;
  
  im_server.insert(int_marker);
  im_server.setCallback(int_marker.name, boost::bind(&World::markerFeedbackCallback, this, _1));
  im_server.applyChanges();
  menu_handler.apply(im_server, int_marker.name);
  im_server.applyChanges();

  return true;
}


void World::removeSurfaceMarker(){
  ROS_INFO("Removing surface marker");
  im_server.erase("surface");
  im_server.applyChanges();
  has_surface = false;
}

visualization_msgs::InteractiveMarker World::getObjectMarker(int index){
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.name = objects[index].getName();
  int_marker.header.frame_id = objects[index].object.point_cloud.header.frame_id;
  int_marker.pose.position = objects[index].object.pose.position;
  int_marker.pose.orientation.w = 1.0;
  int_marker.scale = 1.0;
  sensor_msgs::PointCloud2 pc2 = objects[index].object.point_cloud;
  visualization_msgs::Marker marker = World::pc2ToMarker(pc2, index, marker_duration, base_frame);

  visualization_msgs::InteractiveMarkerControl button_control;
  button_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  button_control.always_visible = true;
  button_control.markers.push_back(marker);
  int_marker.controls.push_back(button_control);

  return int_marker;
}

void World::addSurfaceToPlanningScene(geometry_msgs::Pose pose, geometry_msgs::Vector3 dimensions){
  moveit_msgs::CollisionObject collision_object;
  /* The header must contain a valid TF frame*/
  collision_object.header.frame_id = base_frame;
  /* The id of the object */
  collision_object.id = "surface";

  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = dimensions.x;
  primitive.dimensions[1] = dimensions.y;
  primitive.dimensions[2] = dimensions.z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = moveit_msgs::CollisionObject::ADD;
  moveit_msgs::PlanningScene planning_scene_msg;
  planning_scene_msg.world.collision_objects.push_back(collision_object);
  planning_scene_msg.is_diff = true;
  // for (int i=0; i < 5; i++){
  planning_scene_diff_publisher.publish(planning_scene_msg);
  // ros::Duration(0.1).sleep();
  // }
}

void World::removeSurfaceFromPlanningScene(){
  ROS_INFO("Removing the object from the world.");
  moveit_msgs::PlanningScene planning_scene_msg;
  moveit_msgs::CollisionObject remove_object;
  remove_object.id = "surface";
  remove_object.header.frame_id = base_frame;
  remove_object.operation = moveit_msgs::CollisionObject::REMOVE;
  planning_scene_msg.world.collision_objects.push_back(remove_object);
  planning_scene_msg.is_diff = true;
  // for (int i=0; i < 5; i++){
  planning_scene_diff_publisher.publish(planning_scene_msg);
  // ros::Duration(0.1).sleep();
  // }
}

void World::publishTfPose(geometry_msgs::Pose pose, std::string name, std::string parent){
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  transform.setRotation(q);
  if (name == ""){
    ROS_INFO("No name");
  }
  if (parent == ""){
    ROS_INFO("No parent");
  }
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent, name));
}

void World::removeObject(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  std::string to_remove = feedback->marker_name;
  fetch_pbd_interaction::GetObjectFromNameRequest req;
  req.ref_name = to_remove;
  fetch_pbd_interaction::GetObjectFromNameResponse resp;
  getObjectFromNameCallback(req, resp);
  for (int i=0; i < objects.size(); i ++){
    if (objects[i].object.name == resp.obj.name){
      objects.erase(objects.begin() + i );
      im_server.erase(objects[i].int_marker.name);
      im_server.applyChanges();
      break;
    }
  }

}

void World::addGrasp(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  ROS_INFO("Marker menu feedback: Grasp object");
  std::string to_grasp = feedback->marker_name;
  fetch_pbd_interaction::GetObjectFromNameRequest req;
  req.ref_name = to_grasp;
  fetch_pbd_interaction::GetObjectFromNameResponse resp;
  getObjectFromNameCallback(req, resp);
  if (resp.has_object){
    add_grasp_pub.publish(resp.obj);
  }
  else {
    ROS_WARN("Cannot generate grasp. No object matching that name.");
  }
}

}