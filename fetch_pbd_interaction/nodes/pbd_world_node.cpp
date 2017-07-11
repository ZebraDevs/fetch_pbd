#include "fetch_pbd_interaction/world.h"

using fetch_pbd_interaction::World;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pbd_world_node");
  ros::NodeHandle n, pn("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  std::string im_topic;
  std::string add_grasp_topic;
  std::string world_update_topic;
  std::string segmentation_service_name;
  std::string segmented_objects_topic_name;
  std::string segmented_table_topic_name;
  std::string planning_scene_topic;
  float obj_similar_dist_threshold;
  float obj_add_dist_threshold;
  float obj_nearest_dist_threshold;
  float obj_dist_zero_clamp;
  float text_height;
  float surface_height;
  float text_offset;
  std::string base_frame;

  pn.getParam("interactive_marker_topic", im_topic);
  pn.getParam("add_grasp_topic", add_grasp_topic);
  pn.getParam("world_update_topic", world_update_topic);
  pn.getParam("segmentation_service", segmentation_service_name);
  pn.getParam("segmented_objects_topic", segmented_objects_topic_name);
  pn.getParam("segmented_table_topic", segmented_table_topic_name);
  pn.getParam("planning_scene_topic", planning_scene_topic);
  pn.getParam("object_similar_distance_threshold", obj_similar_dist_threshold);
  pn.getParam("object_add_distance_threshold", obj_add_dist_threshold);
  pn.getParam("object_nearest_distance_threshold", obj_nearest_dist_threshold);
  pn.getParam("object_distance_zero_clamp", obj_dist_zero_clamp);
  pn.getParam("text_height", text_height);
  pn.getParam("surface_height", surface_height);
  pn.getParam("text_offset", text_offset);
  pn.getParam("base_frame", base_frame);

  World world(n, pn, im_topic, 
              add_grasp_topic, world_update_topic,
              segmentation_service_name, segmented_objects_topic_name, segmented_table_topic_name,
              planning_scene_topic, obj_similar_dist_threshold, obj_add_dist_threshold,
              obj_nearest_dist_threshold, 
              obj_dist_zero_clamp, text_height, surface_height, text_offset, base_frame);
  ros::Rate r(10);
  while (ros::ok())
  {
    world.update();
    r.sleep();
  }

  return 0;
}
