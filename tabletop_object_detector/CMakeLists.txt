cmake_minimum_required(VERSION 2.8.3)
project(tabletop_object_detector)

find_package(catkin REQUIRED COMPONENTS 
	geometry_msgs
	household_objects_database_msgs
	message_generation
	pcl_conversions
	pcl_ros
	roscpp
	sensor_msgs
	shape_msgs
	std_msgs
	tf
	visualization_msgs
)

add_message_files(
	FILES 
	Table.msg
	TabletopDetectionResult.msg)

add_service_files(
	FILES
	AddModelExclusion.srv
	ClearExclusionsList.srv
	NegateExclusions.srv
	SegmentObjectInHand.srv
	TabletopDetection.srv
	TabletopObjectRecognition.srv
	TabletopSegmentation.srv
)

include_directories(include ${catkin_INCLUDE_DIRS})

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	shape_msgs
	household_objects_database_msgs
	sensor_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime
               INCLUDE_DIRS include)

add_executable(tabletop_segmentation src/tabletop_segmentation.cpp src/marker_generator.cpp)
add_dependencies(tabletop_segmentation tabletop_object_detector_generate_messages_cpp)
# add_dependencies(your_program ${tabletop_segmentation_EXPORTED_TARGETS})
target_link_libraries(tabletop_segmentation
	${catkin_LIBRARIES}
)
