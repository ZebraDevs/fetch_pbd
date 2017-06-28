'''Everything related to perception of the world'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# System builtins
import threading
from numpy.linalg import norm
from numpy import array
import struct
import ctypes

# ROS builtins
from tf import TransformListener, TransformBroadcaster
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose, PoseStamped
from std_msgs.msg import ColorRGBA, Header
from std_srvs.srv import Empty, EmptyResponse
from visualization_msgs.msg import Marker, InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer
import moveit_commander
from rail_manipulation_msgs.msg import SegmentedObjectList, SegmentedObject
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import ros_numpy

# Local
from fetch_pbd_interaction.msg import WorldState, Landmark
from fetch_pbd_interaction.srv import GetObjectList, GetObjectListResponse, \
                                      GetMostSimilarObject, \
                                      GetMostSimilarObjectResponse, \
                                      GetObjectFromName, \
                                      GetObjectFromNameResponse, \
                                      GetObjectFromNameRequest, \
                                      GetNearestObject, \
                                      GetNearestObjectResponse
from fetch_pbd_interaction.world_landmark import WorldLandmark


# ######################################################################
# Module level constants
# ######################################################################

# Two objects must be closer than this to be considered 'the same'.
OBJ_SIMILAR_DIST_THRESHHOLD = 0.075

# When adding objects, if they are closer than this they'll replace one
# another.
OBJ_ADD_DIST_THRESHHOLD = 0.02

# How close to 'nearest' object something must be to be counted as
# 'near' it.
OBJ_NEAREST_DIST_THRESHHOLD = 0.4

# Landmark distances below this will be clamped to zero.
OBJ_DIST_ZERO_CLAMP = 0.0001

# Scales
SCALE_TEXT = Vector3(0.0, 0.0, 0.03)
SURFACE_HEIGHT = 0.01  # 0.01 == 1cm (I think)
OFFSET_OBJ_TEXT_Z = 0.06  # How high objects' labels are above them.

# Colors
COLOR_OBJ = ColorRGBA(0.2, 0.8, 0.0, 0.6)
COLOR_SURFACE = ColorRGBA(0.8, 0.0, 0.4, 0.4)
COLOR_TEXT = ColorRGBA(0.0, 0.0, 0.0, 0.5)

# Frames
BASE_LINK = 'base_link'

# Time
MARKER_DURATION = rospy.Duration(0.0)


# ######################################################################
# Classes
# ######################################################################

class World:
    '''Everything related to perception of the world'''


    def __init__(self):

        self._tf_listener = TransformListener()
        self._surface = None
        self._lock = threading.Lock()
        self._tf_broadcaster = TransformBroadcaster()
        self._im_server = InteractiveMarkerServer('world_objects')
        self._planning_scene = moveit_commander.PlanningSceneInterface()
        # Type: [WorldLandmark]
        self._objects = []


        rospy.wait_for_service('rail_segmentation/segment')
        self._segmentation_service = rospy.ServiceProxy(
            'rail_segmentation/segment',
            Empty)

        # rospy.Subscriber('rail_segmentation/segmented_objects', SegmentedObjectList, self._tabletop_update)
        rospy.Subscriber('rail_segmentation/segmented_table', SegmentedObject, self._table_update)

        self._world_update_pub = rospy.Publisher('world_update', WorldState,
                                                 queue_size=1)
        rospy.Service('clear_world_objects', Empty, self._clear_objects)
        rospy.Service('get_most_similar_object', GetMostSimilarObject,
                      self._get_most_similar_obj)

        rospy.Service('update_world', GetObjectList, self._update_world)
        rospy.Service('get_object_list', GetObjectList, self._get_obj_list_cb)

        rospy.Service('get_nearest_object', GetNearestObject,
                      self._get_nearest_object)
        rospy.Service('get_object_from_name', GetObjectFromName,
                      self._get_object_from_name)

        self._add_grasp_pub = rospy.Publisher('add_grasp', Landmark, queue_size=1)

        self._clear_all_objects()

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def update(self):
        '''Update function called in a loop.
        '''
        # Visualize the detected object
        self._lock.acquire()
        if self._has_objects():
            for i in range(len(self._objects)):
                self._publish_tf_pose(
                    self._objects[i].object.pose,
                    self._objects[i].get_name(),
                    BASE_LINK
                )

        self._lock.release()

    # ##################################################################
    # Static methods: Internal ("private")
    # ##################################################################
    @staticmethod
    def _pc2_to_points_and_colors(point_cloud2):
        ''' Uses numpy_ros and returns lists of Point and ColorRGBA msgs'''
        points_list = []
        colors_list = []

        points_arr = ros_numpy.numpify(point_cloud2)
        points_and_colors = ros_numpy.point_cloud2.split_rgb_field(points_arr)
        # rospy.loginfo("Shape of points and colors:{}".format(points_and_colors.shape))
        # rospy.loginfo("Size of points and colors:{}".format(points_and_colors.size))
        # rospy.loginfo("Array: {}".format(new_points_arr))
        for i in range(points_and_colors.shape[0]):
            x = points_and_colors[i][0]
            y = points_and_colors[i][1]
            z = points_and_colors[i][2]
            point = Point(x, y, z)
            points_list.append(point)
            # point.x+=0.01
            # points_list.append(point)
            r = float(points_and_colors[i][3]) / 255.0
            g = float(points_and_colors[i][4]) / 255.0 
            b = float(points_and_colors[i][5]) / 255.0
            color = ColorRGBA(r, g, b, 0.9)
            # colors_list.append(color)
            colors_list.append(color)

        return points_list, colors_list

    @staticmethod
    def _object_dissimilarity(obj1, obj2):
        '''Returns distance between two objects.

        Returns:
            float
        '''
        d1 = obj1.dimensions
        d2 = obj2.dimensions
        return norm(array([d1.x, d1.y, d1.z]) - array([d2.x, d2.y, d2.z]))

    @staticmethod
    def _pose_distance(pose1, pose2, is_on_table=True):
        '''Returns distance between two world poses.

        Args:
            pose1 (Pose)
            pose2 (Pose)
            is_on_table (bool, optional): Whether the objects are on the
                table (if so, disregards z-values in computations).

        Returns:
            float
        '''
        if pose1 == [] or pose2 == []:
            return 0.0
        else:
            p1p = pose1.position
            p2p = pose2.position
            if is_on_table:
                arr1 = array([p1p.x, p1p.y])
                arr2 = array([p2p.x, p2p.y])
            else:
                arr1 = array([p1p.x, p1p.y, p1p.z])
                arr2 = array([p2p.x, p2p.y, p2p.z])
            dist = norm(arr1 - arr2)
            if dist < OBJ_DIST_ZERO_CLAMP:
                dist = 0
            return dist

    @staticmethod
    def log_pose(log_fn, pose):
        '''For printing a pose to rosout. We don't do it on one line
        becuase that messes up the indentation with the rest of the log.

        Args:
            log_fn (function(str)): A logging function that takes a
                string as an argument. For example, rospy.loginfo.
            pose (Pose): The pose to log
        '''
        p, o = pose.position, pose.orientation
        log_fn(' - position: (%f, %f, %f)' % (p.x, p.y, p.z))
        log_fn(' - orientation: (%f, %f, %f, %f)' % (o.x, o.y, o.z, o.w))


    @staticmethod
    def _get_mesh_marker(marker, mesh):
        '''Generates and returns a marker from a mesh.

        Args:
            marker (Marker)
            mesh (Mesh)

        Returns:
            Marker
        '''
        marker.type = Marker.TRIANGLE_LIST
        index = 0
        marker.scale = Vector3(1.0, 1.0, 1.0)
        while index + 2 < len(mesh.triangles):
            if (mesh.triangles[index] < len(mesh.vertices)
                    and mesh.triangles[index + 1] < len(mesh.vertices)
                    and mesh.triangles[index + 2] < len(mesh.vertices)):
                marker.points.append(mesh.vertices[mesh.triangles[index]])
                marker.points.append(mesh.vertices[mesh.triangles[index + 1]])
                marker.points.append(mesh.vertices[mesh.triangles[index + 2]])
                index += 3
            else:
                rospy.logerr('Mesh contains invalid triangle!')
                break
        return marker

    @staticmethod
    def _get_surface_marker(pose, dimensions):
        '''Returns a surface marker with provided pose and dimensions.

        Args:
            pose (Pose)
            dimensions  (Vector3)

        Returns:
            InteractiveMarker
        '''
        int_marker = InteractiveMarker()
        int_marker.name = 'surface'
        int_marker.header.frame_id = BASE_LINK
        int_marker.pose = pose
        int_marker.scale = 1
        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True
        object_marker = Marker(
            type=Marker.CUBE,
            id=2000,
            lifetime=MARKER_DURATION,
            scale=dimensions,
            header=Header(),
            color=COLOR_SURFACE,
            pose=Pose(Point(), Quaternion(w=1.0))
        )
        button_control.markers.append(object_marker)
        int_marker.controls.append(button_control)
        return int_marker

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _world_changed(self):
        '''Update markers and publish new WorldState when world changed'''
        new_state = WorldState()
        new_state.object_list = self._get_obj_list()
        self.update()
        self._world_update_pub.publish(new_state)

    def _get_most_similar_obj(self, req):
        '''Finds the most similar object in the world.

        Args:
            req (GetMostSimilarObjectRequest) : Landmark object
        Returns:
            GetMostSimilarObjectResponse : bool has_similar, Landmark object
        '''
        self._lock.acquire()
        resp = GetMostSimilarObjectResponse()
        best_dist = 10000  # Not a constant; an absurdly high number.
        ref_object = req.original_object
        chosen_obj = None
        ref_obj_list = self._get_obj_list()
        for obj in ref_obj_list:
            dist = World._object_dissimilarity(obj, ref_object)
            if dist < best_dist:
                best_dist = dist
                chosen_obj = obj
        if chosen_obj is None:
            rospy.loginfo('Did not find a similar object.')
            resp.has_similar = False
        else:
            rospy.loginfo('Landmark dissimilarity is --- ' + str(best_dist))
            if best_dist > OBJ_SIMILAR_DIST_THRESHHOLD:
                rospy.loginfo('Found some objects, but not similar enough.')
                chosen_obj = None
                resp.has_similar = False
            else:
                rospy.loginfo(
                    'Most similar to new object: ' + str(chosen_obj.name))
                resp.has_similar = True
                resp.similar_object = chosen_obj

        # Regardless, return the "closest object," which may be None.
        self._lock.release()
        return resp

    def _get_obj_list(self):
        '''Function that returns the list of reference frames (Landmarks).

        Returns:
            [Landmark]: List of Landmark (as defined by Landmark.msg), the
                current reference frames.
        '''
        return [w_obj.object for w_obj in self._objects]

    def _get_obj_list_cb(self, req):
        '''Function that returns the list of reference frames (Landmarks).

        Args:
            req (GetObjectListRequest) : Unused

        Returns:
            GetObjectListResponse : [Landmark] List of Landmark
                (as defined by Landmark.msg), the
                current reference frames.
        '''
        self.update()
        return GetObjectListResponse(self._get_obj_list())

    def _update_world(self, req):
        '''Re-observe objects, return updated list of objects

        Args:
            req (GetObjectListRequest) : Unused
        Returns:
            GetObjectListResponse : [Landmark] List of Landmark
                (as defined by Landmark.msg), the
                current reference frames.
        '''
        self._record_object_pose()
        return GetObjectListResponse(self._get_obj_list())

    def _get_object_from_name(self, req):
        '''Return Landmark from its name

        Args:
            req (GetObjectFromNameRequest) : string ref_name
        Returns:
            GetObjectFromNameResponse : bool has_object, Landmark obj
        '''
        resp = GetObjectFromNameResponse()
        obj_list = self._get_obj_list()
        for obj in obj_list:
            if obj.name == req.ref_name:
                resp.has_object = True
                resp.obj = obj
                return resp

        resp.has_object = False
        return resp

    def _has_objects(self):
        '''Returns whetehr there are any objects (reference frames).

        Returns:
            bool
        '''
        return len(self._objects) > 0


    def _record_object_pose(self):
        ''' Function to record poses of objects in world.

        Returns:
            bool : Returns true if able to record world
        '''

        rospy.loginfo("waiting for segmentation service")
        try:
            resp = self._segmentation_service()
            msg = rospy.wait_for_message('rail_segmentation/segmented_objects', SegmentedObjectList)
            self._tabletop_update(msg)
        except rospy.ServiceException, e:
            print "Call to segmentation service failed: %s" % e

    def _tabletop_update(self, msg):
        '''Callback for segmentation updates. 

        Args:
            msg : SegmentedObjectList
        '''
        self._reset_objects()
        self._lock.acquire()


        for obj in msg.objects:
            if obj.point_cloud.height == 0 or obj.point_cloud.width == 0:
                rospy.logwarn('Empty point cloud')
                continue
            pc = obj.point_cloud
            points = pc2.read_points(pc, field_names=['x', 'y', 'z'],
                                     skip_nans=True)
            rospy.loginfo('point_cloud: {}'.format(points))

            x, y, z = points.next()
            [min_x, max_x, min_y, max_y, min_z, max_z] = [x, x, y, y, z, z]
            for x, y, z in points:
                min_x = min(min_x, x)
                min_y = min(min_y, y)
                min_z = min(min_z, z)
                max_x = max(max_x, x)
                max_y = max(max_y, y)
                max_z = max(max_z, z)
            added = self._add_new_object(
                        Pose(Point((min_x + max_x) / 2, (min_y + max_y) / 2,
                        (min_z + max_z) / 2), Quaternion(0, 0, 0, 1)),
                        Point(max_x - min_x, max_y - min_y, max_z - min_z),
                        False, pc)
            if not added:
                rospy.loginfo("Failed to add object")
            else:
                rospy.loginfo("Object added?")
                rospy.loginfo("World objects: {}".format(self._objects))
        self._lock.release()
        self._world_changed()

    def _table_update(self, msg):
        '''Callback for table position updates

        Args:
            msg: SegmentedObject
        '''
        # add the table
        if msg.point_cloud.height == 0 or msg.point_cloud.width == 0:
            rospy.logwarn('No table detected')
            return

        pc = msg.point_cloud
        points = pc2.read_points(pc, field_names=['x', 'y', 'z'],
                                 skip_nans=True)
        rospy.loginfo('point_cloud: {}'.format(points))

        x, y, z = points.next()
        [min_x, max_x, min_y, max_y, min_z, max_z] = [x, x, y, y, z, z]
        for x, y, z in points:
            min_x = min(min_x, x)
            min_y = min(min_y, y)
            min_z = min(min_z, z)
            max_x = max(max_x, x)
            max_y = max(max_y, y)
            max_z = max(max_z, z)
        depth = max_x - min_x
        width = max_y - min_y

        position = msg.center
        dimensions = Vector3(depth, width, 0.01)
        pose = Pose()
        pose.position = position
        pose.orientation.w = 1.0
        self._surface = World._get_surface_marker(pose, dimensions)
        self._im_server.insert(self._surface,
                               self._marker_feedback_cb)
        self._im_server.applyChanges()
        # For some reason that defies logic, it's best to add and remove
        # planning scene objects in a loop.
        # It sometimes doesn't work the first time.
        threading.Thread(group=None,
                     target=self._add_surface_to_planning_scene,
                     args=(pose, dimensions),
                     name='session_state_publish_thread').start()


    def _clear_objects(self, req):
        '''Responds to service call to clear objects

        Args:
            req (EmptyRequest): Unused
        Returns:
            EmptyResponse
        '''
        self._clear_all_objects()
        return EmptyResponse()

    def _clear_all_objects(self):
        '''Removes all objects from the world.'''
        self._reset_objects()
        self._remove_surface()
        self._world_changed()
        threading.Thread(group=None,
                         target=self._remove_surface_from_planning_scene,
                         name='session_state_publish_thread').start()

    def _remove_surface_from_planning_scene(self):
        for i in range(5):
            self._planning_scene.remove_world_object("surface")
            rospy.sleep(0.1)

    def _add_surface_to_planning_scene(self, pose, dimensions):
        for i in range(10):
                self._planning_scene.add_box(
                    "surface",
                    PoseStamped(Header(frame_id=BASE_LINK), pose),
                    (dimensions.x, dimensions.y, dimensions.z))
                rospy.sleep(0.1)

    def _get_nearest_object(self, req):
        '''Returns the nearest object, if one exists.

        Args:
            req (GetNearestObjectRequest): pose (PoseStamped) End-effector pose

        Returns:
            GetNearestObjectResponse :
                bool :  has_nearest
                Landmark : the nearest object (if it
                is close enough), or None if there were none close
                enough.
        '''
        # First, find which object is the closest.
        resp = GetNearestObjectResponse()
        arm_pose = self._tf_listener.transformPose('base_link', req.pose)
        distances = []
        for wobj in self._objects:
            dist = self._pose_distance(wobj.object.pose, arm_pose.pose)
            distances.append(dist)

        # Then, see if the closest is actually below our threshhold for
        # a 'closest object.'
        if len(distances) > 0:
            if min(distances) < OBJ_NEAREST_DIST_THRESHHOLD:
                chosen = distances.index(min(distances))
                resp.has_nearest = True
                resp.nearest_object = self._objects[chosen].object
                return resp

        # We didn't have any objects or none were close enough.
        resp.has_nearest = False
        return resp

    def _marker_feedback_cb(self, feedback):
        '''Callback for when feedback from a marker is received.

        Args:
            feedback (InteractiveMarkerFeedback)
        '''
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo('Clicked on object ' + str(feedback.marker_name))
            rospy.loginfo('Number of objects ' + str(len(self._objects)))
        else:
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            rospy.logdebug('Unknown event: ' + str(feedback.event_type))

    def _reset_objects(self):
        '''Removes all objects.'''
        self._lock.acquire()
        for wobj in self._objects:
            self._im_server.erase(wobj.int_marker.name)
            self._im_server.applyChanges()
        if self._surface is not None:
            self._remove_surface()
        self._im_server.clear()
        self._im_server.applyChanges()
        self._objects = []
        self._lock.release()
        self._world_changed()

    def _add_new_object(self, pose, dimensions, is_recognized, point_cloud=None, mesh=None):
        '''Maybe add a new object with the specified properties to our
        object list.

        It might not be added if too similar of an object already
        exists (and has been added).

        Args:
            pose (Pose)
            dimensions (Vector3)
            is_recognized (bool)
            mesh (Mesh, optional): A mesh, if it exists. Default is
                None.

        Returns:
            bool: Whether the object was actually added.
        '''
        # to_remove = None
        if is_recognized:
            # TODO(mbforbes): Re-implement object recognition or remove
            # this dead code.
            return False
            # # Check if there is already an object
            # for i in range(len(self._objects)):
            #     distance = World._pose_distance(
            #         self._objects[i].object.pose, pose)
            #     if distance < OBJ_ADD_DIST_THRESHHOLD:
            #         if self._objects[i].is_recognized:
            #             rospy.loginfo(
            #                 'Previously recognized object at the same ' +
            #                 'location, will not add this object.')
            #             return False
            #         else:
            #             rospy.loginfo(
            #                 'Previously unrecognized object at the same ' +
            #                 'location, will replace it with the recognized '+
            #                 'object.')
            #             to_remove = i
            #             break

            # # Remove any duplicate objects.
            # if to_remove is not None:
            #     self._remove_object(to_remove)

            # # Actually add the object.
            # self._add_new_object_internal(
            #     pose, dimensions, is_recognized, mesh)
            # return True
        else:
            # Whether whether we already have an object at ~ the same
            # location (and if so, don't add).
            for wobj in self._objects:
                if (World._pose_distance(wobj.object.pose, pose)
                        < OBJ_ADD_DIST_THRESHHOLD):
                    rospy.loginfo(
                        'Previously detected object at the same location, ' +
                        'will not add this object.')
                    return False

            # Actually add the object.
            self._add_new_object_internal(
                pose, dimensions, is_recognized, point_cloud, mesh)
            return True

    def _add_new_object_internal(self, pose, dimensions, is_recognized,
                                 point_cloud=None, mesh=None):
        '''Does the 'internal' adding of an object with the passed
        properties. Call _add_new_object to do all pre-requisite checks
        first (it then calls this function).

        Args:
            pose (Pose)
            dimensions (Vector3)
            is_recognized (bool)
            mesh (Mesh, optional): Unused.
        '''
        # TODO(sarah) : implement adding meshes?
        n_objects = len(self._objects)
        self._objects.append(WorldLandmark(self._remove_object,
            self._generate_grasps, pose, n_objects, dimensions, 
            is_recognized, point_cloud))
        int_marker = self._get_object_marker(len(self._objects) - 1)
        self._objects[-1].int_marker = int_marker
        self._im_server.insert(int_marker, self._marker_feedback_cb)
        self._im_server.applyChanges()
        self._objects[-1].menu_handler.apply(
            self._im_server, int_marker.name)
        self._im_server.applyChanges()

    def _remove_object(self, to_remove):
        '''Remove an object by name.

        Args:
            to_remove (string): Name of the object to remove in
                self._objects.
        '''

        resp = self._get_object_from_name(GetObjectFromNameRequest(to_remove))
        for obj in self._objects:
            if obj.object == resp.obj:
                self._objects.remove(obj)
                rospy.loginfo('Removing object ' + obj.int_marker.name)
                self._im_server.erase(obj.int_marker.name)
                self._im_server.applyChanges()
            break

        # TODO(mbforbes): Re-implement object recognition or remove
        # this dead code.
        # if (obj.is_recognized):
        #     for i in range(len(self._objects)):
        #         if ((self._objects[i].is_recognized)
        #                 and self._objects[i].index > obj.index):
        #             self._objects[i].decrease_index()
        #     self.n_recognized -= 1
        # else:
        #     for i in range(len(self._objects)):
        #         if ((not self._objects[i].is_recognized) and
        #                 self._objects[i].index > obj.index):
        #             self._objects[i].decrease_index()
        #     self.n_unrecognized -= 1

    def _generate_grasps(self, to_grasp):
        '''Generate grasps for an object by name.

        Args:
            to_grasp (string): Name of the object to grasp in
                self._objects.
        '''
        rospy.loginfo("Asking for grasp suggestions from session")
        resp = self._get_object_from_name(GetObjectFromNameRequest(to_grasp))
        if resp.has_object:
            self._add_grasp_pub.publish(resp.obj)
        else:
            rospy.logwarn("No objects matching that name")

    def _remove_surface(self):
        '''Function to request removing surface (from IM).'''
        rospy.loginfo('Removing surface')
        self._im_server.erase('surface')
        self._im_server.applyChanges()
        self._surface = None

    def _get_object_marker(self, index, mesh=None):
        '''Generate and return a marker for world objects.

        Args:
            index (int): ID for the new marker.
            mesh (Mesh, optional):  Mesh to use for the marker. Only
                utilized if not None. Defaults to None.

        Returns:
            InteractiveMarker
        '''
        int_marker = InteractiveMarker()
        int_marker.name = self._objects[index].get_name()
        int_marker.header.frame_id = self._objects[index].object.point_cloud.header.frame_id
        int_marker.pose = self._objects[index].object.pose
        int_marker.scale = 1

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True

        # object_marker = Marker(
        #     type=Marker.CUBE,
        #     id=index,
        #     lifetime=MARKER_DURATION,
        #     scale=self._objects[index].object.dimensions,
        #     header=Header(frame_id=''),
        #     color=COLOR_OBJ,
        #     pose=Pose(Point(), Quaternion(w=1.0))
        # )

        pc = self._objects[index].object.point_cloud

        points_list, colors_list = World._pc2_to_points_and_colors(pc)
        object_marker = Marker(
            type=Marker.SPHERE_LIST, 
            id=index, 
            lifetime=MARKER_DURATION,
            scale=Vector3(0.005, 0.005, 0.005),
            header=Header(frame_id=pc.header.frame_id),
            pose=Pose(Point(), Quaternion(w=1.0)),
            points=points_list,
            colors=colors_list
        )

        rospy.loginfo("Length of points: {}, length of colours: {}".format(len(points_list), len(colors_list)))

        if mesh is not None:
            object_marker = World._get_mesh_marker(object_marker, mesh)
        button_control.markers.append(object_marker)

        # text_pos = Point()
        # text_pos.x = self._objects[index].object.pose.position.x
        # text_pos.y = self._objects[index].object.pose.position.y
        # text_pos.z = (
        #     self._objects[index].object.pose.position.z +
        #     self._objects[index].object.dimensions.z / 2 + OFFSET_OBJ_TEXT_Z)
        # button_control.markers.append(
        #     Marker(
        #         type=Marker.TEXT_VIEW_FACING,
        #         id=index + 100,
        #         scale=SCALE_TEXT,
        #         text=int_marker.name,
        #         color=COLOR_TEXT,
        #         header=Header(frame_id=BASE_LINK),
        #         pose=Pose(text_pos, Quaternion(0, 0, 0, 1))
        #     )
        # )
        int_marker.controls.append(button_control)
        return int_marker

    def _publish_tf_pose(self, pose, name, parent):
        ''' Publishes a TF for object named name with pose pose and
        parent reference frame parent.

        Args:
            pose (Pose): The object's pose.
            name (str): The object's name.
            parent (str): The parent reference frame.
        '''
        if pose is not None:
            position = pose.position
            orientation = pose.orientation
            pos = (position.x, position.y, position.z)
            rot = (orientation.x, orientation.y, orientation.z, orientation.w)
            # TODO(mbforbes): Is it necessary to change the position
            # and orientation into tuples to send to TF?
            self._tf_broadcaster.sendTransform(
                pos, rot, rospy.Time.now(), name, parent)
