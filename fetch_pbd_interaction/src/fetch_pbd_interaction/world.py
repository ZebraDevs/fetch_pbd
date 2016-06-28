'''Everything related to perception of the world'''

# ######################################################################
# Imports
# ######################################################################

import time
import threading
from numpy.linalg import norm
from numpy import array
import rospy
import tf
from tf import TransformListener, TransformBroadcaster
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose, PoseStamped
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from actionlib_msgs.msg import GoalStatus
import actionlib
from math import pi, sin, cos
# from ar_track_alvar.msg import AlvarMarkers
from tabletop_object_detector.srv import TabletopSegmentation
from fetch_pbd_interaction.msg import Landmark, ArmState
from fetch_pbd_interaction.response import Response
from fetch_social_gaze.msg import GazeGoal
from world_landmark import WorldLandmark


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
# Landmark dimensions. I don't fully understand this, as it seems like
# each object's dimensions should be extracted from the point cloud.
# But apparently this works and is a default or something?
DIMENSIONS_OBJ = Vector3(0.2, 0.2, 0.2)

# Colors
COLOR_OBJ = ColorRGBA(0.2, 0.8, 0.0, 0.6)
COLOR_SURFACE = ColorRGBA(0.8, 0.0, 0.4, 0.4)
COLOR_TEXT = ColorRGBA(0.0, 0.0, 0.0, 0.5)

# Frames
BASE_LINK = 'base_link'

# Time
MARKER_DURATION = rospy.Duration(2)
# How long to pause when waiting for external code, like gaze actions or
# object segmentation, to finish before checking again.
PAUSE_SECONDS = rospy.Duration(0.1)
# How long we're willing to wait for object recognition.
RECOGNITION_TIMEOUT_SECONDS = rospy.Duration(5.0)


# ######################################################################
# Classes
# ######################################################################
class World:
    '''Handles object recognition, localization, and coordinate space
    transformations.'''

    tf_listener = None

    # Type: [WorldLandmark]
    objects = []

    def __init__(self):
        # Public attributes
        if World.tf_listener is None:
            World.tf_listener = TransformListener()
        self.surface = None

        # Private attributes
        self._lock = threading.Lock()
        self._tf_broadcaster = TransformBroadcaster()
        self._im_server = InteractiveMarkerServer('world_objects')
        
        rospy.wait_for_service('tabletop_segmentation')
        self._segmentation_service = rospy.ServiceProxy(
            'tabletop_segmentation',
            TabletopSegmentation)

        # rospy.wait_for_service('find_cluster_bounding_box')
        # self._bb_service = rospy.ServiceProxy(
        #     'find_cluster_bounding_box',
        #     FindClusterBoundingBox)
        
        # self._object_action_client = actionlib.SimpleActionClient(
        #     'object_detection_user_command',
        #     UserCommandAction)
        # self._object_action_client.wait_for_server()

        # rospy.loginfo(
        #     'Interactive object detection action server has responded.')

        # Setup other ROS machinery
        # rospy.Subscriber(
        #     'interactive_object_recognition_result',
        #     GraspableLandmarkList,
        #     self.receive_object_info)
        # rospy.Subscriber('tabletop_segmentation_markers',
        #     Marker,
        #     self.receive_table_marker)

        # Init
        self.clear_all_objects()

    # ##################################################################
    # Static methods: Public (API)
    # ##################################################################

    @staticmethod
    def get_pose_from_transform(transform):
        '''Returns pose for transformation matrix.

        Args:
            transform (Matrix3x3): (I think this is the correct type.
                See ActionStepMarker as a reference for how to use.)

        Returns:
            Pose
        '''
        pos = transform[:3, 3].copy()
        rot = tf.transformations.quaternion_from_matrix(transform)
        return Pose(
            Point(pos[0], pos[1], pos[2]),
            Quaternion(rot[0], rot[1], rot[2], rot[3])
        )

    @staticmethod
    def get_matrix_from_pose(pose):
        '''Returns the transformation matrix for given pose.

        Args:
            pose (Pose)

        Returns:
            Matrix3x3: (I think this is the correct type. See
                ActionStepMarker as a reference for how to use.)
        '''
        pp, po = pose.position, pose.orientation
        rotation = [po.x, po.y, po.z, po.w]
        transformation = tf.transformations.quaternion_matrix(rotation)
        position = [pp.x, pp.y, pp.z]
        transformation[:3, 3] = position
        return transformation

    @staticmethod
    def get_absolute_pose(arm_state):
        '''Returns absolute pose of an end effector state (trasnforming
        if relative).

        Args:
            arm_state (ArmState)

        Returns:
            Pose
        '''
        if arm_state.refFrame == ArmState.OBJECT:
            arm_state_copy = ArmState(
                arm_state.refFrame, Pose(
                    arm_state.ee_pose.position,
                    arm_state.ee_pose.orientation),
                arm_state.joint_pose[:],
                arm_state.refFrameLandmark)
            World.convert_ref_frame(arm_state_copy, ArmState.ROBOT_BASE)
            return arm_state_copy.ee_pose
        else:
            return arm_state.ee_pose

    @staticmethod
    def get_most_similar_obj(ref_object, ref_frame_list):
        '''Finds the most similar object in the world.

        Args:
            ref_object (?)
            ref_frame_list ([Landmark]): List of objects (as defined by
                Landmark.msg).

        Returns:
            Landmark|None: As in one of Landmark.msg, or None if no object
                was found close enough.
        '''
        best_dist = 10000  # Not a constant; an absurdly high number.
        chosen_obj = None
        for ref_frame in ref_frame_list:
            dist = World.object_dissimilarity(ref_frame, ref_object)
            if dist < best_dist:
                best_dist = dist
                chosen_obj = ref_frame
        if chosen_obj is None:
            rospy.loginfo('Did not find a similar object.')
        else:
            rospy.loginfo('Landmark dissimilarity is --- ' + str(best_dist))
            if best_dist > OBJ_SIMILAR_DIST_THRESHHOLD:
                rospy.loginfo('Found some objects, but not similar enough.')
                chosen_obj = None
            else:
                rospy.loginfo(
                    'Most similar to new object: ' + str(chosen_obj.name))

        # Regardless, return the "closest object," which may be None.
        return chosen_obj

    @staticmethod
    def get_frame_list():
        '''Function that returns the list of reference frames (Landmarks).

        Returns:
            [Landmark]: List of Landmark (as defined by Landmark.msg), the
                current reference frames.
        '''
        return [w_obj.object for w_obj in World.objects]

    @staticmethod
    def has_objects():
        '''Returns whetehr there are any objects (reference frames).

        Returns:
            bool
        '''
        return len(World.objects) > 0

    @staticmethod
    def object_dissimilarity(obj1, obj2):
        '''Returns distance between two objects.

        Returns:
            float
        '''
        d1 = obj1.dimensions
        d2 = obj2.dimensions
        return norm(array([d1.x, d1.y, d1.z]) - array([d2.x, d2.y, d2.z]))

    @staticmethod
    def get_ref_from_name(ref_name):
        '''Returns the reference frame type from the reference frame
        name specified by ref_name.

        Args:
            ref_name (str): Name of a referene frame.

        Returns:
            int: One of ArmState.*, the number code of the reference
                frame specified by ref_name.
        '''
        if ref_name == 'base_link':
            return ArmState.ROBOT_BASE
        else:
            return ArmState.OBJECT

    @staticmethod
    def convert_ref_frame(arm_frame, ref_frame, ref_frame_obj=Landmark()):
        '''Transforms an arm frame to a new ref. frame.

        Args:
            arm_frame (ArmState)
            ref_frame (int): One of ArmState.*
            ref_frame_obj (Landmark): As in Landmark.msg

        Returns:
            ArmState: arm_frame (passed in), but modified.
        '''
        if ref_frame == ArmState.ROBOT_BASE:
            if arm_frame.refFrame == ArmState.ROBOT_BASE:
                # Transform from robot base to itself (nothing to do).
                rospy.logdebug(
                    'No reference frame transformations needed (both ' +
                    'absolute).')
            elif arm_frame.refFrame == ArmState.OBJECT:
                # Transform from object to robot base.
                abs_ee_pose = World.transform(
                    arm_frame.ee_pose,
                    arm_frame.refFrameLandmark.name,
                    'base_link'
                )
                arm_frame.ee_pose = abs_ee_pose
                arm_frame.refFrame = ArmState.ROBOT_BASE
                arm_frame.refFrameLandmark = Landmark()
            else:
                rospy.logerr(
                    'Unhandled reference frame conversion: ' +
                    str(arm_frame.refFrame) + ' to ' + str(ref_frame))
        elif ref_frame == ArmState.OBJECT:
            if arm_frame.refFrame == ArmState.ROBOT_BASE:
                # Transform from robot base to object.
                rel_ee_pose = World.transform(
                    arm_frame.ee_pose, 'base_link', ref_frame_obj.name)
                arm_frame.ee_pose = rel_ee_pose
                arm_frame.refFrame = ArmState.OBJECT
                arm_frame.refFrameLandmark = ref_frame_obj
            elif arm_frame.refFrame == ArmState.OBJECT:
                # Transform between the same object (nothign to do).
                if arm_frame.refFrameLandmark.name == ref_frame_obj.name:
                    rospy.logdebug(
                        'No reference frame transformations needed (same ' +
                        'object).')
                else:
                    # Transform between two different objects.
                    rel_ee_pose = World.transform(
                        arm_frame.ee_pose,
                        arm_frame.refFrameLandmark.name,
                        ref_frame_obj.name
                    )
                    arm_frame.ee_pose = rel_ee_pose
                    arm_frame.refFrame = ArmState.OBJECT
                    arm_frame.refFrameLandmark = ref_frame_obj
            else:
                rospy.logerr(
                    'Unhandled reference frame conversion: ' +
                    str(arm_frame.refFrame) + ' to ' + str(ref_frame))
        return arm_frame

    @staticmethod
    def has_object(object_name):
        '''Returns whether the world contains an Landmark with object_name.

        Args:
            object_name (str)

        Returns:
            bool
        '''
        return object_name in [wobj.object.name for wobj in World.objects]

    @staticmethod
    def is_frame_valid(object_name):
        '''Returns whether the frame (object) name is valid for
        transforms.

        Args:
            object_name (str)

        Returns:
            bool
        '''
        return object_name == 'base_link' or World.has_object(object_name)

    @staticmethod
    def transform(pose, from_frame, to_frame):
        '''Transforms a pose between two reference frames. If there is a
        TF exception or object does not exist, it will return the pose
        back without any transforms.

        Args:
            pose (Pose)
            from_frame (str)
            to_frame (str)

        Returns:
            Pose
        '''
        if World.is_frame_valid(from_frame) and World.is_frame_valid(to_frame):
            pose_stamped = PoseStamped()
            try:
                common_time = World.tf_listener.getLatestCommonTime(
                    from_frame, to_frame)
                pose_stamped.header.stamp = common_time
                pose_stamped.header.frame_id = from_frame
                pose_stamped.pose = pose
                rel_ee_pose = World.tf_listener.transformPose(
                    to_frame, pose_stamped)
                return rel_ee_pose.pose
            except tf.Exception:
                rospy.logerr('TF exception during transform.')
                return pose
            except rospy.ServiceException:
                rospy.logerr('ServiceException during transform.')
                return pose
        else:
            rospy.logdebug(
                'One of the frame objects might not exist: ' + from_frame +
                ' or ' + to_frame)
            return pose

    @staticmethod
    def pose_distance(pose1, pose2, is_on_table=True):
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

    # ##################################################################
    # Static methods: Internal ("private")
    # ##################################################################

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
            header=Header(frame_id=BASE_LINK),
            color=COLOR_SURFACE,
            pose=pose
        )
        button_control.markers.append(object_marker)
        text_pos = Point()
        position = pose.position
        dimensions = dimensions
        text_pos.x = position.x + dimensions.x / 2 - 0.06
        text_pos.y = position.y - dimensions.y / 2 + 0.06
        text_pos.z = position.z + dimensions.z / 2 + 0.06
        text_marker = Marker(
            type=Marker.TEXT_VIEW_FACING,
            id=2001,
            scale=SCALE_TEXT, text=int_marker.name,
            color=COLOR_TEXT,
            header=Header(frame_id=BASE_LINK),
            pose=Pose(text_pos, Quaternion(0, 0, 0, 1))
        )
        button_control.markers.append(text_marker)
        int_marker.controls.append(button_control)
        return int_marker

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def update_object_pose(self):
        ''' Function to externally update an object pose.'''
        # Look down at the table.
        rospy.loginfo('Head attempting to look at table.')
        Response.force_gaze_action(GazeGoal.LOOK_DOWN)
        while (Response.gaze_client.get_state() == GoalStatus.PENDING or
               Response.gaze_client.get_state() == GoalStatus.ACTIVE):
            rospy.sleep(PAUSE_SECONDS)
        if Response.gaze_client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr('Could not look down to take table snapshot')
            return False
        rospy.loginfo('Head is now (successfully) stairing at table.')

        rospy.loginfo("waiting for segmentation service")

        try:
            resp = self._segmentation_service()
            rospy.loginfo("Adding landmarks")

            self._reset_objects()

            # add the table
            xmin = resp.table.x_min
            ymin = resp.table.y_min
            xmax = resp.table.x_max
            ymax = resp.table.y_max
            depth = xmax - xmin
            width = ymax - ymin

            pose = resp.table.pose.pose
            pose.position.x = pose.position.x + xmin + depth / 2
            pose.position.y = pose.position.y + ymin + width / 2
            dimensions = Vector3(depth, width, 0.01)
            self.surface = World._get_surface_marker(pose, dimensions)
            self._im_server.insert(self.surface,
                                   self.marker_feedback_cb)
            self._im_server.applyChanges()

            for cluster in resp.clusters:
                points = cluster.points
                if (len(points) == 0):
                    return Point(0, 0, 0)
                [minX, maxX, minY, maxY, minZ, maxZ] = [
                    points[0].x, points[0].x, points[0].y, points[0].y,
                    points[0].z, points[0].z]
                for pt in points:
                    minX = min(minX, pt.x)
                    minY = min(minY, pt.y)
                    minZ = min(minZ, pt.z)
                    maxX = max(maxX, pt.x)
                    maxY = max(maxY, pt.y)
                    maxZ = max(maxZ, pt.z)
                self._add_new_object(Pose(Point((minX + maxX) / 2, (minY + maxY) / 2,
                                                (minZ + maxZ) / 2), Quaternion(0, 0, 0, 1)),
                                     Point(maxX - minX, maxY - minY, maxZ - minZ), False)
            return True

        except rospy.ServiceException, e:
            print "Call to segmentation service failed: %s" % e
            return False

    @staticmethod
    def get_tf_pose(tf_name, ref_frame='base_link'):
        ''' Returns end effector pose for the arm'''
        try:
            time = World.tf_listener.getLatestCommonTime(ref_frame,
                                                         tf_name)
            (position, orientation) = World.tf_listener.lookupTransform(
                                                ref_frame, tf_name, time)
            tf_pose = Pose()
            tf_pose.position = Point(position[0], position[1], position[2])
            tf_pose.orientation = Quaternion(orientation[0], orientation[1],
                                             orientation[2], orientation[3])
            return tf_pose
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logwarn('Something wrong with transform request: ' + str(e))
            return None

    def clear_all_objects(self):
        '''Removes all objects from the world.'''
        self._reset_objects()
        self._remove_surface()

    def get_nearest_object(self, arm_pose):
        '''Returns the nearest object, if one exists.

        Args:
            arm_pose (Pose): End-effector pose.

        Returns:
            Landmark|None: As in Landmark.msg, the nearest object (if it
                is close enough), or None if there were none close
                enough.
        '''
        # First, find which object is the closest.
        distances = []
        for wobj in World.objects:
            dist = World.pose_distance(wobj.object.pose, arm_pose)
            distances.append(dist)

        # Then, see if the closest is actually below our threshhold for
        # a 'closest object.'
        if len(distances) > 0:
            if min(distances) < OBJ_NEAREST_DIST_THRESHHOLD:
                chosen = distances.index(min(distances))
                return World.objects[chosen].object

        # We didn't have any objects or none were close enough.
        return None

    def marker_feedback_cb(self, feedback):
        '''Callback for when feedback from a marker is received.

        Args:
            feedback (InteractiveMarkerFeedback)
        '''
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo('Clicked on object ' + str(feedback.marker_name))
            rospy.loginfo('Number of objects ' + str(len(World.objects)))
        else:
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            rospy.logdebug('Unknown event: ' + str(feedback.event_type))

    def update(self):
        '''Update function called in a loop.

        Returns:
            bool: Whether any tracked objects were removed, AKA "is
                world changed."
        '''
        # Visualize the detected object
        is_world_changed = False
        self._lock.acquire()
        if World.has_objects():
            to_remove = None
            for i in range(len(World.objects)):
                self._publish_tf_pose(
                    World.objects[i].object.pose,
                    World.objects[i].get_name(),
                    BASE_LINK
                )
                if World.objects[i].is_removed:
                    to_remove = i
            if to_remove is not None:
                self._remove_object(to_remove)
                is_world_changed = True

        self._lock.release()
        return is_world_changed

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _reset_objects(self):
        '''Removes all objects.'''
        self._lock.acquire()
        for wobj in World.objects:
            self._im_server.erase(wobj.int_marker.name)
            self._im_server.applyChanges()
        if self.surface is not None:
            self._remove_surface()
        self._im_server.clear()
        self._im_server.applyChanges()
        World.objects = []
        self._lock.release()

    def _add_new_object(self, pose, dimensions, is_recognized, mesh=None):
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
        to_remove = None
        if is_recognized:
            # TODO(mbforbes): Re-implement object recognition or remove
            # this dead code.
            return False
            # # Check if there is already an object
            # for i in range(len(World.objects)):
            #     distance = World.pose_distance(
            #         World.objects[i].object.pose, pose)
            #     if distance < OBJ_ADD_DIST_THRESHHOLD:
            #         if World.objects[i].is_recognized:
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
            for wobj in World.objects:
                if (World.pose_distance(wobj.object.pose, pose)
                        < OBJ_ADD_DIST_THRESHHOLD):
                    rospy.loginfo(
                        'Previously detected object at the same location, ' +
                        'will not add this object.')
                    return False

            # Actually add the object.
            self._add_new_object_internal(
                pose, dimensions, is_recognized, mesh)
            return True

    def _add_new_object_internal(self, pose, dimensions, is_recognized, mesh):
        '''Does the 'internal' adding of an object with the passed
        properties. Call _add_new_object to do all pre-requisite checks
        first (it then calls this function).

        Args:
            pose (Pose)
            dimensions (Vector3)
            is_recognized (bool)
            mesh (Mesh|None): A mesh, if it exists (can be None).
        '''
        n_objects = len(World.objects)
        World.objects.append(WorldLandmark(
            pose, n_objects, dimensions, is_recognized))
        int_marker = self._get_object_marker(len(World.objects) - 1)
        World.objects[-1].int_marker = int_marker
        self._im_server.insert(int_marker, self.marker_feedback_cb)
        self._im_server.applyChanges()
        World.objects[-1].menu_handler.apply(
            self._im_server, int_marker.name)
        self._im_server.applyChanges()

    def _remove_object(self, to_remove):
        '''Remove an object by index.

        Args:
            to_remove (int): Index of the object to remove in
                World.objects.
        '''
        obj = World.objects.pop(to_remove)
        rospy.loginfo('Removing object ' + obj.int_marker.name)
        self._im_server.erase(obj.int_marker.name)
        self._im_server.applyChanges()
        # TODO(mbforbes): Re-implement object recognition or remove
        # this dead code.
        # if (obj.is_recognized):
        #     for i in range(len(World.objects)):
        #         if ((World.objects[i].is_recognized)
        #                 and World.objects[i].index > obj.index):
        #             World.objects[i].decrease_index()
        #     self.n_recognized -= 1
        # else:
        #     for i in range(len(World.objects)):
        #         if ((not World.objects[i].is_recognized) and
        #                 World.objects[i].index > obj.index):
        #             World.objects[i].decrease_index()
        #     self.n_unrecognized -= 1

    def _remove_surface(self):
        '''Function to request removing surface (from IM).'''
        rospy.loginfo('Removing surface')
        self._im_server.erase('surface')
        self._im_server.applyChanges()
        self.surface = None

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
        int_marker.name = World.objects[index].get_name()
        int_marker.header.frame_id = 'base_link'
        int_marker.pose = World.objects[index].object.pose
        int_marker.scale = 1

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True

        object_marker = Marker(
            type=Marker.CUBE,
            id=index,
            lifetime=MARKER_DURATION,
            scale=World.objects[index].object.dimensions,
            header=Header(frame_id=BASE_LINK),
            color=COLOR_OBJ,
            pose=World.objects[index].object.pose
        )

        if mesh is not None:
            object_marker = World._get_mesh_marker(object_marker, mesh)
        button_control.markers.append(object_marker)

        text_pos = Point()
        text_pos.x = World.objects[index].object.pose.position.x
        text_pos.y = World.objects[index].object.pose.position.y
        text_pos.z = (
            World.objects[index].object.pose.position.z +
            World.objects[index].object.dimensions.z / 2 + OFFSET_OBJ_TEXT_Z)
        button_control.markers.append(
            Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=index,
                scale=SCALE_TEXT,
                text=int_marker.name,
                color=COLOR_TEXT,
                header=Header(frame_id=BASE_LINK),
                pose=Pose(text_pos, Quaternion(0, 0, 0, 1))
            )
        )
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
            pp = pose.position
            po = pose.orientation
            pos = (pp.x, pp.y, pp.z)
            rot = (po.x, po.y, po.z, po.w)
            # TODO(mbforbes): Is it necessary to change the position
            # and orientation into tuples to send to TF?
            self._tf_broadcaster.sendTransform(
                pos, rot, rospy.Time.now(), name, parent)
