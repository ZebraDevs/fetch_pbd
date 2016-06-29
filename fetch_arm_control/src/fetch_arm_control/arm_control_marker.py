'''Represents a single 3D marker (in RViz) for visualizing the steps of
an action.'''

# ######################################################################
# Imports
# ######################################################################

import rospy
import numpy
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose
from std_msgs.msg import Header, ColorRGBA
import tf
from interactive_markers.interactive_marker_server import (
    InteractiveMarkerServer)
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import (
    Marker, InteractiveMarker, InteractiveMarkerControl,
    InteractiveMarkerFeedback)
import threading
from fetch_arm_control.msg import GripperState
import numpy.linalg
from numpy import *



# ######################################################################
# Constants
# ######################################################################

DEFAULT_OFFSET = 0.085
COLOR_MESH_REACHABLE = ColorRGBA(0.5, 0.5, 0.5, 0.6)
COLOR_MESH_UNREACHABLE = ColorRGBA(0.05, 0.05, 0.05, 0.6)
STR_MESH_GRIPPER_FOLDER = 'package://fetch_description/meshes/'
STR_GRIPPER_PALM_FILE = STR_MESH_GRIPPER_FOLDER + 'gripper_link.STL'
STR_L_GRIPPER_FINGER_FILE = STR_MESH_GRIPPER_FOLDER + 'l_gripper_finger_link.STL'
STR_R_GRIPPER_FINGER_FILE = STR_MESH_GRIPPER_FOLDER + 'r_gripper_finger_link.STL'
INT_MARKER_SCALE = 0.2
GRIPPER_MARKER_SCALE = 1.05
REF_FRAME = 'base_link'

# ######################################################################
# Class 
# ######################################################################

class ArmControlMarker:
    '''Marker for visualizing the steps of an action.'''

    _im_server = None

    def __init__(self, arm):
        '''
        Args:
            arm (Arm): Arm object for the right or left PR2 arm.
        '''
        if ArmControlMarker._im_server is None:
            im_server = InteractiveMarkerServer('interactive_arm_control')
            ArmControlMarker._im_server = im_server

        self._arm = arm
        self._is_control_visible = False
        self._menu_handler = None
        self._prev_is_reachable = None
        self._pose = self._arm.get_ee_state()
        self._lock = threading.Lock()
        self._current_pose = None
        self._menu_control = None

    @staticmethod
    def pose2array(p):
        return array((p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    @staticmethod
    def isTheSame(pose1, pose2):
        diff = pose1 - pose2
        dist = linalg.norm(diff)
        return (dist < 0.001)

    def _markerMoved(self):
        moved = True
        current_pose = self.get_pose()
        if not self._current_pose is None:
            if ArmControlMarker.isTheSame(ArmControlMarker.pose2array(current_pose), 
                ArmControlMarker.pose2array(self._current_pose)):
                moved = False
        self._current_pose = current_pose
        return moved 

    def update(self):

        self._menu_handler = MenuHandler()

        # Inset main menu entries.
        self._menu_handler.insert(
            'Move gripper here', callback=self.move_to_cb)
        self._menu_handler.insert(
            'Move marker to current gripper pose',
            callback=self.move_pose_to_cb)

        if self._is_hand_open():
            self._menu_handler.insert(
                'Close gripper',
                callback=self.close_gripper_cb)
        else:
            self._menu_handler.insert(
                'Open gripper',
                callback=self.open_gripper_cb)

        frame_id = REF_FRAME
        pose = self.get_pose()

        if self._markerMoved() or self._menu_control is None: 
            rospy.loginfo("Marker moved")

            menu_control = InteractiveMarkerControl()
            menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
            menu_control.always_visible = True
       

            menu_control = self._make_gripper_marker(
                menu_control, self._is_hand_open())
            self._menu_control = menu_control

        # Make and add interactive marker.
        int_marker = InteractiveMarker()
        int_marker.name = self._get_name()
        int_marker.header.frame_id = frame_id
        int_marker.pose = pose
        int_marker.scale = INT_MARKER_SCALE
        self._add_6dof_marker(int_marker, True)
        int_marker.controls.append(self._menu_control)
        ArmControlMarker._im_server.insert(
            int_marker, self.marker_feedback_cb)

        self._menu_handler.apply(ArmControlMarker._im_server,
            self._get_name())
        ArmControlMarker._im_server.applyChanges()

    def reset(self):
        self.set_new_pose(self._arm.get_ee_state(), is_offset=True)  

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
    def _offset_pose(pose, constant=1):
        '''Offsets the world pose for visualization.

        Args:
            pose (Pose): The pose to offset.
            constant (int, optional): How much to scale the set offset
                by (scales DEFAULT_OFFSET). Defaults to 1.

        Returns:
            Pose: The offset pose.
        '''
        transform = ArmControlMarker.get_matrix_from_pose(pose)
        offset_array = [constant * DEFAULT_OFFSET, 0, 0]
        offset_transform = tf.transformations.translation_matrix(offset_array)
        hand_transform = tf.transformations.concatenate_matrices(
            transform, offset_transform)
        return ArmControlMarker.get_pose_from_transform(hand_transform)

    def get_uid(self):
        '''Returns a unique id for this marker.

        Returns:
            int: A number that is unique given the arm
                index.
        '''
        return 0

    def destroy(self):
        '''Removes marker from the world.'''
        ArmControlMarker._im_server.erase(self._get_name())
        ArmControlMarker._im_server.applyChanges()

    def set_new_pose(self, new_pose, is_offset=False):
        '''Changes the pose of the action step to new_pose.

        Args:
            new_pose (Pose)
        '''
        self._lock.acquire()
        if is_offset:
            self._pose = new_pose
        else:
            self._pose = ArmControlMarker._offset_pose(new_pose, -1)
        self._lock.release()

    @staticmethod
    def copy_pose(pose):
        copy = Pose(
            Point(pose.position.x, pose.position.y, pose.position.z),
            Quaternion(pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w)
        )
        return copy

    def get_pose(self):
        '''Returns the pose of the action step.

        Returns:
            Pose
        '''
        self._lock.acquire()
        pose = ArmControlMarker.copy_pose(self._pose)
        self._lock.release()
        return ArmControlMarker._offset_pose(pose)

    def marker_feedback_cb(self, feedback):
        '''Callback for when an event occurs on the marker.

        Args:
            feedback (InteractiveMarkerFeedback)
        '''
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.set_new_pose(feedback.pose)
        elif feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            # Set the visibility of the 6DOF controller.
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            rospy.logdebug('Changing visibility of the pose controls.')
            self._is_control_visible = not self._is_control_visible
        else:
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            rospy.logdebug('Unknown event: ' + str(feedback.event_type))


    def open_gripper_cb(self, __):
        self._arm.open_gripper()

    def close_gripper_cb(self, __):
        self._arm.close_gripper()

    def move_to_cb(self, __):
        '''Callback for when moving to a pose is requested.

        Args:
            __ (???): Unused
        '''
        rospy.loginfo("Move to cb from marker")

        self._lock.acquire()
        pose = ArmControlMarker.copy_pose(self._pose)
        self._lock.release()

        target_joints = self._arm.get_ik_for_ee(
                pose, self._arm.get_joint_state())

        if target_joints is not None:
            time_to_pose = self._arm.get_time_to_pose(self.get_pose())

            thread = threading.Thread(
                group=None,
                target=self._arm.move_to_joints,
                args=(target_joints, time_to_pose),
                name='move_to_arm_state_thread'
            )
            thread.start()

            # Log
            # side_str = self._arm.side()
            rospy.loginfo('Started thread to move arm.')
        else:
            rospy.loginfo('Will not move arm; unreachable.')

    def move_pose_to_cb(self, __):
        '''Callback for when a pose change to current is requested.

        Args:
            __ (???): Unused

        '''
        self.reset()

    def _is_reachable(self):
        '''Checks and returns whether there is an IK solution for this
        action step.

        Returns:
            bool: Whether this action step is reachable.
        '''

        rospy.loginfo("Is reachable??")

        self._lock.acquire()
        pose = ArmControlMarker.copy_pose(self._pose)
        self._lock.release()

        target_joints = self._arm.get_ik_for_ee(
                pose, self._arm.get_joint_state())


        is_reachable = (target_joints is not None)

        # A bit more complicated logging to avoid spamming the logs
        # while still giving useful info. It now logs when reachability
        # is first calculated, or changes.
        report = False

        # See if we've set reachability for the first time.
        if self._prev_is_reachable is None:
            report = True
            reachable_str = (
                'is reachable' if is_reachable else 'is not reachable')
        # See if we've got a different reachability than we had before.
        elif self._prev_is_reachable != is_reachable:
            report = True
            reachable_str = (
                'is now reachable' if is_reachable else
                'is no longer reachable')

        # Log if it's changed.
        if report:
            rospy.loginfo(reachable_str)

        # Cache and return.
        self._prev_is_reachable = is_reachable

        print 
        return is_reachable

    def _get_name(self):
        '''Generates the unique name for the marker.

        Returns:
            str: A human-readable unique name for the marker.
        '''
        return 'arm'

    def _is_hand_open(self):
        '''Returns whether the gripper is open for this action step.

        Returns:
            bool
        '''
        return self._arm.get_gripper_state() == GripperState.OPEN

    def _add_6dof_marker(self, int_marker, is_fixed):
        '''Adds a 6 DoF control marker to the interactive marker.

        Args:
            int_marker (InteractiveMarker)
            is_fixed (bool): Looks like whether position is fixed (?).
                Currently only passed as True.
        '''
        # Each entry in options is (name, orientation, is_move)
        options = [
            ('rotate_x', Quaternion(1, 0, 0, 1), False),
            ('move_x', Quaternion(1, 0, 0, 1), True),
            ('rotate_z', Quaternion(0, 1, 0, 1), False),
            ('move_z', Quaternion(0, 1, 0, 1), True),
            ('rotate_y', Quaternion(0, 0, 1, 1), False),
            ('move_y', Quaternion(0, 0, 1, 1), True),
        ]
        for opt in options:
            name, orient, is_move = opt
            control = self._make_6dof_control(
                name, orient, is_move, is_fixed)
            int_marker.controls.append(control)

    def _make_6dof_control(self, name, orientation, is_move, is_fixed):
        '''Creates and returns one component of the 6dof controller.

        Args:
            name (str): Name for hte control
            orientation (Quaternion): How the control should be
                oriented.
            is_move (bool): Looks like whether the marker moves the
                object (?). Currently passed as True for moving markers,
                False for rotating ones.
            is_fixed (bool): Looks like whether position is fixed (?).
                Currently always passed as True.

        Returns:
            InteractiveMarkerControl
        '''
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation = orientation
        control.always_visible = False
        if self._is_control_visible:
            if is_move:
                control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            else:
                control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        else:
            control.interaction_mode = InteractiveMarkerControl.NONE
        if is_fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        return control

    def _make_mesh_marker(self, color):
        '''Creates and returns a mesh marker.

        Returns:
            Marker
        '''
        mesh = Marker()
        mesh.mesh_use_embedded_materials = False
        mesh.type = Marker.MESH_RESOURCE
        mesh.scale.x = 1.0
        mesh.scale.y = 1.0
        mesh.scale.z = 1.0
        mesh.color = color
        return mesh

    def _get_mesh_marker_color(self):
        '''Gets the color for the mesh marker (thing that looks like a
        gripper) for this step.

        A simple implementation of this will return one color for
        reachable poses, another for unreachable ones. Future
        implementations may provide further visual cues.

        Returns:
            ColorRGBA: The color for the gripper mesh for this step.
        '''
        if self._is_reachable():
            return COLOR_MESH_REACHABLE
        else:
            return COLOR_MESH_UNREACHABLE

    def _make_gripper_marker(self, control, is_hand_open=False):
        '''Makes a gripper marker, adds it to control, returns control.

        Args:
            control (InteractiveMarkerControl): IM Control we're using.
            is_hand_open (bool, optional): Whether the gripper is open.
                Defaults to False (closed).

        Returns:
            InteractiveMarkerControl: The passed control.
        '''
        mesh_color = self._get_mesh_marker_color()

        # Create mesh 1 (palm).
        mesh1 = self._make_mesh_marker(mesh_color)
        mesh1.mesh_resource = STR_GRIPPER_PALM_FILE
        mesh1.pose.position.x = DEFAULT_OFFSET
        mesh1.pose.orientation.w = 1

        # TODO (sarah): make all of these numbers into constants
        if is_hand_open:
            mesh2 = self._make_mesh_marker(mesh_color)
            mesh2.mesh_resource = STR_L_GRIPPER_FINGER_FILE
            mesh2.pose.position.x = 0.08
            mesh2.pose.position.y = -0.165
            mesh2.pose.orientation.w = 1

            mesh3 = self._make_mesh_marker(mesh_color)
            mesh3.mesh_resource = STR_R_GRIPPER_FINGER_FILE
            mesh3.pose.position.x = 0.08 
            mesh3.pose.position.y = 0.165
            mesh3.pose.orientation.w = 1
        else:
            mesh2 = self._make_mesh_marker(mesh_color)
            mesh2.mesh_resource = STR_L_GRIPPER_FINGER_FILE
            mesh2.pose.position.x = 0.08 
            mesh2.pose.position.y = -0.116
            mesh2.pose.orientation.w = 1

            mesh3 = self._make_mesh_marker(mesh_color)
            mesh3.mesh_resource = STR_R_GRIPPER_FINGER_FILE
            mesh3.pose.position.x = 0.08 
            mesh3.pose.position.y = 0.116
            mesh3.pose.orientation.w = 1

        # Append all meshes we made.
        control.markers.append(mesh1)
        control.markers.append(mesh2)
        control.markers.append(mesh3)

        # Return back the control.
        # TODO(mbforbes): Why do we do this?
        return control
