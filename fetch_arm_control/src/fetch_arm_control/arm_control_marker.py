'''Represents a single 3D marker (in RViz) for controlling the Fetch arm.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# System builtins
from numpy import array, linalg
import threading

# ROS builtins
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped
from std_msgs.msg import Header, ColorRGBA
import tf
from interactive_markers.interactive_marker_server import (
    InteractiveMarkerServer)
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import (
    Marker, InteractiveMarker, InteractiveMarkerControl,
    InteractiveMarkerFeedback)

# Local
from fetch_arm_control.msg import GripperState


# ######################################################################
# Constants
# ######################################################################

DEFAULT_OFFSET = 0.085
COLOR_MESH_REACHABLE = ColorRGBA(1.0, 0.5, 0.0, 0.6)
COLOR_MESH_UNREACHABLE = ColorRGBA(0.5, 0.5, 0.5, 0.6)
STR_MESH_GRIPPER_FOLDER = 'package://fetch_description/meshes/'
STR_GRIPPER_PALM_FILE = STR_MESH_GRIPPER_FOLDER + 'gripper_link.STL'
STR_L_GRIPPER_FINGER_FILE = STR_MESH_GRIPPER_FOLDER + \
                                'l_gripper_finger_link.STL'
STR_R_GRIPPER_FINGER_FILE = STR_MESH_GRIPPER_FOLDER + \
                                'r_gripper_finger_link.STL'
INT_MARKER_SCALE = 0.2
GRIPPER_MARKER_SCALE = 1.05
REF_FRAME = 'base_link'

# ######################################################################
# Class
# ######################################################################

class ArmControlMarker:
    '''Marker for controlling arm of robot.'''

    _im_server = None

    def __init__(self, arm):
        '''
        Args:
            arm (Arm): Arm object for the robot's arm.
        '''
        if ArmControlMarker._im_server is None:
            im_server = InteractiveMarkerServer('interactive_arm_control')
            ArmControlMarker._im_server = im_server

        self._arm = arm
        self._name = 'arm'
        self._is_control_visible = False
        self._menu_handler = None
        self._prev_is_reachable = None
        self._pose = self._arm.get_ee_state()
        self._lock = threading.Lock()
        self._current_pose = None
        self._menu_control = None

        self.update()


    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def update(self):
        '''Updates marker/arm loop'''

        self._menu_handler = MenuHandler()

        # Inset main menu entries.
        self._menu_handler.insert(
            'Move gripper here', callback=self._move_to_cb)
        self._menu_handler.insert(
            'Move marker to current gripper pose',
            callback=self._move_pose_to_cb)

        if self._is_hand_open():
            self._menu_handler.insert(
                'Close gripper',
                callback=self._close_gripper_cb)
        else:
            self._menu_handler.insert(
                'Open gripper',
                callback=self._open_gripper_cb)

        frame_id = REF_FRAME
        pose = self.get_pose()

        # if self._marker_moved() or self._menu_control is None:
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
        int_marker.pose = pose.pose
        int_marker.scale = INT_MARKER_SCALE
        self._add_6dof_marker(int_marker, True)
        int_marker.controls.append(self._menu_control)
        ArmControlMarker._im_server.insert(
            int_marker, self._marker_feedback_cb)

        self._menu_handler.apply(ArmControlMarker._im_server,
            self._get_name())
        ArmControlMarker._im_server.applyChanges()

    def reset(self):
        '''Sets marker to current arm pose'''
        self.set_new_pose(self._arm.get_ee_state(), is_offset=True)

    def destroy(self):
        '''Removes marker from the world.'''
        ArmControlMarker._im_server.erase(self._get_name())
        ArmControlMarker._im_server.applyChanges()

    def set_new_pose(self, new_pose, is_offset=False):
        '''Changes the pose of the marker to new_pose.

        Args:
            new_pose (PoseStamped)
        '''
        self._lock.acquire()
        if is_offset:
            self._pose = new_pose
        else:
            self._pose = ArmControlMarker._offset_pose(new_pose, -1)
        self._lock.release()

    def get_pose(self):
        '''Returns the pose of the marker

        Returns:
            Pose
        '''
        self._lock.acquire()
        pose = ArmControlMarker._copy_pose(self._pose)
        self._lock.release()
        return ArmControlMarker._offset_pose(pose)

    # ##################################################################
    # Static methods: Internal ("private")
    # ##################################################################

    @staticmethod
    def _get_pose_from_transform(transform):
        '''Returns pose for transformation matrix.

        Args:
            transform (Matrix3x3)

        Returns:
            PoseStamped
        '''
        pos = transform[:3, 3].copy()
        rot = tf.transformations.quaternion_from_matrix(transform)
        return PoseStamped(Header(frame_id='base_link'),
                Pose(Point(pos[0], pos[1], pos[2]),
                     Quaternion(rot[0], rot[1], rot[2], rot[3])
        ))

    @staticmethod
    def _get_matrix_from_pose(pose):
        '''Returns the transformation matrix for given pose.

        Args:
            pose (PoseStamped)

        Returns:
            Matrix3x3
        '''
        pp, po = pose.pose.position, pose.pose.orientation
        rotation = [po.x, po.y, po.z, po.w]
        transformation = tf.transformations.quaternion_matrix(rotation)
        position = [pp.x, pp.y, pp.z]
        transformation[:3, 3] = position
        return transformation

    @staticmethod
    def _offset_pose(pose, constant=1):
        '''Offsets the world pose for visualization.

        Args:
            pose (PoseStamped): The pose to offset.
            constant (int, optional): How much to scale the set offset
                by (scales DEFAULT_OFFSET). Defaults to 1.

        Returns:
            PoseStamped: The offset pose.
        '''
        transform = ArmControlMarker._get_matrix_from_pose(pose)
        offset_array = [constant * DEFAULT_OFFSET, 0, 0]
        offset_transform = tf.transformations.translation_matrix(offset_array)
        hand_transform = tf.transformations.concatenate_matrices(
            transform, offset_transform)
        return ArmControlMarker._get_pose_from_transform(hand_transform)



    @staticmethod
    def _copy_pose(pose):
        '''Copies pose msg

        Args:
            pose (PoseStamped)
        Returns:
            PoseStamped
        '''
        copy = PoseStamped(
            Header(frame_id=pose.header.frame_id),
            Pose(Point(pose.pose.position.x, pose.pose.position.y,
                pose.pose.position.z),
            Quaternion(pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w))
        )
        return copy

    @staticmethod
    def _pose2array(p):
        '''Converts pose to array

        Args:
            p (PoseStamped)
        Returns:
            numpy.array
        '''
        return array((p.pose.position.x, p.pose.position.y,
                     p.pose.position.z,
                     p.pose.orientation.x, p.pose.orientation.y,
                     p.pose.orientation.z, p.pose.orientation.w))

    @staticmethod
    def _is_the_same(pose1, pose2, tol=0.001):
        '''Checks if poses are the same within tolerance

        Args:
            pose1 (Pose)
            pose2 (Pose)
            tol (float, optional)
        Returns:
            bool
        '''
        if pose1 is None or pose2 is None:
            return True

        diff = pose1 - pose2
        dist = linalg.norm(diff)
        return dist < tol

    @staticmethod
    def _make_mesh_marker(color):
        '''Creates and returns a mesh marker.

        Args:
            color (ColorRGBA)
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

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _marker_moved(self):
        '''Returns whether marker has moved

        Returns:
            bool
        '''
        moved = True
        current_pose = self.get_pose()
        if not self._current_pose is None:
            if ArmControlMarker._is_the_same(
                    ArmControlMarker._pose2array(current_pose),
                    ArmControlMarker._pose2array(self._current_pose)):
                moved = False
        self._current_pose = current_pose
        return moved

    def _marker_feedback_cb(self, feedback):
        '''Callback for when an event occurs on the marker.

        Args:
            feedback (InteractiveMarkerFeedback)
        '''
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            # Set the visibility of the 6DOF controller.
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            rospy.logdebug('Changing visibility of the pose controls.')
            self._is_control_visible = not self._is_control_visible
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.set_new_pose(PoseStamped(Header(frame_id='base_link'),
                                                 feedback.pose))
            self.update()
        else:
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            rospy.logdebug('Unknown event: ' + str(feedback.event_type))

    def _open_gripper_cb(self, feedback):
        '''Callback for opening gripper.

        Args:
            feedback (InteractiveMarkerFeedback): Unused
        '''
        self._arm.open_gripper()
        for i in range(30):
            if self._arm.get_gripper_state() == GripperState.OPEN:
                break
            rospy.sleep(0.1)
        self.update()

    def _close_gripper_cb(self, feedback):
        '''Callback for closing gripper.

        Args:
            feedback (InteractiveMarkerFeedback): Unused
        '''
        self._arm.close_gripper()
        for i in range(30):
            if self._arm.get_gripper_state() != GripperState.OPEN:
                break
            rospy.sleep(0.1)
        self.update()

    def _move_to_cb(self, feedback):
        '''Callback for when moving to a pose is requested.

        Args:
            feedback (InteractiveMarkerFeedback): Unused
        '''

        if not ArmControlMarker._is_the_same(
            ArmControlMarker._pose2array(
                self._pose),
                ArmControlMarker._pose2array(self._arm.get_ee_state()),
                0.01):
            rospy.loginfo("Move to cb from marker for real")

            self._lock.acquire()
            pose = ArmControlMarker._copy_pose(self._pose)
            self._lock.release()

            target_pose = pose

            if target_pose is not None:
                # time_to_pose = self._arm.get_time_to_pose(self.get_pose())

                thread = threading.Thread(
                    group=None,
                    target=self._arm.move_to_pose,
                    args=(target_pose,),
                    name='move_to_arm_state_thread'
                )
                thread.start()

                # Log
                # side_str = self._arm.side()
                rospy.loginfo('Started thread to move arm.')
            else:
                rospy.loginfo('Will not move arm; unreachable.')
        else:
            rospy.loginfo("move too small?")

    def _move_pose_to_cb(self, feedback):
        '''Callback for moving gripper marker to current pose.

        Args:
            feedback (InteractiveMarkerFeedback): Unused
        '''
        self.reset()
        self.update()

    def _is_reachable(self):
        '''Checks and returns whether there is an IK solution for this
        marker pose

        Returns:
            bool: Whether this action step is reachable.
        '''

        self._lock.acquire()
        pose = ArmControlMarker._copy_pose(self._pose)
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
        return self._name

    def _is_hand_open(self):
        '''Returns whether the gripper is open

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
            name (str): Name for the control
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
        mesh1 = ArmControlMarker._make_mesh_marker(mesh_color)
        mesh1.mesh_resource = STR_GRIPPER_PALM_FILE
        mesh1.pose.position.x = DEFAULT_OFFSET
        mesh1.pose.orientation.w = 1

        # TODO (sarah): make all of these numbers into constants
        if is_hand_open:
            mesh2 = ArmControlMarker._make_mesh_marker(mesh_color)
            mesh2.mesh_resource = STR_L_GRIPPER_FINGER_FILE
            mesh2.pose.position.x = 0.08
            mesh2.pose.position.y = -0.165
            mesh2.pose.orientation.w = 1

            mesh3 = ArmControlMarker._make_mesh_marker(mesh_color)
            mesh3.mesh_resource = STR_R_GRIPPER_FINGER_FILE
            mesh3.pose.position.x = 0.08
            mesh3.pose.position.y = 0.165
            mesh3.pose.orientation.w = 1
        else:
            mesh2 = ArmControlMarker._make_mesh_marker(mesh_color)
            mesh2.mesh_resource = STR_L_GRIPPER_FINGER_FILE
            mesh2.pose.position.x = 0.08
            mesh2.pose.position.y = -0.116
            mesh2.pose.orientation.w = 1

            mesh3 = ArmControlMarker._make_mesh_marker(mesh_color)
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
