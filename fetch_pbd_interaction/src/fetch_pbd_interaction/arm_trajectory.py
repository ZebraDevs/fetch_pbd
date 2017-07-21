'''Defines behaviour for ArmTrajectory primitive. This is for primitives
where the arm moves to a series of poses.
'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# System builtins
from collections import Counter

# ROS builtins
import tf
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Vector3, Point, Pose, Quaternion, PoseStamped
from visualization_msgs.msg import Marker, InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers.menu_handler import MenuHandler

# Local
from fetch_pbd_interaction.primitive import Primitive
from fetch_pbd_interaction.msg import ArmState, Landmark
from fetch_pbd_interaction.srv import GetObjectList, \
                                      GetObjectFromName, GetMostSimilarObject


# ######################################################################
# Module level constants
# ######################################################################

# Marker options
# --------------
# Colors
COLOR_TRAJ_ENDPOINT_SPHERES = ColorRGBA(1.0, 0.5, 0.0, 0.8)
COLOR_TRAJ_LINE = ColorRGBA(0.8, 0.4, 0.0, 0.8)
COLOR_TRAJ_ENDPOINT_SPHERES_SELECTED = ColorRGBA(0.0, 1.0, 0.0, 0.8)
COLOR_TRAJ_LINE_SELECTED = ColorRGBA(0.0, 1.0, 0.0, 0.8)
# COLOR_OBJ_REF_ARROW = ColorRGBA(1.0, 0.8, 0.2, 0.5)

# Scales
SCALE_TRAJ_LINE = Vector3(0.02, 0.0, 0.0)
SCALE_OBJ_REF_ARROW = Vector3(0.02, 0.03, 0.04)

# Gripper mesh related
STR_MESH_GRIPPER_FOLDER = 'package://fetch_description/meshes/'
STR_GRIPPER_PALM_FILE = STR_MESH_GRIPPER_FOLDER + 'gripper_link.STL'
STR_L_GRIPPER_FINGER_FILE = STR_MESH_GRIPPER_FOLDER + \
                                'l_gripper_finger_link.STL'
STR_R_GRIPPER_FINGER_FILE = STR_MESH_GRIPPER_FOLDER + \
                                'r_gripper_finger_link.STL'
DEFAULT_OFFSET = 0.085

# Right-click menu.
MENU_OPTIONS = {
    'ref': 'Reference frame',
    'move_here': 'Move arm here',
    'move_current': 'Move to current arm pose',
    'del': 'Delete',
}

# Offets to maintain globally-unique IDs but with new sets of objects.
# Each primitive marker has a unique ID, and this allows each to
# have a matching unique id for trajectories, text, etc. Assumes we'll
# have < 1k steps.
ID_OFFSET_REF_ARROW = 1000
ID_OFFSET_TRAJ_FIRST = 2000
ID_OFFSET_TRAJ_LAST = 3000

# Other
TRAJ_MARKER_LIFETIME = rospy.Duration()
TEXT_Z_OFFSET = 0.1
INT_MARKER_SCALE = 0.2
TRAJ_ENDPOINT_SCALE = 0.05

# ROS topics, etc.
# ----------------
# Namespace for interactive marker server.
TOPIC_IM_SERVER = 'programmed_actions'

# We might want to refactor this even further, as it's used throughout
# the code.
BASE_LINK = 'base_link'
FIXED_LINK = 'torso_lift_link'

# ######################################################################
# Classes
# ######################################################################

class ArmTrajectory(Primitive):
    '''Defines behaviour for ArmTrajectory primitive. This is for primitives
    where the arm moves to a series of poses.
    '''

    _offset = DEFAULT_OFFSET

    def __init__(self, robot, tf_listener, im_server, number=None):
        '''
        Args:
            robot (Robot) : interface to lower level robot functionality
            tf_listener (TransformListener)
            im_server (InteractiveMarkerSerever)
            number (int, optional): The number of this primitive in the
            action sequence
        '''
        self._name = '' # Unused currently
        self._robot = robot
        self._im_server = im_server
        self._start_time = rospy.Time.now()
        self._tf_listener = tf_listener
        self._is_control_visible = False
        self._is_selected = False
        self._marker_visible = False
        self._color_traj_line = COLOR_TRAJ_LINE
        self._color_traj_endpoint_spheres = COLOR_TRAJ_ENDPOINT_SPHERES

        # Timing is not currently used but the current implementation
        # for executing trajectories is very bad and I think in future
        # implementations this will be very useful
        self._timing = []
        self._menu_handler = MenuHandler()

        self._time_offset = rospy.Duration(0.1)
        self._arm_states = []
        self._gripper_states = []
        self._number = number
        self._ref_type = ArmState.FIXED_LINK
        self._ref_landmark = Landmark()
        self._sub_entries = None
        self._marker_click_cb = None
        self._marker_delete_cb = None
        self._pose_change_cb = None
        self._action_change_cb = None

        self._get_object_from_name_srv = rospy.ServiceProxy(
                                         '/fetch_pbd/get_object_from_name',
                                         GetObjectFromName)
        self._get_most_similar_obj_srv = rospy.ServiceProxy(
                                         '/fetch_pbd/get_most_similar_object',
                                         GetMostSimilarObject)
        self._get_object_list_srv = rospy.ServiceProxy('/fetch_pbd/get_object_list',
                                                       GetObjectList)

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def get_json(self):
        '''Returns json representation of primitive'''

        json = {}

        json['name'] = self._name
        json['number'] = self._number
        json['timing'] = self._timing
        json['ref_type'] = self._ref_type
        json['ref_landmark'] = \
                    ArmTrajectory._get_json_from_landmark(self._ref_landmark)
        json['arm_states'] = []
        json['gripper_states'] = []

        for i in range(len(self._arm_states)):
            arm_state = ArmTrajectory._get_json_from_arm_state(
                                            self._arm_states[i])
            gripper_state = self._gripper_states[i]
            json['arm_states'].append(arm_state)
            json['gripper_states'].append(gripper_state)

        return {'arm_trajectory': json}

    def build_from_json(self, json):
        '''Fills out ArmTrajectory using json from db'''

        self._name = json['name']
        self._number = json['number']
        self._timing = json['timing']
        self._ref_type = json['ref_type']
        self._ref_landmark = \
                    ArmTrajectory._get_landmark_from_json(json['ref_landmark'])

        for i in range(len(json['arm_states'])):
            arm_state = ArmTrajectory._get_arm_state_from_json(
                                            json['arm_states'][i])
            self._arm_states.append(arm_state)

            gripper_state = json['gripper_states'][i]
            self._gripper_states.append(gripper_state)

    def get_pre_condition(self):
        ''' Currently just a placeholder
            Meant to return conditions that need to be met before a
            primitive can be executed. This could be something like
            "There should only be one object" or something.

            Returns:
                None
        '''

        return None

    def get_post_condition(self):
        ''' Currently just a placeholder
            Meant to return conditions that need to be met after a
            primitive is executed in order for execution to be a success.
            This could be something like "object must be 0.1m above surface"

            Returns:
                None
        '''
        return None

    def add_marker_callbacks(self, click_cb, delete_cb, pose_change_cb,
                    action_change_cb):
        '''Adds marker to world'''

        # rospy.loginfo("Making marker")
        self._marker_click_cb = click_cb
        self._marker_delete_cb = delete_cb
        self._pose_change_cb = pose_change_cb
        self._action_change_cb = action_change_cb
        # self.update_ref_frames()

    def show_marker(self):
        '''Adds marker for primitive'''
        if self.update_ref_frames():
            try:
                self._update_menu()
                self._update_viz_core()
                self._menu_handler.apply(self._im_server, self.get_name())
                self._im_server.applyChanges()
            except Exception, e:
                rospy.logwarn(e)

        self._marker_visible = True

    def hide_marker(self):
        '''Removes marker from the world.'''

        rospy.loginfo("Deleting marker for: {}".format(self.get_name()))
        self._im_server.erase(self.get_name())
        self._im_server.applyChanges()
        self._marker_visible = False

    def marker_visible(self):
        '''Return whether or not marker is visible

        Returns:
            bool
        '''
        return self._marker_visible

    def update_ref_frames(self):
        '''Updates and re-assigns coordinate frames when the world changes.'''

        if self._ref_type != ArmState.FIXED_LINK:
            rospy.logwarn("Trajectories can only be relative to the robot's base")
            return False
        else:
            return True

    def change_ref_frame(self, ref_type, landmark):
        '''Sets new reference frame for primitive

        Args:

        '''
        rospy.logwarn("Trajectories are always relative to the robot's base")

    def get_ref_frame_name(self):
        '''Returns the name string for the reference frame object of the
        primitive.

        Returns:
            str|None: Under all normal circumstances, returns the str
                reference frame name. Returns None in error.
        '''
        # "Normal" step (saved pose).
        return FIXED_LINK

    def select(self, is_selected):
        '''Set whether primitive is selected or not

        Args:
            is_selected (bool)
        '''
        self._selected = is_selected
        self.set_control_visible(is_selected)
        if is_selected:
            self._color_traj_line = COLOR_TRAJ_LINE_SELECTED
            self._color_traj_endpoint_spheres = \
                COLOR_TRAJ_ENDPOINT_SPHERES_SELECTED
        else:
            self._color_traj_line = COLOR_TRAJ_LINE
            self._color_traj_endpoint_spheres = COLOR_TRAJ_ENDPOINT_SPHERES
        self.update_viz()

    def is_selected(self):
        '''Return whether or not primitive is selected

        Returns
            bool
        '''
        return self._selected

    def is_control_visible(self):
        '''Check if the marker control is visible

        Returns
            bool
        '''
        return self._is_control_visible

    def set_control_visible(self, visible=True):
        '''Set visibility of marker controls

        Args:
            visible (bool, optional)
        '''
        self._is_control_visible = visible

    def update_viz(self, check_reachable=True):
        '''Updates visualization fully.

        Args:
            check_reachable (bool) : Unused
        '''
        if self._marker_visible:
            try:
                self._update_menu()
                self._update_viz_core()
                self._menu_handler.apply(self._im_server, self.get_name())
                self._im_server.applyChanges()
            except Exception, e:
                rospy.logwarn(e)

    def get_primitive_number(self):
        '''Returns what number this primitive is in the sequence'''
        return self._number

    def set_primitive_number(self, num):
        '''Sets what number this primitive is in the sequence

        Args:
            num (int)
        '''
        self._number = num

    def is_object_required(self):
        '''Check if this primitive requires an object to be present

        Returns:
            bool
        '''
        return False

    def execute(self):
        '''Execute this primitive

        Returns
            bool : Success of execution
        '''
        first_arm_state = self._arm_states[0]

        velocities = [0.2] * len(first_arm_state.joint_pose)

        first_arm_state.velocities = velocities
        # Sleep to let arm actually reach goal?
        rospy.sleep(0.2)
        self._robot.move_arm_to_pose(first_arm_state)
        self._robot.move_arm_to_joints_plan(first_arm_state)
        # Sleep to let arm actually reach goal?
        rospy.sleep(0.2)
        all_states = self._robot.move_arm_to_joints(self._arm_states,
                                                  self._timing)
        last_arm_state = self._arm_states[-1]

        velocities = [0.0] * len(last_arm_state.joint_pose)

        last_arm_state.velocities = velocities
        self._robot.move_arm_to_joints_plan(last_arm_state)


        gripper_state = self._gripper_states[-1]

        self._robot.set_gripper_state(gripper_state)
        return all_states

    def head_busy(self):
        '''Return true if head busy

        Returns:
            bool
        '''
        return False

    def is_reachable(self):
        '''Check if robot can physically reach all steps in trajectory'''
        for arm_state in self._arm_states:
            if not self._robot.can_reach(arm_state):
                return False
        return True

    def get_relative_pose(self, use_final=True):
        '''Returns the absolute pose of the primitive.
        Args:
            use_final (bool, optional). Whether to
                get the final pose in the trajectory. Defaults to True.
        Returns:
            PoseStamped
        '''
        index = len(self._arm_states) - 1 if use_final else 0
        arm_state = self._arm_states[index]

        return arm_state.ee_pose

    def get_absolute_pose(self):
        '''Returns the absolute pose of the primitive.

        Args:
            None

        Returns:
            PoseStamped
        '''
        index = len(self._arm_states) - 1 if use_final else 0
        arm_state = self._arm_states[index]

        try:
            # self._tf_listener.waitForTransform(BASE_LINK,
            #                  arm_state.ee_pose.header.frame_id,
            #                  rospy.Time.now(),
            #                  rospy.Duration(5.0))
            abs_pose = self._tf_listener.transformPose(BASE_LINK,
                                                   arm_state.ee_pose)
            return abs_pose
        except:
            frame_id = arm_state.ee_pose.header.frame_id
            rospy.logwarn("Frame: {} does not exist".format(frame_id))
            return None

    def get_absolute_marker_pose(self, use_final=True):
        '''Returns the absolute pose of the primitive marker.

        Args:
            use_final (bool, optional). For trajectories only. Whether to
                get the final pose in the trajectory. Defaults to True.

        Returns:
            PoseStamped
        '''
        index = len(self._arm_states) - 1 if use_final else 0
        arm_state = self._arm_states[index]

        try:
            # self._tf_listener.waitForTransform(BASE_LINK,
            #                  arm_state.ee_pose.header.frame_id,
            #                  rospy.Time.now(),
            #                  rospy.Duration(5.0))
            abs_pose = self._tf_listener.transformPose(BASE_LINK,
                                                   arm_state.ee_pose)
            return ArmTrajectory._offset_pose(abs_pose)
        except:
            frame_id = arm_state.ee_pose.header.frame_id
            rospy.logwarn("Frame: {} does not exist".format(frame_id))
            return None

    def get_absolute_marker_position(self, use_final=True):
        '''Returns the absolute position of the primitive marker.

        Args:
            use_final (bool, optional). For trajectories only. Whether to
                get the final position in the trajectory. Defaults to
                True.

        Returns:
            Point
        '''
        abs_pose = self.get_absolute_marker_pose(use_final)
        if not abs_pose is None:
            return abs_pose.pose.position
        else:
            return None

    def decrease_id(self):
        '''Reduces the number of the primitive.'''
        self._number -= 1
        # self._update_menu()

    def add_step(self, arm_state, gripper_state):
        '''Add step to trajectory

        Args:
            arm_state (ArmState)
            gripper_state (GripperState.OPEN|GripperState.CLOSED)
        '''
        # Adjusts durations so that they start at 0.1s
        # And adds 0.1s of padding between each step
        if not self._timing:
            self._time_offset = ((rospy.Time.now() -
                                 self._start_time).to_sec() - 0.1)
            self._timing = [0.1]
        else:
            self._timing.append((rospy.Time.now() -
                                self._start_time).to_sec() -
                                self._time_offset)
        arm_state = self._convert_ref_frame(arm_state)
        self._arm_states.append(arm_state)
        self._gripper_states.append(gripper_state)

    def set_name(self, name):
        '''Sets the display name for the primitive.

        Args:
            name (str) : A human-readable unique name for the primitive.
        '''
        self._name = name

    def get_name(self):
        '''Returns the display name for the primitive.

        Returns:
            str: A human-readable unique name for the primitive.
        '''
        return self._name

    def get_number(self):
        '''Returns number of primitive

        Returns:
            int
        '''
        return self._number

    def set_pose(self, pose):
        '''CHanging pose of trajectory is currently not supported

        Args:
            pose (PoseStamped) : Unused
        '''
        rospy.logwarn("Changing pose of trajectory is not currently supported")

    def pose_editable(self):
        '''Return whether pose of primitive is editable

        Returns:
            bool : False
        '''
        return False

    def get_ref_type(self):
        '''Return reference type of primitive

        Returns:
            ArmState.ROBOT_BASE, etc
        '''
        return self._ref_type

    # ##################################################################
    # Static methods: Internal ("private")
    # ##################################################################

    @staticmethod
    def _get_json_from_arm_state(arm_state):
        '''Return json containing data of arm_state

        Args:
            arm_state (ArmState)
        Returns:
            json (dict)
        '''
        json = {}
        json['ref_type'] = arm_state.ref_type
        json['joint_pose'] = arm_state.joint_pose
        pose = arm_state.ee_pose
        json['ee_pose'] = ArmTrajectory._get_json_from_pose_stamped(pose)
        landmark = ArmTrajectory._get_json_from_landmark(arm_state.ref_landmark)
        json['ref_landmark'] = landmark
        return json

    @staticmethod
    def _get_json_from_pose_stamped(pose_stamped):
        '''Return json containing data of pose_stamped

        Args:
            pose_stamped (PoseStamped)
        Returns:
            json (dict)
        '''
        json = {}
        json['pose'] = ArmTrajectory._get_json_from_pose(pose_stamped.pose)
        json['frame_id'] = pose_stamped.header.frame_id

        return json

    @staticmethod
    def _get_json_from_pose(pose):
        '''Return json containing data of pose

        Args:
            pose (Pose)
        Returns:
            json (dict)
        '''
        json = {}
        json['position'] = {}
        json['position']['x'] = pose.position.x
        json['position']['y'] = pose.position.y
        json['position']['z'] = pose.position.z

        json['orientation'] = {}
        json['orientation']['x'] = pose.orientation.x
        json['orientation']['y'] = pose.orientation.y
        json['orientation']['z'] = pose.orientation.z
        json['orientation']['w'] = pose.orientation.w

        return json

    @staticmethod
    def _get_json_from_landmark(landmark):
        '''Return json containing data of landmark

        Args:
            landmark (Landmark)
        Returns:
            json (dict)
        '''
        json = {}
        # json['type'] = landmark.type
        json['name'] = landmark.name
        json['pose'] = ArmTrajectory._get_json_from_pose(landmark.pose)
        json['dimensions'] = {}
        json['dimensions']['x'] = landmark.dimensions.x
        json['dimensions']['y'] = landmark.dimensions.y
        json['dimensions']['z'] = landmark.dimensions.z

        return json

    @staticmethod
    def _get_arm_state_from_json(json):
        '''Return ArmState msg from json

        Args:
            json (dict)
        Returns:
            ArmState
        '''
        arm_state = ArmState()
        arm_state.ref_type = json['ref_type']
        arm_state.joint_pose = json['joint_pose']
        pose = ArmTrajectory._get_pose_stamped_from_json(json['ee_pose'])
        arm_state.ee_pose = pose
        landmark = ArmTrajectory._get_landmark_from_json(json['ref_landmark'])
        arm_state.ref_landmark = landmark
        return arm_state

    @staticmethod
    def _get_landmark_from_json(json):
        '''Return Landmark msg from json

        Args:
            json (dict)
        Returns:
            Landmark
        '''
        landmark = Landmark()
        # landmark.type = json['type']
        landmark.name = json['name']

        landmark_pose = ArmTrajectory._get_pose_from_json(json['pose'])
        landmark.pose = landmark_pose

        landmark_dimensions = Vector3()
        landmark_dimensions.x = json['dimensions']['x']
        landmark_dimensions.y = json['dimensions']['y']
        landmark_dimensions.z = json['dimensions']['z']

        landmark.dimensions = landmark_dimensions

        return landmark

    @staticmethod
    def _get_pose_stamped_from_json(json):
        '''Return PoseStamped msg from json

        Args:
            json (dict)
        Returns:
            PoseStamped
        '''
        pose_stamped = PoseStamped()
        pose = ArmTrajectory._get_pose_from_json(json['pose'])
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = json['frame_id']

        return pose_stamped

    @staticmethod
    def _get_pose_from_json(json):
        '''Return Pose msg from json

        Args:
            json (dict)
        Returns:
            Pose
        '''
        pose = Pose()

        pose.position.x = json['position']['x']
        pose.position.y = json['position']['y']
        pose.position.z = json['position']['z']

        pose.orientation.x = json['orientation']['x']
        pose.orientation.y = json['orientation']['y']
        pose.orientation.z = json['orientation']['z']
        pose.orientation.w = json['orientation']['w']

        return pose

    @staticmethod
    def _offset_pose(pose, constant=1):
        '''Offsets the world pose for visualization.

        Args:
            pose (PoseStamped): The pose to offset.
            constant (int, optional): How much to scale the set offset
                by (scales ArmTrajectory._offset). Defaults to 1.

        Returns:
            PoseStamped: The offset pose.
        '''
        transform = ArmTrajectory._get_matrix_from_pose(pose)
        offset_array = [constant * ArmTrajectory._offset, 0, 0]
        offset_transform = tf.transformations.translation_matrix(offset_array)
        hand_transform = tf.transformations.concatenate_matrices(
            transform, offset_transform)

        new_pose = PoseStamped()
        new_pose.header.frame_id = pose.header.frame_id
        new_pose.pose = ArmTrajectory._get_pose_from_transform(hand_transform)
        return new_pose

    @staticmethod
    def _get_matrix_from_pose(pose):
        '''Returns the transformation matrix for given pose.

        Args:
            pose (PoseStamped)

        Returns:
            Matrix3x3
        '''
        position, orientation = pose.pose.position, pose.pose.orientation
        rot_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        transformation = tf.transformations.quaternion_matrix(rot_list)
        pos_list = [position.x, position.y, position.z]
        transformation[:3, 3] = pos_list
        return transformation

    @staticmethod
    def _get_pose_from_transform(transform):
        '''Returns pose for transformation matrix.

        Args:
            transform (Matrix3x3): (I think this is the correct type.
                See ArmTrajectory as a reference for how to use.)

        Returns:
            Pose
        '''
        pos = transform[:3, 3].copy()
        rot = tf.transformations.quaternion_from_matrix(transform)
        return Pose(
            Point(pos[0], pos[1], pos[2]),
            Quaternion(rot[0], rot[1], rot[2], rot[3])
        )

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _update_menu(self):
        '''Recreates the menu when something has changed.'''
        self._menu_handler = MenuHandler()

        # Insert sub entries.
        self._sub_entries = []
        # frame_entry = self._menu_handler.insert(MENU_OPTIONS['ref'])
        # object_list = self._get_object_list_srv().object_list
        # refs = [obj.name for obj in object_list]
        # if self._number > 0:
        #     refs.append(PREVIOUS_PRIMITIVE)
        # if len(refs) > 0:
        #     refs.append(BASE_LINK)
        # for ref in refs:
        #     subent = self._menu_handler.insert(
        #         ref, parent=frame_entry, callback=self._change_ref_cb)
        #     self._sub_entries += [subent]

        # Inset main menu entries.
        self._menu_handler.insert(
            MENU_OPTIONS['del'], callback=self._delete_primitive_cb)

        # Make all unchecked to start.
        # for subent in self._sub_entries:
        #     self._menu_handler.setCheckState(subent, MenuHandler.UNCHECKED)

        # Check if necessary.
        # menu_id = self._get_menu_id(self._get_menu_ref())
        # if not menu_id is None:
        #     # self.has_object = False
        #     self._menu_handler.setCheckState(menu_id, MenuHandler.CHECKED)

        # Update.
        # if self._update_viz_core():
        #     self._menu_handler.apply(self._im_server, self.get_name())
        #     self._im_server.applyChanges()

    def _convert_ref_frame(self, arm_state):
        '''Convert arm_state to be in a different reference frame

            Args:
                arm_state (ArmState)
                new_landmark (Landmark)
            Returns:
                ArmState
        '''
        ee_pose = self._tf_listener.transformPose(
                                FIXED_LINK,
                                arm_state.ee_pose
                            )
        arm_state.ee_pose = ee_pose

        return arm_state

    def _get_marker_pose(self):
        '''Returns the pose of the primitive.

        Returns:
            Pose
        '''
        try:
            i = int(len(self._arm_states) - 1)
            # self._tf_listener.waitForTransform(BASE_LINK,
            #                      self._arm_states[i].ee_pose.header.frame_id,
            #                      rospy.Time(0),
            #                      rospy.Duration(4.0))
            # intermediate_pose = self._tf_listener.transformPose(
            #                                         BASE_LINK,
            #                                         self._arm_states[i].ee_pose)
            offset_pose = ArmTrajectory._offset_pose(self._arm_states[i].ee_pose)
            return self._tf_listener.transformPose(self.get_ref_frame_name(),
                                                    offset_pose)
        except:
            rospy.logwarn("Frame not available yet.")
            return None

    def _update_viz_core(self):
        '''Updates visualization after a change.'''
        # Create a new IM control.
        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        frame_id = self.get_ref_frame_name()
        pose = None
        pose = self._get_marker_pose()
        if pose is None:
            return

        # Handle trajectories.
        # First, get all trajectory positions.
        point_list = []
        traj_pose = None
        try:
            self._tf_listener.waitForTransform(FIXED_LINK,
                             "primitive_" + str(self._number),
                             rospy.Time.now(),
                             rospy.Duration(1.0))
        except Exception, e:
            pass
        for j in range(len(self._timing)):
            traj_pose = self._get_traj_pose(j)
            if not traj_pose is None:
                point_list.append(traj_pose.pose.position)
            else:
                break

        if traj_pose is None:
            try:
                self._tf_listener.waitForTransform(FIXED_LINK,
                                 "primitive_" + str(self._number),
                                 rospy.Time.now(),
                                 rospy.Duration(4.0))
            except Exception, e:
                pass
            for j in range(len(self._timing)):
                traj_pose = self._get_traj_pose(j)
                if not traj_pose is None:
                    point_list.append(traj_pose.pose.position)
                else:
                    return

        rospy.loginfo("last pose: {}".format(self._arm_states[0].ee_pose))

        last_point = self._tf_listener.transformPose(FIXED_LINK, self._get_traj_pose(0))
        rospy.loginfo("last point: {}".format(last_point))

        # Add a main maker for all points in the trajectory (sphere
        # list).
        menu_control.markers.append(
            Marker(
                type=Marker.LINE_STRIP,
                id=self._number,
                lifetime=TRAJ_MARKER_LIFETIME,
                scale=SCALE_TRAJ_LINE,
                header=Header(frame_id=''),
                color=self._color_traj_line,
                points=point_list
            )
        )

        # Add a marker for the first step in the trajectory.
        menu_control.markers.append(
            self._make_sphere_marker(
                self._number + ID_OFFSET_TRAJ_FIRST,
                self._get_traj_pose(0).pose,
                '',
                TRAJ_ENDPOINT_SCALE
            )
        )

        # Add a marker for the last step in the trajectory.
        last_index = len(self._timing) - 1
        menu_control.markers.append(
            self._make_sphere_marker(
                self._number + ID_OFFSET_TRAJ_LAST,
                self._get_traj_pose(last_index).pose,
                '',
                TRAJ_ENDPOINT_SCALE
            )
        )

        # Add an arrow to the relative object, if there is one.
        # if not self._ref_type == ArmState.ROBOT_BASE:
        #     menu_control.markers.append(
        #         Marker(
        #             type=Marker.ARROW,
        #             id=(ID_OFFSET_REF_ARROW + self._number),
        #             lifetime=TRAJ_MARKER_LIFETIME,
        #             scale=SCALE_OBJ_REF_ARROW,
        #             header=Header(frame_id=frame_id),
        #             color=COLOR_OBJ_REF_ARROW,
        #             points=[pose.pose.position, Point(0, 0, 0)]
        #         )
        #     )

        # Make and add interactive marker.
        int_marker = InteractiveMarker()
        int_marker.name = self.get_name()
        int_marker.header.frame_id = frame_id
        int_marker.pose = pose.pose
        int_marker.scale = INT_MARKER_SCALE
        # self._add_6dof_marker(int_marker, True)
        int_marker.controls.append(menu_control)
        self._im_server.insert(
            int_marker, self._marker_feedback_cb)
        return

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
            control = self._make_6dof_control(name, orient, is_move, is_fixed)
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

    def _make_sphere_marker(self, uid, pose, frame_id, radius):
        '''Creates and returns a sphere marker.

        Args:
            uid (int): The id to use for the marker.
            pose (Pose): The pose of the marker.
            frame_id (str): Frame the marker is associated with. (See
                std_msgs/Header.msg for more info.)
            radius (float): Amount to scale the marker by. Scales in all
                directions (x, y, and z).

        Returns:
            Marker
        '''
        return Marker(
            type=Marker.SPHERE,
            id=uid,
            lifetime=rospy.Duration(2),
            scale=Vector3(radius, radius, radius),
            pose=pose,
            header=Header(frame_id=frame_id),
            color=self._color_traj_endpoint_spheres
        )

    def _delete_primitive_cb(self, feedback):
        '''Callback for when delete is requested.

        Args:
            feedback (InteractiveMarkerFeedback): Unused
        '''
        self._marker_delete_cb(self._number)

    def _get_traj_pose(self, index):
        '''Returns this trajectory's pose at index.

        Args:
            index (int): Which step in the trajectory to return the
                pose from.

        Returns:
            Pose
        '''
        try:
            # intermediate_pose = self._tf_listener.transformPose(
            #                             BASE_LINK,
            #                             self._arm_states[index].ee_pose)
            offset_pose = ArmTrajectory._offset_pose(
                                            self._arm_states[index].ee_pose)

            new_pose = self._tf_listener.transformPose(
                                            "primitive_" + str(self._number),
                                            offset_pose)
            return new_pose

        except Exception, e:
            rospy.logwarn(e)
            rospy.logwarn("Unable to transform marker to" +
                    "correct frame: {}".format("primitive_" + str(self._number)))
            return None

    def _marker_feedback_cb(self, feedback):
        '''Callback for when an event occurs on the marker.

        Args:
            feedback (InteractiveMarkerFeedback)
        '''
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.logwarn(
                'Modification of trajectory segments is not ' +
                'implemented.')
        elif feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            # Set the visibility of the 6DOF controller.
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            rospy.logdebug('Changing visibility of the pose controls.')
            self._is_control_visible = not self._is_control_visible
            self._marker_click_cb(
                self._number, self._is_control_visible)
        else:
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            rospy.logdebug('Unknown event: ' + str(feedback.event_type))
