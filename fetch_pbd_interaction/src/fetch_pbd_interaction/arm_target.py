'''Defines behaviour for ArmTarget primitive. This is for primitives
where the arm moves to a single pose.
'''

# ######################################################################
# Imports
# ######################################################################
# System builtins
import math

# Core ROS imports come first.
import rospy

# ROS builtins
import tf
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Vector3, Point, Pose, Quaternion, PoseStamped
from visualization_msgs.msg import Marker, InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers.menu_handler import MenuHandler

# Local
from fetch_pbd_interaction.primitive import Primitive
from fetch_arm_control.msg import GripperState
from fetch_pbd_interaction.msg import ArmState, Landmark
from fetch_pbd_interaction.srv import GetObjectList, \
                                      GetObjectFromName, GetMostSimilarObject

# ######################################################################
# Module level constants
# ######################################################################

# Marker options
# --------------
# Colors
# COLOR_OBJ_REF_ARROW = ColorRGBA(1.0, 0.8, 0.2, 0.5)
COLOR_MESH_REACHABLE = ColorRGBA(1.0, 0.5, 0.0, 0.6)
COLOR_MESH_REACHABLE_SELECTED = ColorRGBA(0.0, 1.0, 0.0, 1.0)

COLOR_MESH_UNREACHABLE = ColorRGBA(0.5, 0.5, 0.5, 0.6)
COLOR_MESH_UNREACHABLE_SELECTED = ColorRGBA(0.5, 0.9, 0.5, 0.4)

# Scales
SCALE_TRAJ_STEP_SPHERES = Vector3(0.02, 0.02, 0.02)
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
    'target': 'Change target object:'
}

# Offets to maintain globally-unique IDs but with new sets of objects.
# Each action step marker has a unique ID, and this allows each to
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
PREVIOUS_PRIMITIVE = "previous primitive"

# ######################################################################
# Classes
# ######################################################################

class ArmTarget(Primitive):
    '''Defines behaviour for ArmTarget primitive. This is for primitives
    where the arm moves to a single pose.
    '''

    _offset = DEFAULT_OFFSET

    def __init__(self, robot, tf_listener, im_server, arm_state=ArmState(),
                 gripper_state=None, number=None):
        '''
        Args:
            robot (Robot) : interface to lower level robot functionality
            tf_listener (TransformListener)
            im_server (InteractiveMarkerSerever)
            arm_state (ArmState, optional)
            gripper_state (GripperState.OPEN|GripperState.CLOSED, optional)
            number (int, optional): The number of this primitive in the
            action sequence
        '''
        self._name = "" #Unused currently
        self._im_server = im_server
        self._robot = robot
        self._arm_state = arm_state
        self._gripper_state = gripper_state
        self._number = number
        self._is_control_visible = False
        self._selected = False
        self._tf_listener = tf_listener
        self._marker_visible = False
        self._color_mesh_reachable = COLOR_MESH_REACHABLE
        self._color_mesh_unreachable = COLOR_MESH_UNREACHABLE
        self._reachable = True

        # self._ref_names = []
        # self._im_server = InteractiveMarkerServer("programmed_actions")
        self._menu_handler = MenuHandler()

        self._sub_ref_entries = None
        self._marker_click_cb = None
        self._marker_delete_cb = None
        self._pose_change_cb = None
        self._action_change_cb = None

        self._get_object_from_name_srv = rospy.ServiceProxy(
                                         'get_object_from_name',
                                         GetObjectFromName)
        self._get_most_similar_obj_srv = rospy.ServiceProxy(
                                         'get_most_similar_object',
                                         GetMostSimilarObject)
        self._get_object_list_srv = rospy.ServiceProxy(
                                        'get_object_list',
                                        GetObjectList)
        self._status_publisher = rospy.Publisher('fetch_pbd_status',
                                                String,
                                                queue_size=10,
                                                latch=True)

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def get_json(self):
        '''Returns json representation of primitive'''

        json = {}

        json['name'] = self._name
        json['number'] = self._number
        json['arm_state'] = ArmTarget._get_json_from_arm_state(self._arm_state)
        json['gripper_state'] = self._gripper_state

        return {'arm_target': json}

    def build_from_json(self, json):
        '''Fills out ArmTarget using json from db'''

        self._name = json['name']
        self._number = json['number']
        arm_state_json = json['arm_state']

        # build self._arm_state
        self._arm_state = ArmState()
        self._arm_state.ref_type = arm_state_json['ref_type']
        self._arm_state.joint_pose = arm_state_json['joint_pose']
        pose = ArmTarget._get_pose_stamped_from_json(arm_state_json['ee_pose'])
        self._arm_state.ee_pose = pose

        self._arm_state.ref_landmark = Landmark()
        landmark_name = arm_state_json['ref_landmark']['name']
        self._arm_state.ref_landmark.name = landmark_name

        landmark_pose_json = arm_state_json['ref_landmark']['pose']
        landmark_pose = ArmTarget._get_pose_from_json(landmark_pose_json)
        self._arm_state.ref_landmark.pose = landmark_pose

        landmark_dimensions = Vector3()
        x_dim = arm_state_json['ref_landmark']['dimensions']['x']
        y_dim = arm_state_json['ref_landmark']['dimensions']['y']
        z_dim = arm_state_json['ref_landmark']['dimensions']['z']
        landmark_dimensions.x = x_dim
        landmark_dimensions.y = y_dim
        landmark_dimensions.z = z_dim

        # build self._gripper_state
        self._gripper_state = json['gripper_state']

        self._arm_state.ref_landmark.dimensions = landmark_dimensions

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
        self._marker_visible = False
        if self.update_ref_frames():
            try:
                self._update_menu()
                self._update_viz_core()
                self._menu_handler.apply(self._im_server, self.get_name())
                self._im_server.applyChanges()
                self._marker_visible = True
            except Exception, e:
                rospy.logwarn(e)

        return self._marker_visible

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

        arm_pose = self._arm_state
        if arm_pose.ref_type == ArmState.OBJECT:
            prev_ref_obj = arm_pose.ref_landmark
            # rospy.loginfo("prev ref object: {}".format(prev_ref_obj))
            resp = self._get_most_similar_obj_srv(prev_ref_obj)
            if resp.has_similar:
                self._arm_state.ref_landmark = resp.similar_object
                return True
            else:
                return False
        else:
            return True

    def change_ref_frame(self, ref_type, landmark):
        '''Sets new reference frame for primitive

        Args:

        '''
        self._arm_state.ref_type = ref_type
        # self._arm_state.ref_landmark = landmark

        self._convert_ref_frame(landmark)
        rospy.loginfo(
            'Switching reference frame for primitive ' +
            self.get_name())
        self._menu_handler.reApply(self._im_server)
        self._im_server.applyChanges()
        self.update_viz(False)
        self._action_change_cb()

    def get_ref_frame_name(self):
        '''Returns the name string for the reference frame object of the
        primitive.

        Returns:
            str|None: Under all normal circumstances, returns the str
                reference frame name. Returns None in error.
        '''
        # "Normal" step (saved pose).
        ref_type = self._arm_state.ref_type
        ref_name = self._arm_state.ref_landmark.name


        # Update ref frame name if it's absolute.
        if ref_type == ArmState.ROBOT_BASE:
            ref_name = BASE_LINK
        elif ref_type == ArmState.PREVIOUS_TARGET:
            ref_name = "primitive_" + str(self._number - 1)
        elif ref_name == '':
            ref_name = BASE_LINK
            rospy.loginfo("Empty frame: {}".format(self._number))

        return ref_name

    def select(self, is_selected):
        '''Set whether primitive is selected or not

        Args:
            is_selected (bool)
        '''
        self._selected = is_selected
        self.set_control_visible(is_selected)
        if is_selected:
            self._color_mesh_reachable = COLOR_MESH_REACHABLE_SELECTED
            self._color_mesh_unreachable = COLOR_MESH_UNREACHABLE_SELECTED
        else:
            self._color_mesh_reachable = COLOR_MESH_REACHABLE
            self._color_mesh_unreachable = COLOR_MESH_UNREACHABLE

        # self.update_viz(False)

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
            check_reachable (bool) : whether to evaluate reachability
                                    before drawing marker
        '''
        rospy.loginfo("Updating viz for: {}".format(self.get_name()))
        draw_markers = True
        if self._arm_state.ref_type == ArmState.OBJECT:
            landmark_name = self._arm_state.ref_landmark.name
            resp = self._get_object_from_name_srv(landmark_name)
            if not resp.has_object:
                draw_markers = False

        if draw_markers and self._marker_visible:
            try:
                self._update_menu()
                if self._update_viz_core(check_reachable):
                    self._menu_handler.apply(self._im_server, self.get_name())
                    self._im_server.applyChanges()
            except Exception, e:
                rospy.logwarn("Error when updating primitive viz")
                rospy.logwarn(e)

    def get_primitive_number(self):
        '''Returns what number this primitive is in the sequence

        Returns:
            int
        '''
        return self._number

    def set_primitive_number(self, number):
        '''Sets what number this primitive is in the sequence

        Args:
            num (int)
        '''
        self._number = number

    def is_object_required(self):
        '''Check if this primitive requires an object to be present

        Returns:
            bool
        '''

        is_required = False
        ref = self._arm_state.ref_type

        if ref == ArmState.OBJECT:
            is_required = True

        return is_required

    def execute(self):
        '''Execute this primitive

        Returns
            bool : Success of execution
        '''
        if not self._robot.move_arm_to_pose(self._arm_state):
            return False
        if not self._gripper_state == self._robot.get_gripper_state():
            self._robot.set_gripper_state(self._gripper_state)
        return True

    def head_busy(self):
        '''Return true if head busy

        Returns:
            bool
        '''
        return False

    def is_reachable(self):
        '''Check if robot can physically reach target'''
        self._reachable = self._robot.can_reach(self._arm_state)
        return self._reachable

    def get_relative_pose(self, use_final=True):
        '''Returns the relative pose of the primitive.
        Args:
            use_final (bool, optional) : Unused
        Returns:
            PoseStamped
        '''
        return self._arm_state.ee_pose


    def get_absolute_pose(self):
        '''Returns the absolute pose of the primitive.

        Args:
            None

        Returns:
            PoseStamped
        '''
        try:
            abs_pose = self._tf_listener.transformPose('base_link',
                                               self._arm_state.ee_pose)
            return abs_pose
        except:
            frame_id = self._arm_state.ee_pose.header.frame_id
            rospy.logwarn("Frame: {} does not exist".format(frame_id))
            return None

    def get_absolute_marker_pose(self, use_final=True):
        '''Returns the absolute pose of the primitive marker.

        Args:
            use_final (bool, optional). Unused

        Returns:
            PoseStamped
        '''
        try:
            abs_pose = self._tf_listener.transformPose('base_link',
                                               self._arm_state.ee_pose)
            return ArmTarget._offset_pose(abs_pose)
        except:
            frame_id = self._arm_state.ee_pose.header.frame_id
            # rospy.logwarn("Frame: {} does not exist".format(frame_id))
            return None

    def get_absolute_marker_position(self, use_final=True):
        '''Returns the absolute position of the primitive marker.

        Args:
            use_final (bool, optional) : Unused
                True.

        Returns:
            Point
        '''
        abs_pose = self.get_absolute_marker_pose()
        if not abs_pose is None:
            return abs_pose.pose.position
        else:
            return None

    def decrease_id(self):
        '''Reduces the number of the primitive.'''
        self._number -= 1

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

    def set_pose(self, new_pose):
        '''Changes the pose of the primitive to new_pose.

        Args:
            new_pose (PoseStamped)
        '''
        self._arm_state.ee_pose = new_pose
        self.update_viz()
        self._action_change_cb()

    def pose_editable(self):
        '''Return whether pose of primitive is editable

        Returns:
            bool : True
        '''
        return True

    def get_ref_type(self):
        '''Return reference type of primitive

        Returns:
            ArmState.ROBOT_BASE, etc
        '''
        return self._arm_state.ref_type

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
        pose = ArmTarget._get_json_from_pose_stamped(arm_state.ee_pose)
        json['ee_pose'] = pose
        landmark = arm_state.ref_landmark
        json['ref_landmark'] = ArmTarget._get_json_from_landmark(landmark)
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
        json['pose'] = ArmTarget._get_json_from_pose(pose_stamped.pose)
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
        json['pose'] = ArmTarget._get_json_from_pose(landmark.pose)
        json['dimensions'] = {}
        json['dimensions']['x'] = landmark.dimensions.x
        json['dimensions']['y'] = landmark.dimensions.y
        json['dimensions']['z'] = landmark.dimensions.z

        return json

    @staticmethod
    def _get_pose_stamped_from_json(json):
        '''Return PoseStamped msg from json

        Args:
            json (dict)
        Returns:
            PoseStamped
        '''
        pose_stamped = PoseStamped()
        pose = ArmTarget._get_pose_from_json(json['pose'])
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
    def _diff(float1, float2, thresh):
        '''Return true if floats are different by a threshold

        Args:
            float1 (float)
            float2 (float)
            thresh (float)
        '''
        if math.fabs(float1 - float2) < thresh:
            return False
        else:
            return True

    @staticmethod
    def _offset_pose(pose, constant=1):
        '''Offsets the world pose for visualization.

        Args:
            pose (PoseStamped): The pose to offset.
            constant (int, optional): How much to scale the set offset
                by (scales ArmTarget._offset). Defaults to 1.

        Returns:
            PoseStamped: The offset pose.
        '''
        transform = ArmTarget._get_matrix_from_pose(pose)
        offset_array = [constant * ArmTarget._offset, 0, 0]
        offset_transform = tf.transformations.translation_matrix(offset_array)
        hand_transform = tf.transformations.concatenate_matrices(
            transform, offset_transform)

        new_pose = PoseStamped()
        new_pose.header.frame_id = pose.header.frame_id
        new_pose.pose = ArmTarget._get_pose_from_transform(hand_transform)
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
                See ArmTarget as a reference for how to use.)

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
    def _make_mesh_marker(color):
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

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _update_menu(self):
        '''Recreates the menu when something has changed.'''
        rospy.loginfo("Making new menu")
        self._menu_handler = MenuHandler()

        # Insert sub entries.
        self._sub_ref_entries = []
        frame_entry = self._menu_handler.insert(MENU_OPTIONS['ref'])
        object_list = self._get_object_list_srv().object_list
        refs = [obj.name for obj in object_list]
        if self._number > 0:
            refs.append(PREVIOUS_PRIMITIVE)
        if len(refs) > 0:
            refs.append(BASE_LINK)
        for ref in refs:
            subent = self._menu_handler.insert(
                ref, parent=frame_entry, callback=self._change_ref_cb)
            self._sub_ref_entries += [subent]


        # Make all unchecked to start.
        for subent in self._sub_ref_entries:
            self._menu_handler.setCheckState(subent, MenuHandler.UNCHECKED)

        # Check if necessary.
        menu_id = self._get_menu_ref_id(self._get_menu_ref())
        if not menu_id is None:
            # self.has_object = False
            self._menu_handler.setCheckState(menu_id, MenuHandler.CHECKED)


        self._sub_target_entries = []
        target_entry = self._menu_handler.insert(MENU_OPTIONS['target'])
        object_list = self._get_object_list_srv().object_list
        targets = [obj.name + " " for obj in object_list]
        for target in targets:
            subent = self._menu_handler.insert(
                target, parent=target_entry, callback=self._change_target_cb)
            self._sub_target_entries += [subent]

        # Make all unchecked to start.
        for subent in self._sub_target_entries:
            self._menu_handler.setCheckState(subent, MenuHandler.UNCHECKED)

        # Check if necessary.
        menu_id = self._get_menu_target_id(self._get_menu_ref(target=True))
        if not menu_id is None:
            # self.has_object = False
            self._menu_handler.setCheckState(menu_id, MenuHandler.CHECKED)

        # Inset main menu entries.

        self._menu_handler.insert(
            MENU_OPTIONS['move_here'], callback=self._move_to_cb)
        self._menu_handler.insert(
            MENU_OPTIONS['move_current'], callback=self._move_pose_to_cb)
        self._menu_handler.insert(
            MENU_OPTIONS['del'], callback=self._delete_primitive_cb)

        # Update.

    def _get_menu_ref_id(self, ref_name):
        '''Returns the unique menu id from its name or None if the
        object is not found.

        Args:
            ref_name (str)
        Returns:
            int (?)|None
        '''
        object_list = self._get_object_list_srv().object_list
        refs = [obj.name for obj in object_list]
        if self._number > 0:
            refs.append(PREVIOUS_PRIMITIVE)
        refs.append(BASE_LINK)
        if ref_name in refs:
            index = refs.index(ref_name)
            if index < len(self._sub_ref_entries):
                return self._sub_ref_entries[index]
            else:
                return None
        else:
            return None

    def _get_menu_target_id(self, ref_name):
        '''Returns the unique menu id from its name or None if the
        object is not found.

        Args:
            ref_name (str)
        Returns:
            int (?)|None
        '''
        object_list = self._get_object_list_srv().object_list
        refs = [obj.name + " " for obj in object_list]
        if ref_name in refs:
            index = refs.index(ref_name)
            if index < len(self._sub_target_entries):
                return self._sub_target_entries[index]
            else:
                return None
        else:
            return None


    def _get_menu_ref_name(self, menu_id):
        '''Returns the menu name from its unique menu id.

        Args:
            menu_id (int)
        Returns:
            str
        '''
        index = self._sub_ref_entries.index(menu_id)
        object_list = self._get_object_list_srv().object_list
        refs = [obj.name for obj in object_list]
        if self._number > 0:
            refs.append(PREVIOUS_PRIMITIVE)
        refs.append(BASE_LINK)
        return refs[index]

    def _get_menu_target_name(self, menu_id):
        '''Returns the menu name from its unique menu id.

        Args:
            menu_id (int)
        Returns:
            str
        '''
        index = self._sub_target_entries.index(menu_id)
        object_list = self._get_object_list_srv().object_list
        refs = [obj.name for obj in object_list]
        return refs[index]


    def _set_target(self, new_ref):
        '''Changes the reference frame of the primitive to
        new_ref_name.

        Args:
            new_ref_name
        '''
        # Get the id of the new ref (an int).
        self._arm_state.ref_type = ArmState.OBJECT
        new_ref_obj = self._get_object_from_name_srv(new_ref).obj
        rospy.loginfo("Setting reference of primitive" + 
                      "{} to object".format(self._number))
        self._arm_state.ref_landmark = new_ref_obj
        self._arm_state.ee_pose.header.frame_id = new_ref_obj.name

    def _set_ref(self, new_ref):
        '''Changes the reference frame of the primitive to
        new_ref_name.

        Args:
            new_ref_name
        '''
        # Get the id of the new ref (an int).
        new_ref_obj = Landmark()
        if new_ref == PREVIOUS_PRIMITIVE:
            self._arm_state.ref_type = ArmState.PREVIOUS_TARGET
            rospy.loginfo("PREVIOUS!!!")
        elif new_ref == BASE_LINK:
            self._arm_state.ref_type = ArmState.ROBOT_BASE
        else:
            self._arm_state.ref_type = ArmState.OBJECT
            new_ref_obj = self._get_object_from_name_srv(new_ref).obj
            rospy.loginfo("OBJECT!!!")

        self._convert_ref_frame(new_ref_obj)

    def _convert_ref_frame(self, new_landmark):
        '''Convert arm_state to be in a different reference frame

            Args:
                new_landmark (Landmark)
            Returns:
                ArmState
        '''
        if self._arm_state.ref_type == ArmState.OBJECT:
            rospy.loginfo("Relative to object")
            if self._arm_state.ref_landmark.name != new_landmark.name:
                ee_pose = self._tf_listener.transformPose(
                                    new_landmark.name,
                                    self._arm_state.ee_pose
                                )
                self._arm_state.ref_landmark = new_landmark
                self._arm_state.ee_pose = ee_pose
        elif self._arm_state.ref_type == ArmState.ROBOT_BASE:
            ee_pose = self._tf_listener.transformPose(
                                    BASE_LINK,
                                    self._arm_state.ee_pose
                                )
            self._arm_state.ee_pose = ee_pose
            self._arm_state.ref_landmark = Landmark()
        elif self._arm_state.ref_type == ArmState.PREVIOUS_TARGET:
            prev_frame_name = 'primitive_' + str(self._number - 1)
            rospy.loginfo("Original pose: {}".format(self._arm_state.ee_pose))
            ee_pose = self._tf_listener.transformPose(
                                    prev_frame_name,
                                    self._arm_state.ee_pose
                                )
            rospy.loginfo("New pose: {}".format(ee_pose))

            self._arm_state.ee_pose = ee_pose
            self._arm_state.ref_landmark = Landmark()

    def _get_marker_pose(self):
        '''Returns the pose of the primitive.

        Returns:
            Pose
        '''
        try:
            self._tf_listener.waitForTransform(BASE_LINK,
                                 self._arm_state.ee_pose.header.frame_id,
                                 rospy.Time(0),
                                 rospy.Duration(4.0))
            intermediate_pose = self._tf_listener.transformPose(
                                                        BASE_LINK,
                                                        self._arm_state.ee_pose)
            offset_pose = ArmTarget._offset_pose(intermediate_pose)
            return self._tf_listener.transformPose(self.get_ref_frame_name(),
                                                offset_pose)
        except Exception, e:
            rospy.logwarn(e)
            rospy.logwarn("Frame not available yet: {}".format(self.get_ref_frame_name()))
            return None

    def _update_viz_core(self, check_reachable=True):
        '''Updates visualization after a change.

        Args:
            check_reachable (bool) : Check reachability of pose before drawing marker
        '''
        # Create a new IM control.
        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        frame_id = self.get_ref_frame_name()
        pose = self._get_marker_pose()
        if pose is None:
            return

        if check_reachable:
            self.is_reachable()

        menu_control = self._make_gripper_marker(
               menu_control, self._gripper_state)

        # Make and add interactive marker.
        int_marker = InteractiveMarker()
        int_marker.name = self.get_name()
        int_marker.header.frame_id = frame_id
        int_marker.pose = pose.pose
        int_marker.scale = INT_MARKER_SCALE
        self._add_6dof_marker(int_marker, True)
        int_marker.controls.append(menu_control)
        prev_marker = self._im_server.get(self.get_name())
        prev_color = None
        if not prev_marker is None:
            if len(prev_marker.controls) > 0:
                if len(prev_marker.controls[-1].markers) > 0:
                    prev_color = prev_marker.controls[-1].markers[-1].color
        new_color = None
        if len(int_marker.controls) > 0:
            if len(int_marker.controls[-1].markers) > 0:
                new_color = int_marker.controls[-1].markers[-1].color

        if not prev_marker:
            self._im_server.insert(
                int_marker, self._marker_feedback_cb)
            rospy.logwarn("Adding marker for primitive {}".format(self.get_number()))
            return True
        elif (prev_marker.pose != int_marker.pose) or (prev_color != new_color):
            rospy.loginfo("Updating marker")
            self._im_server.insert(
                int_marker, self._marker_feedback_cb)
            return True

        rospy.logwarn("Not updating marker for primitive {}".format(self.get_number()))
        return False

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

    def _set_new_pose(self, new_pose, frame_id):
        '''Changes the pose of the primitive to new_pose.

        Args:
            new_pose (Pose)
        '''
        rospy.loginfo("Setting new ee_pose!!!")
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose = new_pose
        pose_stamped_transformed = self._tf_listener.transformPose(
                                                    self.get_ref_frame_name(),
                                                    pose_stamped)
        self._arm_state.ee_pose = ArmTarget._offset_pose(
                                                    pose_stamped_transformed,
                                                    -1)
        self.update_viz()

    def _pose_changed(self, new_pose, frame_id):
        '''Check if marker pose has changed

        Args: 
            pose (Pose)
        '''
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose = new_pose
        pose_stamped_transformed = self._tf_listener.transformPose(
                                                    self.get_ref_frame_name(),
                                                    pose_stamped)
        offset_pose = ArmTarget._offset_pose(pose_stamped_transformed,
                                                    -1)
        if ArmTarget._diff(offset_pose.pose.position.x, 
                            self._arm_state.ee_pose.pose.position.x,
                            0.001) \
            or ArmTarget._diff(offset_pose.pose.position.y, 
                            self._arm_state.ee_pose.pose.position.y,
                            0.001) \
            or ArmTarget._diff(offset_pose.pose.position.z, 
                            self._arm_state.ee_pose.pose.position.z,
                            0.001) \
            or ArmTarget._diff(offset_pose.pose.orientation.x, 
                            self._arm_state.ee_pose.pose.orientation.x,
                            0.01) \
            or ArmTarget._diff(offset_pose.pose.orientation.y, 
                            self._arm_state.ee_pose.pose.orientation.y,
                            0.01) \
            or ArmTarget._diff(offset_pose.pose.orientation.z, 
                            self._arm_state.ee_pose.pose.orientation.z,
                            0.01) \
            or ArmTarget._diff(offset_pose.pose.orientation.w, 
                            self._arm_state.ee_pose.pose.orientation.w,
                            0.01):
            rospy.loginfo("New pose: {}".format(new_pose))
            rospy.loginfo("New pose transformed: {}".format(offset_pose))
            rospy.loginfo("Old pose: {}".format(self._arm_state.ee_pose))
            return True
        else:
            return False

    def _get_mesh_marker_color(self):
        '''Gets the color for the mesh marker (thing that looks like a
        gripper) for this primitive.

        A simple implementation of this will return one color for
        reachable poses, another for unreachable ones. Future
        implementations may provide further visual cues.

        Returns:
            ColorRGBA: The color for the gripper mesh for this arm target.
        '''
        if self._reachable:
            return self._color_mesh_reachable
        else:
            return self._color_mesh_unreachable

    def _make_gripper_marker(self, control, gripper_state=GripperState.CLOSED):
        '''Makes a gripper marker, adds it to control, returns control.

        Args:
            control (InteractiveMarkerControl): IM Control we're using.
            is_hand_open (bool, optional): Whether the gripper is open.
                Defaults to False (closed).

        Returns:
            InteractiveMarkerControl: The passed control.
        '''

        mesh_color = self._get_mesh_marker_color()
        # frame_id = self._get_menu_ref()

        # Create mesh 1 (palm).
        mesh1 = ArmTarget._make_mesh_marker(mesh_color)
        mesh1.mesh_resource = STR_GRIPPER_PALM_FILE
        mesh1.pose.position.x = ArmTarget._offset
        mesh1.pose.orientation.w = 1
        # mesh1.header.frame_id = frame_id

        # TODO (sarah): make all of these numbers into constants
        if gripper_state == GripperState.OPEN:
            mesh2 = ArmTarget._make_mesh_marker(mesh_color)
            mesh2.mesh_resource = STR_L_GRIPPER_FINGER_FILE
            mesh2.pose.position.x = 0.08
            mesh2.pose.position.y = -0.165
            mesh2.pose.orientation.w = 1
            # mesh2.header.frame_id = frame_id

            mesh3 = ArmTarget._make_mesh_marker(mesh_color)
            mesh3.mesh_resource = STR_R_GRIPPER_FINGER_FILE
            mesh3.pose.position.x = 0.08
            mesh3.pose.position.y = 0.165
            mesh3.pose.orientation.w = 1
            # mesh3.header.frame_id = frame_id
        else:
            mesh2 = ArmTarget._make_mesh_marker(mesh_color)
            mesh2.mesh_resource = STR_L_GRIPPER_FINGER_FILE
            mesh2.pose.position.x = 0.08
            mesh2.pose.position.y = -0.116
            mesh2.pose.orientation.w = 1
            # mesh2.header.frame_id = frame_id

            mesh3 = ArmTarget._make_mesh_marker(mesh_color)
            mesh3.mesh_resource = STR_R_GRIPPER_FINGER_FILE
            mesh3.pose.position.x = 0.08
            mesh3.pose.position.y = 0.116
            mesh3.pose.orientation.w = 1
            # mesh3.header.frame_id = frame_id

        # Append all meshes we made.
        control.markers.append(mesh1)
        control.markers.append(mesh2)
        control.markers.append(mesh3)

        # Return back the control.
        # TODO(mbforbes): Why do we do this?
        return control

    def _delete_primitive_cb(self, feedback):
        '''Callback for when delete is requested.

        Args:
            feedback (InteractiveMarkerFeedback): Unused
        '''
        self._marker_delete_cb(self._number)

    def _move_to_cb(self, feedback):
        '''Callback for when moving to a pose is requested.

        Args:
            feedback (InteractiveMarkerFeedback): Unused
        '''
        self._robot.move_arm_to_pose(self._arm_state)

    def _move_pose_to_cb(self, feedback):
        '''Callback for when a pose change to current is requested.

        Args:
            feedback (InteractiveMarkerFeedback): Unused

        '''
        self._arm_state = self._robot.get_arm_state()
        self._gripper_state = self._robot.get_gripper_state()
        self.update_ref_frames()

    def _change_target_cb(self, feedback):
        '''Callback for when a reference frame change is requested.

        Args:
            feedback (InteractiveMarkerFeedback (?))
        '''
        self._menu_handler.setCheckState(
            self._get_menu_target_id(self._get_menu_ref(target=True)), MenuHandler.UNCHECKED)
        self._menu_handler.setCheckState(
            feedback.menu_entry_id, MenuHandler.CHECKED)
        new_ref = self._get_menu_target_name(feedback.menu_entry_id)
        self._set_target(new_ref)
        rospy.loginfo(
            'Switching reference frame to ' + new_ref + ' for primitive ' +
            self.get_name())
        self._menu_handler.reApply(self._im_server)
        self._im_server.applyChanges()
        self.update_viz(False)
        self._action_change_cb()

    def _change_ref_cb(self, feedback):
        '''Callback for when a reference frame change is requested.

        Args:
            feedback (InteractiveMarkerFeedback (?))
        '''
        self._menu_handler.setCheckState(
            self._get_menu_ref_id(self._get_menu_ref()), MenuHandler.UNCHECKED)
        self._menu_handler.setCheckState(
            feedback.menu_entry_id, MenuHandler.CHECKED)
        new_ref = self._get_menu_ref_name(feedback.menu_entry_id)
        self._set_ref(new_ref)
        rospy.loginfo(
            'Switching reference frame to ' + new_ref + ' for primitive ' +
            self.get_name())
        self._menu_handler.reApply(self._im_server)
        self._im_server.applyChanges()
        self.update_viz(False)
        self._action_change_cb()

    def _marker_feedback_cb(self, feedback):
        '''Callback for when an event occurs on the marker.

        Args:
            feedback (InteractiveMarkerFeedback)
        '''
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            # Set the visibility of the 6DOF controller.
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            if self._pose_changed(feedback.pose, feedback.header.frame_id):
                rospy.loginfo('Pose change.')
                self._set_new_pose(feedback.pose, feedback.header.frame_id)
                self._pose_change_cb()
                self._action_change_cb()
            else:
                rospy.loginfo('Changing visibility of the pose controls.')
                self._is_control_visible = not self._is_control_visible
                self._marker_click_cb(
                    self._number, self._is_control_visible)
                self._set_new_pose(feedback.pose, feedback.header.frame_id)
                self._pose_change_cb()
                self._action_change_cb()
        else:
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            rospy.logdebug('Unknown event: ' + str(feedback.event_type))

    def _get_menu_ref(self, target=False):
        '''Returns the name string for the reference frame object of the
        primitive. 

        Returns:
            str|None: Under all normal circumstances, returns the str
                reference frame name. Returns None in error.
        '''
        # "Normal" step (saved pose).
        ref_type = self._arm_state.ref_type
        ref_name = self._arm_state.ref_landmark.name

        if not target:
            # Update ref frame name if it's absolute.
            if ref_type == ArmState.ROBOT_BASE:
                ref_name = BASE_LINK
            elif ref_type == ArmState.PREVIOUS_TARGET:
                ref_name = PREVIOUS_PRIMITIVE
            elif ref_name == '':
                ref_name = BASE_LINK
                rospy.loginfo("Empty frame: {}".format(self._number))
        else:
            ref_name = ref_name + " "

        return ref_name



