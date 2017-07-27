'''Defines behaviour for Grasp primitive.
'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# ROS builtins
import tf
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Vector3, Point, Pose, Quaternion, PoseStamped
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers.menu_handler import MenuHandler
from rail_manipulation_msgs.srv import SuggestGrasps, SuggestGraspsRequest
from rail_manipulation_msgs.msg import GraspFeedback

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
COLOR_MESH_REACHABLE = ColorRGBA(0.0, 0.0, 1.0, 0.6)
COLOR_MESH_REACHABLE_SELECTED = ColorRGBA(0.0, 1.0, 0.0, 1.0)

COLOR_MESH_UNREACHABLE = ColorRGBA(0.5, 0.5, 0.5, 0.7)
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
    'ref': 'Change target object:',
    # 'move_here': 'Move arm here',
    # 'move_current': 'Move to current arm pose',
    'del': 'Delete',
    'regen': 'Regenerate grasps',
    'gen' : 'Generate grasps',
    'choice' : 'Switch grasp to:'

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
EE_LINK = 'wrist_roll_link'

# ######################################################################
# Classes
# ######################################################################

class Grasp(Primitive):
    '''Defines behaviour for Grasp primitive. This is for primitives
    where the arm moves to a single pose.
    '''

    _offset = DEFAULT_OFFSET

    def __init__(self, robot, tf_listener, im_server, 
                    grasp_suggestion_service_name=None, 
                    grasp_feedback_topic=None, 
                    external_ee_link=None, 
                    landmark=None, number=None):
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
        self._name = '' #Unused currently
        self._im_server = im_server
        self._robot = robot
        self._number = number

        self._head_busy = False
        self._external_ee_link = external_ee_link
        self._is_control_visible = False
        self._selected = False
        self._tf_listener = tf_listener
        self._marker_visible = False
        self._color_mesh_reachable = COLOR_MESH_REACHABLE
        self._color_mesh_unreachable = COLOR_MESH_UNREACHABLE
        self._grasp_reachable = False
        self._pre_grasp_reachable = False
        self._grasp_state = ArmState()
        self._grasp_state.ref_landmark = landmark
        self._grasp_state.ref_type = ArmState.OBJECT
        self._pre_grasp_state = ArmState()
        self._pre_grasp_state.ref_landmark = landmark
        self._pre_grasp_state.ref_type = ArmState.OBJECT
        self._current_grasp_num = None
        self._current_grasp_list = []
        self._approach_dist = 0.1 # default value
        self._landmark_found = False

        self._menu_handler = MenuHandler()

        self._sub_entries = None
        self._grasp_menu_entries = None
        self._marker_click_cb = None
        self._marker_delete_cb = None
        self._pose_change_cb = None
        self._action_change_cb = None
        self._viewed_grasps = []

        self._get_object_from_name_srv = rospy.ServiceProxy(
                                         '/fetch_pbd/get_object_from_name',
                                         GetObjectFromName)
        self._get_most_similar_obj_srv = rospy.ServiceProxy(
                                         '/fetch_pbd/get_most_similar_object',
                                         GetMostSimilarObject)
        self._get_object_list_srv = rospy.ServiceProxy(
                                        '/fetch_pbd/get_object_list',
                                        GetObjectList)
        self._grasp_suggestion_srv = \
                rospy.ServiceProxy(grasp_suggestion_service_name, SuggestGrasps)
        self._grasp_feedback_publisher = rospy.Publisher(grasp_feedback_topic,
                                                        GraspFeedback,
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
        json['grasp_state'] = Grasp._get_json_from_arm_state(self._grasp_state)
        json['pre_grasp_state'] = Grasp._get_json_from_arm_state(
                                                        self._pre_grasp_state)

        return {'grasp': json}

    def build_from_pose(self, pose_stamped, landmark, 
                        approach_dist=0.1, name=''):
        '''Fills out Grasp using information from David's grasp suggestion'''
        rospy.loginfo("Building from pose: {}".format(pose_stamped))

        self._name = name # Unused
        self._grasp_state.ref_type = ArmState.OBJECT
        new_pose = self._tf_listener.transformPose(landmark.name,
                                               pose_stamped)
        self._grasp_state.ee_pose = new_pose
        self._grasp_state.ref_landmark = landmark

        self._approach_dist = approach_dist

        

        # I think this will make a pre-grasp that's approach_dist 
        # away from the grasp along the x axis of the grasp
        self._set_pre_grasp_state_from_pose(new_pose)

        self._pre_grasp_state.ref_type = ArmState.OBJECT
        
        self._pre_grasp_state.ref_landmark = landmark

    def build_from_json(self, json):
        '''Fills out Grasp using json from db'''

        rospy.loginfo("Json: {}".format(json))

        self._name = json['name']
        self._number = json['number']
        grasp_state_json = json['grasp_state']
        pre_grasp_state_json = json['pre_grasp_state']

        # build self._arm_state
        self._grasp_state = ArmState()
        self._pre_grasp_state = ArmState()
        self._grasp_state.ref_type = grasp_state_json['ref_type']
        self._pre_grasp_state.ref_type = pre_grasp_state_json['ref_type']
        self._grasp_state.joint_pose = grasp_state_json['joint_pose']
        self._pre_grasp_state.joint_pose = pre_grasp_state_json['joint_pose']
        grasp_pose = Grasp._get_pose_stamped_from_json(
                                    grasp_state_json['ee_pose'])
        pre_grasp_pose = Grasp._get_pose_stamped_from_json(
                                    pre_grasp_state_json['ee_pose'])
        self._grasp_state.ee_pose = grasp_pose
        self._pre_grasp_state.ee_pose = pre_grasp_pose

        self._grasp_state.ref_landmark = Landmark()
        self._pre_grasp_state.ref_landmark = Landmark()
        landmark_name = grasp_state_json['ref_landmark']['name']
        self._grasp_state.ref_landmark.name = landmark_name
        self._pre_grasp_state.ref_landmark.name = landmark_name

        landmark_pose_json = grasp_state_json['ref_landmark']['pose']
        landmark_pose = Grasp._get_pose_from_json(landmark_pose_json)
        self._grasp_state.ref_landmark.pose = landmark_pose
        self._pre_grasp_state.ref_landmark.pose = landmark_pose

        landmark_dimensions = Vector3()
        x_dim = grasp_state_json['ref_landmark']['dimensions']['x']
        y_dim = grasp_state_json['ref_landmark']['dimensions']['y']
        z_dim = grasp_state_json['ref_landmark']['dimensions']['z']
        landmark_dimensions.x = x_dim
        landmark_dimensions.y = y_dim
        landmark_dimensions.z = z_dim

        # build self._gripper_state
        self._grasp_state.ref_landmark.dimensions = landmark_dimensions
        self._pre_grasp_state.ref_landmark.dimensions = landmark_dimensions

    def check_pre_condition(self):
        ''' Currently just a placeholder
            Meant to return conditions that need to be met before a
            primitive can be executed. This could be something like
            "There should only be one object" or something.

            Returns:
                None
        '''
        if self._grasp_state.ref_type == ArmState.OBJECT:
            if not self._landmark_found:
                return False, "No matching object found" + \
                        " for primitive: {}".format(self.get_number())
        if self._current_grasp_num is None:
            msg = "Cannot execute action." + \
                            " No grasp chosen. Right-click the blue" + \
                            " grasp marker to generate grasp options."
            return False, msg
        else:
            return True, None

    def check_post_condition(self):
        ''' Currently just a placeholder
            Meant to return conditions that need to be met after a
            primitive is executed in order for execution to be a success.
            This could be something like "object must be 0.1m above surface"

            Returns:
                None
        '''

        return True, None

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
        rospy.loginfo("Showing marker")
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

        rospy.loginfo("Updating primitive reference frames")
        if self._grasp_state.ref_type == ArmState.OBJECT:
            prev_ref_obj = self._grasp_state.ref_landmark
            resp = self._get_most_similar_obj_srv(prev_ref_obj)
            if resp.has_similar:
                self._grasp_state.ref_landmark = resp.similar_object
                self._pre_grasp_state.ref_landmark = resp.similar_object
                self._landmark_found = True
                return True
            else:
                self._landmark_found = False
                return False
        else:
            rospy.logwarn("Grasp has non-OBJECT-type reference frame")
            return False

    def change_ref_frame(self, ref_type, landmark):
        '''Sets new reference frame for primitive

        Args:

        '''
        self._grasp_state.ref_type = ref_type
        self._pre_grasp_state.ref_type = ref_type

        self._convert_ref_frame(landmark)
        rospy.loginfo(
            "Switching reference frame for primitive " +
            self.get_name())
        self._menu_handler.reApply(self._im_server)
        self.update_viz(False)
        self._action_change_cb()
        self._im_server.applyChanges()

    def get_ref_frame_name(self):
        '''Returns the name string for the reference frame object of the
        primitive.

        Returns:
            str|None: Under all normal circumstances, returns the str
                reference frame name. Returns None in error.
        '''
        # "Normal" step (saved pose).
        # ref_type = self._grasp_state.ref_type
        ref_name = self._grasp_state.ref_landmark.name
        rospy.loginfo("Ref frame name: {}".format(ref_name))

        # Update ref frame name if it's absolute.
        # if ref_type == ArmState.ROBOT_BASE:
        #     ref_name = BASE_LINK
        # elif ref_type == ArmState.PREVIOUS_TARGET:
        #     ref_name = "primitive_" + str(self._number - 1)
        # elif ref_name == '':
        #     ref_name = BASE_LINK
        #     rospy.loginfo("Empty frame: {}".format(self._number))

        return ref_name

    def select(self, is_selected):
        '''Set whether primitive is selected or not

        Args:
            is_selected (bool)
        '''
        rospy.loginfo("Selecting primitive: {}".format(self.get_number()))
        self._selected = is_selected
        self.set_control_visible(is_selected)
        if is_selected:
            self._color_mesh_reachable = COLOR_MESH_REACHABLE_SELECTED
            self._color_mesh_unreachable = COLOR_MESH_UNREACHABLE_SELECTED
        else:
            self._color_mesh_reachable = COLOR_MESH_REACHABLE
            self._color_mesh_unreachable = COLOR_MESH_UNREACHABLE

        self.update_viz(False)

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
        rospy.loginfo("ee pose header: {}".format(self._grasp_state.ee_pose.header.frame_id))
        draw_markers = True
        if self._grasp_state.ref_type == ArmState.OBJECT:
            landmark_name = self._grasp_state.ref_landmark.name
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
        return True

    def execute(self):
        '''Execute this primitive, assumes that gripper commands just work

        Returns
            bool : Success of execution
        '''
        if self._current_grasp_num is None:
            msg = "Cannot execute action." + \
                            " No grasp chosen. Right-click the blue" + \
                            " grasp marker to generate grasp options."
            return False, msg
        else:
            rospy.loginfo("Executing grasp")
            if not self._robot.move_arm_to_pose(self._pre_grasp_state):
                return False, "Problem finding IK solution"
            if not self._robot.get_gripper_state() == GripperState.OPEN:
                self._robot.set_gripper_state(GripperState.OPEN)

            if not self._robot.move_arm_to_pose(self._grasp_state):
                return False, "Problem finding IK solution"
            if not self._robot.get_gripper_state() == GripperState.CLOSED:
                self._robot.set_gripper_state(GripperState.CLOSED)
            feedback_msg = GraspFeedback()
            feedback_msg.indices_considered = self._viewed_grasps
            feedback_msg.index_selected = self._current_grasp_num
            self._grasp_feedback_publisher.publish(feedback_msg)
            return True, None

    def head_busy(self):
        '''Return true if head busy

        Returns:
            bool
        '''
        return self._head_busy

    def is_reachable(self):
        '''Check if robot can physically reach target'''
        self._grasp_reachable = self._robot.can_reach(self._grasp_state)
        self._pre_grasp_reachable = self._robot.can_reach(self._grasp_state)
        if not self._grasp_reachable:
            rospy.logwarn("Grasp not reachable")
        if not self._pre_grasp_reachable:
            rospy.logwarn("Pre-grasp not reachable")
        return self._grasp_reachable and self._pre_grasp_reachable

    def get_relative_pose(self, use_final=True):
        '''Returns the absolute pose of the primitive.
        Args:
            use_final (bool, optional). If true, uses pose of grasp_state
            rather than pre_grasp_state
        Returns:
            PoseStamped
        '''
        if self._current_grasp_num is None:
            return PoseStamped(
                    header=self._grasp_state.ref_landmark.point_cloud.header, 
                    pose=self._grasp_state.ref_landmark.pose)
        else:
            if use_final:
                return self._grasp_state.ee_pose
            else:
                return self._pre_grasp_state.ee_pose

    def get_absolute_pose(self):
        '''Returns the absolute pose of the grasp part of the primitive
           (not the pre-grasp).

        Args:
            None

        Returns:
            PoseStamped
        '''
        if self._current_grasp_num is None:
            try:
                pose = PoseStamped(
                    header=self._grasp_state.ref_landmark.point_cloud.header, 
                    pose=self._grasp_state.ref_landmark.pose)
                abs_pose = self._tf_listener.transformPose('base_link',
                                                   pose)
                return abs_pose
            except Exception, e:
                landmark = self._grasp_state.ref_landmark
                frame_id = landmark.point_cloud.header.frame_id
                rospy.logwarn("Frame: {} does not exist".format(frame_id))
                rospy.logwarn(str(e))
                return None
        else:
            try:
                abs_pose = self._tf_listener.transformPose('base_link',
                                                   self._grasp_state.ee_pose)
                return abs_pose
            except Exception, e:
                frame_id = self._grasp_state.ee_pose.header.frame_id
                rospy.logwarn("Frame: {} does not exist".format(frame_id))
                rospy.logwarn(str(e))
                return None

    def get_absolute_marker_pose(self, use_final=True):
        '''Returns the absolute pose of the primitive marker.

        Args:
            use_final (bool, optional). Unused

        Returns:
            PoseStamped
        '''
        if self._current_grasp_num is None:
            try:
                height = self._grasp_state.ref_landmark.point_cloud.height
                width = self._grasp_state.ref_landmark.point_cloud.width
                if height * width == 0:
                    return None
                pose = PoseStamped(
                    header=self._grasp_state.ref_landmark.point_cloud.header, 
                    pose=self._grasp_state.ref_landmark.pose)
                abs_pose = self._tf_listener.transformPose('base_link',
                                                   pose)
                return abs_pose
            except Exception, e:
                landmark = self._grasp_state.ref_landmark
                frame_id = landmark.point_cloud.header.frame_id
                rospy.logwarn("Frame: {} does not exist".format(frame_id))
                rospy.logwarn(str(e))
                return None
        else:
            try:
                if use_final:
                    pose_to_use = self._grasp_state.ee_pose
                else:
                    pose_to_use = self._pre_grasp_state.ee_pose 
                abs_pose = self._tf_listener.transformPose('base_link',
                                                   pose_to_use)
                return Grasp._offset_pose(abs_pose)
            except Exception, e:
                if use_final:
                    pose_to_use = self._grasp_state.ee_pose
                else:
                    pose_to_use = self._pre_grasp_state.ee_pose
                frame_id = pose_to_use.pose.header.frame_id
                rospy.logwarn("Frame: {} does not exist".format(frame_id))
                rospy.logwarn(str(e))
                return None

    def get_absolute_marker_position(self, use_final=True):
        '''Returns the absolute position of the primitive marker.

        Args:
            use_final (bool, optional) : Unused
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
        rospy.loginfo("Setting new ee pose")
        self._grasp_state.ee_pose = new_pose
        self._set_pre_grasp_state_from_pose(new_pose)
        self.update_viz()
        self._action_change_cb()

    def pose_editable(self):
        '''Return whether pose of primitive is editable

        Returns:
            bool : True
        '''
        return False

    def get_ref_type(self):
        '''Return reference type of primitive

        Returns:
            ArmState.ROBOT_BASE, etc
        '''
        return ArmState.OBJECT

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
        pose = Grasp._get_json_from_pose_stamped(arm_state.ee_pose)
        json['ee_pose'] = pose
        landmark = arm_state.ref_landmark
        json['ref_landmark'] = Grasp._get_json_from_landmark(landmark)
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
        json['pose'] = Grasp._get_json_from_pose(pose_stamped.pose)
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
        json['pose'] = Grasp._get_json_from_pose(landmark.pose)
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
        pose = Grasp._get_pose_from_json(json['pose'])
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
                by (scales Grasp._offset). Defaults to 1.

        Returns:
            PoseStamped: The offset pose.
        '''
        transform = Grasp._get_matrix_from_pose(pose)
        offset_array = [constant * Grasp._offset, 0, 0]
        offset_transform = tf.transformations.translation_matrix(offset_array)
        hand_transform = tf.transformations.concatenate_matrices(
            transform, offset_transform)

        new_pose = PoseStamped()
        new_pose.header.frame_id = pose.header.frame_id
        new_pose.pose = Grasp._get_pose_from_transform(hand_transform)
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
                See Grasp as a reference for how to use.)

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
    def _generate_grasps_cb(self, feedback):
        '''Callback for when users want to generate grasps for an object'''
        self._suggest_grasps(self._grasp_state.ref_landmark)


    def _switch_grasp_cb(self, feedback):
        '''Callback to switch between grasps indicated by menu entries'''
        menu_id = feedback.menu_entry_id
        self._current_grasp_num = self._grasp_menu_entries.index(menu_id)
        self._viewed_grasps.append(self._current_grasp_num)
        grasp_pose = self._current_grasp_list[self._current_grasp_num]
        self.build_from_pose(grasp_pose, 
                                self._grasp_state.ref_landmark, 
                                name=self.get_name())
        self._pose_change_cb()


    def _suggest_grasps(self, landmark):
        '''Call grasp suggestion service and process results'''
        self._head_busy = True
        rospy.loginfo("Getting grasps")
        self._robot.look_down()
        req = SuggestGraspsRequest()
        req.cloud = landmark.point_cloud
        rospy.loginfo("Point cloud size: {}".format(
                        req.cloud.height*req.cloud.width))
        rospy.loginfo("PC header: {}".format(landmark.point_cloud.header))
        resp = self._grasp_suggestion_srv(landmark.point_cloud)
        grasps = resp.grasp_list
        if not len(grasps.poses) > 0:
            rospy.logwarn("No grasps received")
            self._robot.look_forward()
            self._head_busy = False
            return False
        else:
            self._current_grasp_list = \
                    [PoseStamped(header=grasps.header, pose=p) \
                     for p in grasps.poses]
            self._change_grasp_frames(EE_LINK)
            pose_stamped = self._current_grasp_list[0]
            self.build_from_pose(pose_stamped, 
                                    landmark,
                                    name=self.get_name())
            rospy.loginfo("Current grasps: {}".format(
                            len(self._current_grasp_list)))
            self._current_grasp_num = 0
            self._pose_change_cb()
            self._head_busy = False
            self._robot.look_forward()
            self._viewed_grasps.append(0)
            return True

    def _change_grasp_frames(self, target_frame):
        '''Change frames of grasps in self._current_grasp_list'''

        if not self._current_grasp_list:
            return
        else:
            pose_frame = self._external_ee_link
            self._tf_listener.waitForTransform(pose_frame,
                                 target_frame,
                                 rospy.Time(0),
                                 rospy.Duration(4.0))
            (trans_diff, rot_diff) = \
                self._tf_listener.lookupTransform(pose_frame, 
                                                    target_frame, 
                                                    rospy.Time(0))
            new_list = []
        
            for pose in self._current_grasp_list:

                rot_mat = Grasp._get_matrix_from_pose(pose)
                x_axis = Vector3(rot_mat[0, 0], rot_mat[1, 0], rot_mat[2, 0])
                y_axis = Vector3(rot_mat[0, 1], rot_mat[1, 1], rot_mat[2, 1])
                z_axis = Vector3(rot_mat[0, 2], rot_mat[1, 2], rot_mat[2, 2])

                pose_temp = PoseStamped()
                pose_temp.header.frame_id = BASE_LINK
                pose_temp.pose.position.x = pose.pose.position.x + \
                                            x_axis.x*trans_diff[0] + \
                                            y_axis.x*trans_diff[1] + \
                                            z_axis.x*trans_diff[2]
                pose_temp.pose.position.y = pose.pose.position.y + \
                                            x_axis.y*trans_diff[0] + \
                                            y_axis.y*trans_diff[1] + \
                                            z_axis.y*trans_diff[2]
                pose_temp.pose.position.z = pose.pose.position.z + \
                                            x_axis.z*trans_diff[0] + \
                                            y_axis.z*trans_diff[1] + \
                                            z_axis.z*trans_diff[2]

                q = [pose.pose.orientation.x, pose.pose.orientation.y, 
                        pose.pose.orientation.z, pose.pose.orientation.w]
                q_temp = tf.transformations.quaternion_multiply(q, rot_diff)
                pose_temp.pose.orientation.x = q_temp[0]
                pose_temp.pose.orientation.y = q_temp[1]
                pose_temp.pose.orientation.z = q_temp[2]
                pose_temp.pose.orientation.w = q_temp[3]
                rospy.loginfo("Pose: {}".format(pose))
                rospy.loginfo("Pose temp: {}".format(pose_temp))
                new_list.append(pose_temp)
            self._current_grasp_list = new_list

    def _update_menu(self):
        '''Recreates the menu when something has changed.'''

        rospy.loginfo("Making new menu")
        self._menu_handler = MenuHandler()

        # Insert sub entries.
        self._sub_entries = []
        frame_entry = self._menu_handler.insert(MENU_OPTIONS['ref'])
        object_list = self._get_object_list_srv().object_list
        refs = [obj.name for obj in object_list]
        for ref in refs:
            subent = self._menu_handler.insert(
                ref, parent=frame_entry, callback=self._change_ref_cb)
            self._sub_entries += [subent]

        # Inset main menu entries.
        self._menu_handler.insert(
            MENU_OPTIONS['del'], callback=self._delete_primitive_cb)
        
        self._grasp_menu_entries = []
        if self._current_grasp_list:
            self._menu_handler.insert(
                MENU_OPTIONS['regen'], callback=self._regenerate_grasps_cb)
            grasp_choice_entry = self._menu_handler.insert( 
                                                MENU_OPTIONS['choice'])
            for i in range(len(self._current_grasp_list)):
                grasp_ent = self._menu_handler.insert("grasp_" + str(i),
                                        parent=grasp_choice_entry,
                                        callback=self._switch_grasp_cb)
                self._grasp_menu_entries += [grasp_ent]

            for grasp_ent in self._grasp_menu_entries:
                self._menu_handler.setCheckState(grasp_ent, 
                                                MenuHandler.UNCHECKED)
            if not self._current_grasp_num is None:
                self._menu_handler.setCheckState(
                            self._grasp_menu_entries[self._current_grasp_num], 
                            MenuHandler.CHECKED)

        else:
            self._menu_handler.insert(
                MENU_OPTIONS['gen'], callback=self._generate_grasps_cb)


        # Make all unchecked to start.
        for subent in self._sub_entries:
            self._menu_handler.setCheckState(subent, MenuHandler.UNCHECKED)

        # Check if necessary.
        menu_id = self._get_menu_id(self._get_menu_ref())
        if not menu_id is None:
            # self.has_object = False
            self._menu_handler.setCheckState(menu_id, MenuHandler.CHECKED)

    def _get_menu_id(self, ref_name):
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
            if index < len(self._sub_entries):
                return self._sub_entries[index]
            else:
                return None
        else:
            return None

    def _get_menu_name(self, menu_id):
        '''Returns the menu name from its unique menu id.

        Args:
            menu_id (int)
        Returns:
            str
        '''
        index = self._sub_entries.index(menu_id)
        object_list = self._get_object_list_srv().object_list
        refs = [obj.name for obj in object_list]
        if self._number > 0:
            refs.append(PREVIOUS_PRIMITIVE)
        refs.append(BASE_LINK)
        return refs[index]

    def _set_ref(self, new_ref):
        '''Changes the reference frame of the primitive to
        new_ref_name.

        Args:
            new_ref_name
        '''
        # Get the id of the new ref (an int).
        self._grasp_state.ref_type = ArmState.OBJECT
        self._pre_grasp_state.ref_type = ArmState.OBJECT
        new_ref_obj = self._get_object_from_name_srv(new_ref).obj
        rospy.loginfo("Setting reference of primitive" + 
                      "{} to object".format(self._number))
        self._grasp_state.ref_landmark = new_ref_obj
        self._pre_grasp_state.ref_landmark = new_ref_obj
        self._grasp_state.ee_pose.header.frame_id = new_ref_obj.name
        self._landmark_found = True

    def _convert_ref_frame(self, new_landmark):
        '''Convert grasp_state and pre_grasp_state to be in a different 
           reference frame

            Args:
                new_landmark (Landmark)
            Returns:
                ArmState
        '''
        ee_pose = PoseStamped()
        if self._grasp_state.ref_type == ArmState.OBJECT:
            rospy.loginfo("Relative to object")
            if self._grasp_state.ref_landmark.name != new_landmark.name:
                ee_pose = self._tf_listener.transformPose(
                                    new_landmark.name,
                                    self._grasp_state.ee_pose
                                )
                self._grasp_state.ref_landmark = new_landmark
                self._grasp_state.ee_pose = ee_pose
                self._pre_grasp_state.ref_landmark = new_landmark
                self._landmark_found = True

        elif self._grasp_state.ref_type == ArmState.ROBOT_BASE:
            ee_pose = self._tf_listener.transformPose(
                                    BASE_LINK,
                                    self._grasp_state.ee_pose
                                )
            self._grasp_state.ee_pose = ee_pose
            self._grasp_state.ref_landmark = Landmark()
            self._pre_grasp_state.ref_landmark = Landmark()
            self._landmark_found = False

        elif self._grasp_state.ref_type == ArmState.PREVIOUS_TARGET:
            prev_frame_name = "primitive_" + str(self._number - 1)
            rospy.loginfo("Original pose: {}".format(self._grasp_state.ee_pose))
            ee_pose = self._tf_listener.transformPose(
                                    prev_frame_name,
                                    self._grasp_state.ee_pose
                                )
            rospy.loginfo("New pose: {}".format(ee_pose))

            self._grasp_state.ee_pose = ee_pose
            self._grasp_state.ref_landmark = Landmark()
            self._pre_grasp_state.ref_landmark = Landmark()
            self._landmark_found = False

        self._set_pre_grasp_state_from_pose(ee_pose)

    def _set_pre_grasp_state_from_pose(self, pose_stamped):
        '''Sets pre_grasp_state based on a pose_stamped msg'''

        rot_mat = Grasp._get_matrix_from_pose(pose_stamped)
        x_axis = Vector3(rot_mat[0, 0], rot_mat[1, 0], rot_mat[2, 0])
        self._pre_grasp_state.ee_pose = PoseStamped()
        self._pre_grasp_state.ee_pose.header.frame_id = \
                            pose_stamped.header.frame_id
        self._pre_grasp_state.ee_pose.pose.orientation.x = \
                            pose_stamped.pose.orientation.x
        self._pre_grasp_state.ee_pose.pose.orientation.y = \
                            pose_stamped.pose.orientation.y
        self._pre_grasp_state.ee_pose.pose.orientation.z = \
                            pose_stamped.pose.orientation.z
        self._pre_grasp_state.ee_pose.pose.orientation.w = \
                            pose_stamped.pose.orientation.w
        self._pre_grasp_state.ee_pose.pose.position.x = \
                            pose_stamped.pose.position.x \
                            - (x_axis.x * self._approach_dist)
        self._pre_grasp_state.ee_pose.pose.position.y = \
                            pose_stamped.pose.position.y \
                            - (x_axis.y * self._approach_dist)
        self._pre_grasp_state.ee_pose.pose.position.z = \
                            pose_stamped.pose.position.z \
                            - (x_axis.z * self._approach_dist)

    def _get_marker_pose(self):
        '''Returns the pose of the marker for the primitive.

        Returns:
            Pose
        '''
        try:
            self._tf_listener.waitForTransform(BASE_LINK,
                                 self._grasp_state.ee_pose.header.frame_id,
                                 rospy.Time(0),
                                 rospy.Duration(4.0))
            intermediate_pose = self._tf_listener.transformPose(
                                                    BASE_LINK,
                                                    self._grasp_state.ee_pose)
            offset_pose = Grasp._offset_pose(intermediate_pose)
            return self._tf_listener.transformPose(self.get_ref_frame_name(),
                                                offset_pose)
        except Exception, e:
            rospy.logwarn(e)
            rospy.logwarn(
                "Frame not available yet: {}".format(self.get_ref_frame_name()))
            return None

    def _update_viz_core(self, check_reachable=True):
        '''Updates visualization after a change.

        Args:
            check_reachable (bool) : Check reachability of 
            pose before drawing marker
        '''
        # Create a new IM control.
        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        frame_id = self.get_ref_frame_name()
        if self._current_grasp_num is None:
            pose = PoseStamped()
            pose.pose.orientation.w = 1.0
            marker = Marker()
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.scale = self._grasp_state.ref_landmark.dimensions
            marker.pose = Pose()
            marker.pose.orientation.w = 1.0
            if self._selected:
                marker.color = COLOR_MESH_REACHABLE_SELECTED
            else:
                marker.color = COLOR_MESH_REACHABLE
            menu_control.markers.append(marker)
        else:
            pose = self._get_marker_pose()
            if pose is None:
                return

            if check_reachable and not self._current_grasp_num is None:
                self.is_reachable()

            menu_control = self._make_gripper_markers(
                   menu_control)

        # Make and add interactive marker.
        int_marker = InteractiveMarker()
        int_marker.name = self.get_name()
        int_marker.header.frame_id = frame_id
        int_marker.pose = pose.pose
        int_marker.scale = INT_MARKER_SCALE
        #self._add_6dof_marker(int_marker, True)
        rospy.loginfo("Marker name: {}".format(self.get_name()))
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
        rospy.loginfo("Setting new pose for grasp primitive")
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose = new_pose
        pose_stamped_transformed = self._tf_listener.transformPose(
                                                    self.get_ref_frame_name(),
                                                    pose_stamped)
        self._grasp_state.ee_pose = Grasp._offset_pose(
                                                    pose_stamped_transformed,
                                                    -1)
        self._set_pre_grasp_state_from_pose(self._grasp_state.ee_pose)
        self.update_viz()

    def _make_gripper_markers(self, control):
        '''Makes a gripper marker, adds it to control, returns control.

        Args:
            control (InteractiveMarkerControl): IM Control we're using.
            is_hand_open (bool, optional): Whether the gripper is open.
                Defaults to False (closed).

        Returns:
            InteractiveMarkerControl: The passed control.
        '''

        if self._grasp_reachable:
            grasp_mesh_color = self._color_mesh_reachable
        else:
            grasp_mesh_color = self._color_mesh_unreachable

        if self._pre_grasp_reachable:
            pre_grasp_mesh_color = self._color_mesh_reachable
        else:
            pre_grasp_mesh_color = self._color_mesh_unreachable

        rospy.loginfo("Mesh color: {}".format(grasp_mesh_color))


        # Make grasp marker

        # Create mesh 1 (palm).
        grasp_mesh1 = Grasp._make_mesh_marker(grasp_mesh_color)
        grasp_mesh1.mesh_resource = STR_GRIPPER_PALM_FILE
        grasp_mesh1.pose.position.x = Grasp._offset
        grasp_mesh1.pose.orientation.w = 1

        # Fingers
        grasp_mesh2 = Grasp._make_mesh_marker(grasp_mesh_color)
        grasp_mesh2.mesh_resource = STR_L_GRIPPER_FINGER_FILE
        grasp_mesh2.pose.position.x = 0.08
        grasp_mesh2.pose.position.y = -0.116
        grasp_mesh2.pose.orientation.w = 1

        grasp_mesh3 = Grasp._make_mesh_marker(grasp_mesh_color)
        grasp_mesh3.mesh_resource = STR_R_GRIPPER_FINGER_FILE
        grasp_mesh3.pose.position.x = 0.08
        grasp_mesh3.pose.position.y = 0.116
        grasp_mesh3.pose.orientation.w = 1

        # make pre-grasp marker 
        pre_grasp_mesh1 = Grasp._make_mesh_marker(pre_grasp_mesh_color)
        pre_grasp_mesh1.mesh_resource = STR_GRIPPER_PALM_FILE
        pre_grasp_mesh1.pose.position.x = Grasp._offset - self._approach_dist
        pre_grasp_mesh1.pose.orientation.w = 1

        pre_grasp_mesh2 = Grasp._make_mesh_marker(pre_grasp_mesh_color)
        pre_grasp_mesh2.mesh_resource = STR_L_GRIPPER_FINGER_FILE
        pre_grasp_mesh2.pose.position.x = 0.08 - self._approach_dist
        pre_grasp_mesh2.pose.position.y = -0.165 
        pre_grasp_mesh2.pose.orientation.w = 1

        pre_grasp_mesh3 = Grasp._make_mesh_marker(pre_grasp_mesh_color)
        pre_grasp_mesh3.mesh_resource = STR_R_GRIPPER_FINGER_FILE
        pre_grasp_mesh3.pose.position.x = 0.08 - self._approach_dist
        pre_grasp_mesh3.pose.position.y = 0.165
        pre_grasp_mesh3.pose.orientation.w = 1

        # Append all meshes we made.
        control.markers.append(grasp_mesh1)
        control.markers.append(grasp_mesh2)
        control.markers.append(grasp_mesh3)
        control.markers.append(pre_grasp_mesh1)
        control.markers.append(pre_grasp_mesh2)
        control.markers.append(pre_grasp_mesh3)

        return control

    def _delete_primitive_cb(self, feedback):
        '''Callback for when delete is requested.

        Args:
            feedback (InteractiveMarkerFeedback): Unused
        '''
        self._marker_delete_cb(self._number)

    def _regenerate_grasps_cb(self, feedback):
        '''Callback for regenerating grasps upon request

        Args:
            feedback (InteractiveMarkerFeedback): Unused
        '''
        self.hide_marker()
        resp = self._get_object_from_name_srv(self.get_ref_frame_name())
        self._suggest_grasps(resp.obj)
        self.update_viz()
        self.show_marker()
        self._action_change_cb()
        self._pose_change_cb()

    def _move_to_cb(self, feedback):
        '''Callback for when moving to a pose is requested.

        Args:
            feedback (InteractiveMarkerFeedback): Unused
        '''
        # for now, "move to" this primitive will just mean execute 
        self.execute()

    def _change_ref_cb(self, feedback):
        '''Callback for when a reference frame change is requested.

        Args:
            feedback (InteractiveMarkerFeedback (?))
        '''
        self._menu_handler.setCheckState(
            self._get_menu_id(self._get_menu_ref()), MenuHandler.UNCHECKED)
        self._menu_handler.setCheckState(
            feedback.menu_entry_id, MenuHandler.CHECKED)
        new_ref = self._get_menu_name(feedback.menu_entry_id)
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
            rospy.logdebug("Changing selected-ness.")
            self._is_control_visible = not self._is_control_visible
            self._marker_click_cb(
                self._number, self._is_control_visible)
        else:
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            rospy.logdebug("Unknown event: " + str(feedback.event_type))

    def _get_menu_ref(self):
        '''Returns the name string for the reference frame object of the
        primitive. This is specifically for

        Returns:
            str|None: Under all normal circumstances, returns the str
                reference frame name. Returns None in error.
        '''
        # "Normal" step (saved pose).
        ref_type = self._grasp_state.ref_type
        ref_name = self._grasp_state.ref_landmark.name

        # Update ref frame name if it's absolute.
        if ref_type == ArmState.ROBOT_BASE:
            ref_name = BASE_LINK
        elif ref_type == ArmState.PREVIOUS_TARGET:
            ref_name = PREVIOUS_PRIMITIVE
        elif ref_name == '':
            ref_name = BASE_LINK
            rospy.loginfo("Empty frame: {}".format(self._number))

        return ref_name

