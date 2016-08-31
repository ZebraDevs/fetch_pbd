'''Robot class that provides an interface to arm and head (maybe base later)'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy
import roslib

# System builtins
import os

# ROS builtins
from sound_play.libsoundplay import SoundClient
from std_srvs.srv import Empty
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

# Local
from fetch_pbd_interaction.srv import MoveArm, GetGripperState, \
                                      SetGripperState, GetArmMovement, \
                                      GetEEPose, GetJointStates, GetGazeGoal, \
                                      GetNearestObject, MoveArmTraj
from fetch_social_gaze.msg import GazeGoal, GazeAction
from fetch_pbd_interaction.msg import ArmState, Landmark
from fetch_arm_control.msg import GripperState

# ######################################################################
# Module level constants
# ######################################################################

# String for base link name
BASE_LINK = 'base_link'

PKG = 'fetch_pbd_interaction'

# Directory sounds are in, within the package.
SOUNDS_DIR = os.path.join(roslib.packages.get_pkg_dir(PKG), 'sounds', '')

# Common postfix for sound files.
SOUND_FILEFORMAT = '.wav'

# ######################################################################
# Classes
# ######################################################################

class Robot:
    '''Robot class that provides an interface to arm
    and head (maybe base later)'''

    def __init__(self, tf_listener):
        '''
        Args:
            tf_listener (TransformListener)
        '''
        self._social_gaze = rospy.get_param("/social_gaze")
        self._play_sound = rospy.get_param("/play_sound")
        self._tf_listener = tf_listener

        # arm services
        self.move_arm_to_joints_plan_srv = rospy.ServiceProxy(
                                        'move_arm_to_joints_plan', MoveArm)
        rospy.wait_for_service('move_arm_to_joints_plan')

        self.move_arm_to_joints_srv = rospy.ServiceProxy(
                                        'move_arm_to_joints', MoveArmTraj)
        rospy.wait_for_service('move_arm_to_joints')

        self.move_arm_to_pose_srv = rospy.ServiceProxy(
                                        'move_arm_to_pose', MoveArm)
        rospy.wait_for_service('move_arm_to_pose')

        self.start_move_arm_to_pose_srv = rospy.ServiceProxy(
                                            'start_move_arm_to_pose', MoveArm)
        rospy.wait_for_service('start_move_arm_to_pose')

        self.is_reachable_srv = rospy.ServiceProxy('is_reachable', MoveArm)
        rospy.wait_for_service('is_reachable')

        self.is_arm_moving_srv = rospy.ServiceProxy(
                                    'is_arm_moving', GetArmMovement)
        rospy.wait_for_service('is_arm_moving')

        self.relax_arm_srv = rospy.ServiceProxy('relax_arm', Empty)
        rospy.wait_for_service('relax_arm')

        self.reset_arm_movement_history_srv = rospy.ServiceProxy(
                                        'reset_arm_movement_history', Empty)
        rospy.wait_for_service('reset_arm_movement_history')

        self.get_gripper_state_srv = rospy.ServiceProxy(
                                        'get_gripper_state', GetGripperState)
        rospy.wait_for_service('get_gripper_state')

        self.get_joint_states_srv = rospy.ServiceProxy(
                                        'get_joint_states', GetJointStates)
        rospy.wait_for_service('get_joint_states')

        self.get_ee_pose_srv = rospy.ServiceProxy('get_ee_pose', GetEEPose)
        rospy.wait_for_service('get_ee_pose')

        self.set_gripper_state_srv = rospy.ServiceProxy(
                                        'set_gripper_state', SetGripperState)
        rospy.wait_for_service('set_gripper_state')

        rospy.loginfo("Got all arm services")

        # head services

        self.gaze_client = SimpleActionClient('gaze_action', GazeAction)

        self.gaze_client.wait_for_server(rospy.Duration(5))

        self.current_gaze_goal_srv = rospy.ServiceProxy(
                                        'get_current_gaze_goal', GetGazeGoal)
        rospy.wait_for_service('get_current_gaze_goal')

        rospy.loginfo("Got all head services")

        # world services

        self._get_nearest_object_srv = rospy.ServiceProxy(
                                        'get_nearest_object', GetNearestObject)
        rospy.wait_for_service('get_nearest_object')

        rospy.loginfo("Got all world services")


        # sound stuff
        self._sound_client = SoundClient()

    # arm stuff

    def move_arm_to_pose(self, arm_state):
        '''Move robot's arm to pose

        Args:
            arm_state (ArmState) : contains pose info
        Return:
            bool : success
        '''
        try:
            result = self.move_arm_to_pose_srv(arm_state).success
        except Exception, e:
            result = False
        return result

    def start_move_arm_to_pose(self, arm_state):
        '''Start thread to move robot's arm to pose

        Args:
            arm_state (ArmState) : contains pose info
        Return:
            bool : success
        '''
        # TODO(sarah): Is this really necessary? Investigate.
        try:
            result = self.start_move_arm_to_pose_srv(arm_state).success
        except Exception, e:
            result = False
        return result

    def move_arm_to_joints_plan(self, arm_state):
        '''Move robot's arm to joints positions from arm_state
        using Moveit to plan

        Args:
            arm_state (ArmState) : contains joint position info
        Return:
            bool : success
        '''
        try:
            result = self.move_arm_to_joints_plan_srv(arm_state).success
        except Exception, e:
            result = False
        return result
    def move_arm_to_joints(self, arm_states, times):
        '''Move robot's arm to joints positions from arm_state
        without planning, just joint interpolation

        Args:
            arm_states ([ArmState]) : contains joint position info
        Return:
            bool : success
        '''
        try:
            result = self.move_arm_to_joints_srv(arm_states, times).success
        except Exception, e:
            result = False
        return result

    def can_reach(self, arm_state):
        '''Returns whether arm can reach pose

        Args:
            arm_state (ArmState) : contains pose info
        Return:
            bool : success
        '''
        try:
            result = self.is_reachable_srv(arm_state).success
        except Exception, e:
            result = False
        return result

    def reset_arm_movement_history(self):
        '''Resets movement history of arm'''
        try:
            self.reset_arm_movement_history_srv()
        except Exception, e:
            pass
        return

    def get_gripper_state(self):
        '''Returns state of gripper

        Returns:
            GripperState.OPEN|GripperState.CLOSED
        '''
        try:
            result = self.get_gripper_state_srv().gripper_state
        except Exception, e:
            # defaults to closed, but maybe not a good idea
            result = GripperState.CLOSED
        return result

    def set_gripper_state(self, gripper_state):
        '''Sets state of gripper. Assumed to succeed

        Args:
            gripper_state (GripperState.OPEN|GripperState.CLOSED)
        '''
        try:
            self.set_gripper_state_srv(gripper_state)
        except Exception, e:
            pass
        return

    def get_arm_state(self):
        '''Returns current state of arm

        Returns:
            ArmState
        '''
        abs_ee_pose = self.get_ee_pose_srv().ee_pose  # (PoseStamped)
        joint_pose = self.get_joint_states_srv().joints  # ([float64])

        state = None
        rel_ee_pose = None
        try:
            resp = self._get_nearest_object_srv(
                abs_ee_pose)
            has_nearest = resp.has_nearest
        except Exception, e:
            has_nearest = False

        if not has_nearest:
            # Arm state is absolute (relative to robot's base_link).
            state = ArmState(
                ArmState.ROBOT_BASE,  # ref_frame (uint8)
                abs_ee_pose,  # ee_pose (PoseStamped)
                joint_pose,  # joint_pose ([float64])
                [], # velocities
                Landmark()  # ref_frame_landmark (Landmark)
            )
        else:
            nearest_obj = resp.nearest_object
            # Arm state is relative (to some object in the world).
            # rospy.loginfo("Relative to: {}".format(nearest_obj.name))

            rel_ee_pose = self._tf_listener.transformPose(
                            nearest_obj.name, abs_ee_pose)

            state = ArmState(
                ArmState.OBJECT,  # ref_frame (uint8)
                rel_ee_pose,  # ee_pose (PoseStamped)
                joint_pose,  # joint_pose [float64]
                [], # velocities
                nearest_obj  # ref_frameLandmark (Landmark)
            )

        return state

    def relax_arm(self):
        '''Make sure gravity compensation controller is on and other
        controllers are off
        '''
        try:
            self.relax_arm_srv()
        except Exception, e:
            pass
        return

    def is_arm_moving(self):
        '''Check if arm is currently moving

        Returns:
            bool : True if arm is moving, else False
        '''
        try:
            result = self.is_arm_moving_srv().moving
        except Exception, e:
            result = False
        return result

    # Head stuff

    def shake_head(self, num=5):
        '''Shakes robot's head

        Args:
            num (int) : number of times to perform action
        '''
        if self._social_gaze:
            try:
                goal = GazeGoal()
                goal.action = GazeGoal.SHAKE
                goal.repeat = num
                current_goal = self.current_gaze_goal_srv().gaze_goal
                if goal.action != current_goal:
                    self.gaze_client.send_goal(goal)
            except Exception, e:
                pass

    def nod_head(self, num=5):
        '''Nods robot's head

        Args:
            num (int) : number of times to perform action
        '''
        if self._social_gaze:
            try:
                goal = GazeGoal()
                goal.action = GazeGoal.NOD
                goal.repeat = num
                current_goal = self.current_gaze_goal_srv().gaze_goal
                if goal.action != current_goal:
                    self.gaze_client.send_goal(goal)
            except Exception, e:
                pass

    def look_at_point(self, point):
        '''Points robot's head at point

        Args:
            point (Point)
        '''
        if self._social_gaze:
            try:
                goal = GazeGoal()
                goal.action = GazeGoal.LOOK_AT_POINT
                goal.point = point
                current_goal = self.current_gaze_goal_srv().gaze_goal
                if goal.action != current_goal:
                    self.gaze_client.send_goal(goal)
            except Exception, e:
                pass

    def look_at_ee(self, follow=True):
        '''Makes head look at (or follow) end effector position

        Args:
            follow (bool, optional) : If True, follow end effector,
                                      else, just glance at end effector
        '''
        # rospy.loginfo("Look at ee")
        if self._social_gaze:
            try:
                goal = GazeGoal()
                if follow:
                    goal.action = GazeGoal.FOLLOW_EE
                else:
                    goal.action = GazeGoal.GLANCE_EE
                current_goal = self.current_gaze_goal_srv().gaze_goal
                if goal.action != current_goal:
                    self.gaze_client.send_goal(goal)
            except Exception, e:
                pass

    def look_forward(self):
        '''Point head forward'''
        if self._social_gaze:
            try:
                goal = GazeGoal()
                goal.action = GazeGoal.LOOK_FORWARD
                current_goal = self.current_gaze_goal_srv().gaze_goal
                if goal.action != current_goal:
                    self.gaze_client.send_goal(goal)
            except Exception, e:
                pass

    def look_down(self):
        '''Point head down at table'''
        # TODO(sarah): maybe actually scan for table instead of
        # looking at static point
        try:
            goal = GazeGoal()
            goal.action = GazeGoal.LOOK_DOWN
            current_goal = self.current_gaze_goal_srv().gaze_goal
            if goal.action != current_goal:
                self.gaze_client.send_goal(goal)
            while (self.gaze_client.get_state() == GoalStatus.PENDING or
                   self.gaze_client.get_state() == GoalStatus.ACTIVE):
                rospy.sleep(0.2)
        except Exception, e:
            pass

    # Sound stuff

    def play_sound(self, requested_sound):
        '''Play sound that is requested

        Args:
            requested_sound (RobotSound.ERROR|etc...) : see RobotSound.msg
        '''
        try:
            if self._play_sound:
                self._sound_client.playWave(
                    os.path.join(SOUNDS_DIR, requested_sound + SOUND_FILEFORMAT))
        except Exception, e:
            rospy.loginfo(e)
