#!/usr/bin/env python

'''Handles where to point robot's head'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# System builtins
import time
from numpy import array, linalg

# ROS builtins
# from scipy.ndimage.filters import *
from actionlib_msgs.msg import GoalStatus
from actionlib import SimpleActionClient, SimpleActionServer
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import Point
from tf import TransformListener

# Local
from fetch_pbd_interaction.srv import GetGazeGoal, GetGazeGoalResponse
from fetch_social_gaze.msg import GazeGoal, GazeAction

# ######################################################################
# Classes
# ######################################################################

class SocialGaze:
    '''Handles where to point robot's head'''

    def __init__(self):
        self._default_focus_point = Point(1, 0, 1.05)
        self._down_focus_point = Point(0.5, 0, 0.5)
        self._target_focus_point = Point(1, 0, 1.05)
        self._current_focus_point = Point(1, 0, 1.05)

        self._current_gaze_action = GazeGoal.LOOK_FORWARD
        self._prev_gaze_action = self._current_gaze_action
        self._prev_target_focus_point = array(self._default_focus_point)

        # Some constants
        self._no_interrupt = [GazeGoal.NOD,
                               GazeGoal.SHAKE, GazeGoal.LOOK_DOWN]
        self._nod_positions = [Point(1, 0, 0.70), Point(1, 0, 1.20)]
        self._shake_positions = [Point(1, 0.2, 1.05), Point(1, -0.2, 1.05)]
        self._n_nods = 5
        self._n_shakes = 5

        self._nod_counter = 5
        self._shake_counter = 5
        self._face_pos = None
        self._glance_counter = 0

        self.gaze_goal_strs = {
            GazeGoal.LOOK_FORWARD: 'LOOK_FORWARD',
            GazeGoal.FOLLOW_EE: 'FOLLOW_EE',
            GazeGoal.GLANCE_EE: 'GLANCE_EE',
            GazeGoal.NOD: 'NOD',
            GazeGoal.SHAKE: 'SHAKE',
            GazeGoal.FOLLOW_FACE: 'FOLLOW_FACE',
            GazeGoal.LOOK_AT_POINT: 'LOOK_AT_POINT',
            GazeGoal.LOOK_DOWN: 'LOOK_DOWN',
            GazeGoal.NONE: 'NONE',
        }

        ## Action client for sending commands to the head.
        self._head_action_client = SimpleActionClient(
            '/head_controller/point_head', PointHeadAction)
        self._head_action_client.wait_for_server()
        rospy.loginfo('Head action client has responded.')

        self._head_goal = PointHeadGoal()
        self._head_goal.target.header.frame_id = 'base_link'
        self._head_goal.min_duration = rospy.Duration(1.0)
        self._head_goal.target.point = Point(1, 0, 1)

        ## Service client for arm states
        self._tf_listener = TransformListener()

        ## Server for gaze requested gaze actions
        self._gaze_action_server = SimpleActionServer(
            '/fetch_pbd/gaze_action', GazeAction, self._execute_gaze_action, False)
        self._gaze_action_server.start()

        self._is_action_complete = True

        rospy.Service('/fetch_pbd/get_current_gaze_goal', GetGazeGoal,
                      self._get_gaze_goal)

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def update(self):
        '''Update goal for head movement'''
        is_action_possibly_complete = True

        if self._current_gaze_action == GazeGoal.FOLLOW_EE:
            self._target_focus_point = self._get_ee_pos()

        elif self._current_gaze_action == GazeGoal.NOD:
            self._target_focus_point = self._get_next_nod_point(
                self._current_focus_point, self._target_focus_point)
            self._head_goal.min_duration = rospy.Duration(0.5)
            is_action_possibly_complete = False

        elif self._current_gaze_action == GazeGoal.SHAKE:
            self._target_focus_point = self._get_next_shake_point(
                self._current_focus_point, self._target_focus_point)
            self._head_goal.min_duration = rospy.Duration(0.5)
            is_action_possibly_complete = False

        elif self._current_gaze_action == GazeGoal.GLANCE_EE:
            self._target_focus_point = self._get_next_glance_point(
                self._current_focus_point, self._target_focus_point)
            is_action_possibly_complete = False

        self._current_focus_point = SocialGaze._filter_look_at_position(
            self._current_focus_point, self._target_focus_point)
        if self._current_gaze_action is GazeGoal.NONE:
            pass
        elif (SocialGaze._is_the_same(
                        SocialGaze._point2array(self._head_goal.target.point),
                        SocialGaze._point2array(self._target_focus_point))):

            if is_action_possibly_complete:
                head_state = self._head_action_client.get_state()
                if head_state == GoalStatus.SUCCEEDED:
                    self._is_action_complete = True
        else:
            self._head_goal.target.point.x = self._current_focus_point.x
            self._head_goal.target.point.y = self._current_focus_point.y
            self._head_goal.target.point.z = self._current_focus_point.z
            self._head_action_client.send_goal(self._head_goal)

        time.sleep(0.02)

    # ##################################################################
    # Static methods: Internal ("private")
    # ##################################################################

    @staticmethod
    def _is_the_same(current, target):
        '''Get 3DOF pose of end effector

        Args:
            current (array): (x, y, z) of point
            target (array): (x, y, z) of point

        Returns:
            bool
        '''
        diff = target - current
        dist = linalg.norm(diff)
        return dist < 0.003

    @staticmethod
    def _point2array(p):
        '''Make Point msg into array

        Args:
            p (Point)

        Returns:
            array
        '''
        return array((p.x, p.y, p.z))

    @staticmethod
    def _array2point(a):
        '''Make array into Point msg

        Args:
            a (array)

        Returns:
            Point
        '''
        return Point(a[0], a[1], a[2])

    @staticmethod
    def _filter_look_at_position(current, target):
        '''If head goal is too far away, returns an intermediate position
           to limit speed

        Args:
            current (Point): current head goal
            target (Point): new head goal

        Returns:
            Point
        '''
        speed = 0.02
        diff = (SocialGaze._point2array(target) -
                    SocialGaze._point2array(current))
        dist = linalg.norm(diff)
        if dist > speed:
            step = dist / speed
            return SocialGaze._array2point(SocialGaze._point2array(current) +
                                           diff / step)
        else:
            return target

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _get_gaze_goal(self, req):
        '''Return current gaze goal

        Args:
            req (GetGazeGoalRequest) : Unused
        Returns:
            GetGazeGoalResponse
        '''
        goal = self._current_gaze_action
        # rospy.loginfo("Gaze Goal: {}".format(goal))
        return GetGazeGoalResponse(int(goal))


    def _get_ee_pos(self):
        '''Get 3DOF pose of end effector

        Returns:
            Point: location of wrist of end effector
                    (could change to be finger tips)
        '''

        from_frame = '/base_link'
        to_frame = '/wrist_roll_link'

        try:
            t = self._tf_listener.getLatestCommonTime(from_frame, to_frame)
            (position, rotation) = self._tf_listener.lookupTransform(
                                                                   from_frame,
                                                                   to_frame, t)
        except:
            rospy.logerr('Could not get the end-effector pose.')

        return Point(position[0], position[1], position[2])

    ## Callback function for receiving gaze commands
    def _execute_gaze_action(self, goal):
        '''Get 3DOF pose of end effector

        Args:
            goal (GazeGoal): what type of action to perform next
        '''
        # rospy.loginfo("Got a head goal: {}".format(goal.action))
        command = goal.action
        if self._no_interrupt.count(self._current_gaze_action) == 0:
            if (self._current_gaze_action != command or
                command == GazeGoal.LOOK_AT_POINT):
                self._is_action_complete = False
                if command == GazeGoal.LOOK_FORWARD:
                    self._target_focus_point = self._default_focus_point
                elif command == GazeGoal.LOOK_DOWN:
                    self._target_focus_point = self._down_focus_point
                elif command == GazeGoal.NOD:
                    self._n_nods = goal.repeat
                    self._start_nod()
                elif command == GazeGoal.SHAKE:
                    self._n_shakes = goal.repeat
                    self._start_shake()
                elif command == GazeGoal.GLANCE_EE:
                    self._start_glance()
                elif command == GazeGoal.LOOK_AT_POINT:
                    self._target_focus_point = goal.point
                # rospy.loginfo('\tSetting gaze action to: ' +
                #               self.gaze_goal_strs[command])
                self._current_gaze_action = command

                while not self._is_action_complete:
                    time.sleep(0.1)
                self._current_gaze_action = GazeGoal.NONE
                # Perturb the head goal so it gets updated in the update loop.
                self._head_goal.target.point.x += 1
                self._gaze_action_server.set_succeeded()
            else:
                self._gaze_action_server.set_aborted()

        else:
            self._gaze_action_server.set_aborted()

    def _start_nod(self):
        '''Start nod action'''
        self._prev_target_focus_point = self._target_focus_point
        self._prev_gaze_action = str(self._current_gaze_action)
        self._nod_counter = 0
        self._target_focus_point = self._nod_positions[0]

    def _start_glance(self):
        '''Start glance action'''
        self._prev_target_focus_point = self._target_focus_point
        self._prev_gaze_action = str(self._current_gaze_action)
        self._glance_counter = 0
        self._target_focus_point = self._get_ee_pos()

    def _start_shake(self):
        '''Start shake action'''
        self._prev_target_focus_point = self._target_focus_point
        self._prev_gaze_action = str(self._current_gaze_action)
        self._shake_counter = 0
        self._target_focus_point = self._shake_positions[0]

    def _get_next_nod_point(self, current, target):
        '''Get next point to look at while nodding

        Args:
            current (Point): current head goal
            target (Point): new head goal

        Returns:
            Point
        '''
        if (SocialGaze._is_the_same(SocialGaze._point2array(current),
                           SocialGaze._point2array(target))):
            self._nod_counter += 1
            if self._nod_counter == self._n_nods:
                self._current_gaze_action = self._prev_gaze_action
                return self._prev_target_focus_point
            else:
                return self._nod_positions[self._nod_counter % 2]
        else:
            return target

    def _get_next_glance_point(self, current, target):
        '''Get next point to look at while glancing

        Args:
            current (Point): current head goal
            target (Point): new head goal

        Returns:
            Point
        '''
        if (SocialGaze._is_the_same(SocialGaze._point2array(current),
                           SocialGaze._point2array(target))):
            self._glance_counter = 1
            self._current_gaze_action = self._prev_gaze_action
            return self._prev_target_focus_point
        else:
            return target

    def _get_next_shake_point(self, current, target):
        '''Get next point to look at while shaking

        Args:
            current (Point): current head goal
            target (Point): new head goal

        Returns:
            Point
        '''
        if (SocialGaze._is_the_same(SocialGaze._point2array(current),
                           SocialGaze._point2array(target))):
            self._shake_counter += 1
            if self._shake_counter == self._n_shakes:
                self._current_gaze_action = self._prev_gaze_action
                return self._prev_target_focus_point
            else:
                return self._shake_positions[self._shake_counter % 2]
        else:
            return target

if __name__ == '__main__':
    rospy.init_node('social_gaze')
    gaze = SocialGaze()
    while not rospy.is_shutdown():
        gaze.update()
