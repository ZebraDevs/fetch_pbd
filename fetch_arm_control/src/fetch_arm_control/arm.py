''' Interface for controlling arm '''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# System builtins
import threading
from numpy import array, sign, pi, dot
from numpy.linalg import norm

# ROS builtins
import moveit_commander
import actionlib
import tf
from control_msgs.msg import FollowJointTrajectoryGoal, \
                             FollowJointTrajectoryAction, \
                             FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from actionlib import SimpleActionClient
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped
from robot_controllers_msgs.msg import QueryControllerStatesAction, \
                                       QueryControllerStatesGoal, \
                                       ControllerState

# Local
from fetch_arm_control.msg import GripperState

# ######################################################################
# Module level constants
# ######################################################################

# The minimum time to allow for moving between poses.
DURATION_MIN_THRESHOLD = 0.5  # seconds

# ######################################################################
# Classes
# ######################################################################

class Arm:
    ''' Interface for controlling mid-level operations of the arm.
    Interfaces with MoveIt and joint trajectory controllers to move the
    arm to locations using different planning strategies.
    '''
    def __init__(self, tf_listener):
        '''
        Args:
            tf_listener (TransformListener)
        '''
        self._tf_listener = tf_listener

        self._gripper_state = None

        self._l_gripper_joint_name = 'l_gripper_finger_joint'
        self._r_gripper_joint_name = 'r_gripper_finger_joint'

        self._ee_name = 'wrist_roll_link'
        self._joint_names = ['shoulder_pan_joint',
                            'shoulder_lift_joint',
                            'upperarm_roll_joint',
                            'elbow_flex_joint',
                            'forearm_roll_joint',
                            'wrist_flex_joint',
                            'wrist_roll_joint']

        # names/poses for joints we have received information about
        self._received_joint_names = []
        self._received_joint_poses = []

        self._last_ee_pose = None
        self._movement_buffer_size = 40
        self._last_unstable_time = rospy.Time.now()
        self._arm_movement = []
        self._is_executing = False

        self._lock = threading.Lock()
        rospy.Subscriber('joint_states', JointState, self._joint_states_cb)

        controller_states = "/query_controller_states"

        self._controller_client = actionlib.SimpleActionClient(
                                        controller_states,
                                        QueryControllerStatesAction)
        self._controller_client.wait_for_server()

        self._gravity_comp_controllers = ["arm_controller/gravity_compensation"]

        self._non_gravity_comp_controllers = list()
        self._non_gravity_comp_controllers.append(
                    "arm_controller/follow_joint_trajectory")
        self._non_gravity_comp_controllers.append(
                    "arm_with_torso_controller/follow_joint_trajectory")

        traj_controller_name = ("/arm_controller/follow_joint_trajectory")
        self._traj_action_client = SimpleActionClient(
                        traj_controller_name, FollowJointTrajectoryAction)
        # self._traj_action_client.wait_for_server()

        # Initialise all the Moveit stuff

        # Wait for move_group to be ready
        rospy.wait_for_service('compute_ik')
        self._move_group = moveit_commander.MoveGroupCommander("arm")
        self._move_group.set_planning_time(2.0)
        self._move_group.set_planner_id('RRTConnectkConfigDefault')

        # Define ground plane
        # This creates objects in the planning scene that mimic the ground
        # If these were not in place gripper could hit the ground
        self._planning_scene = moveit_commander.PlanningSceneInterface()
        self._planning_scene.remove_world_object("ground_plane")

        ground_pose = PoseStamped()
        ground_pose.header.frame_id = "base_link"
        ground_pose.pose.position.x = 0.0
        ground_pose.pose.position.y = 0.0
        ground_pose.pose.position.z = -0.012
        ground_pose.pose.orientation.w = 1.0

        # For some reason that defies logic, it's best to add and remove
        # planning scene objects in a loop.
        # It sometimes doesn't work the first time.
        for i in range(5):
            self._planning_scene.add_box("ground_plane", ground_pose,
                                        (1.5, 1.5, 0.02))
            rospy.sleep(0.2)

        gripper_controller_name = "gripper_controller/gripper_action"
        self._gripper_client = SimpleActionClient(gripper_controller_name,
                                                    GripperCommandAction)
        self._gripper_client.wait_for_server()
        self._update_gripper_state()

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def update(self):
        ''' Periodical update for one arm'''
        ee_pose = self.get_ee_state()
        if ee_pose != None and self._last_ee_pose != None:
            self._record_arm_movement(Arm._get_distance_bw_poses(ee_pose,
                                                        self._last_ee_pose))
        self._last_ee_pose = ee_pose

    def relax_arm(self):
        '''Turns on gravity compensation controller and turns
        off other controllers
        '''

        goal = QueryControllerStatesGoal()

        for controller in self._gravity_comp_controllers:
            state = ControllerState()
            state.name = controller
            state.state = state.RUNNING
            goal.updates.append(state)

        for controller in self._non_gravity_comp_controllers:
            state = ControllerState()
            state.name = controller
            state.state = state.STOPPED
            goal.updates.append(state)

        self._controller_client.send_goal(goal)

    def un_relax_arm(self):
        '''Turns on gravity compensation controller and turns
        off other controllers
        '''

        goal = QueryControllerStatesGoal()

        for controller in self._non_gravity_comp_controllers:
            state = ControllerState()
            state.name = controller
            state.state = state.RUNNING
            goal.updates.append(state)

        # for controller in self._gravity_comp_controllers:
        #     state = ControllerState()
        #     state.name = controller
        #     state.state = state.STOPPED
        #     goal.updates.append(state)

        self._controller_client.send_goal(goal)

    def get_ee_state(self, ref_frame='base_link'):
        ''' Returns end effector pose for the arm

        Args:
            ref_frame (st, optional): Reference frame that you
                                      want the pose return in

        Returns:
            PoseStamped | None
        '''
        try:
            time = self._tf_listener.getLatestCommonTime(ref_frame,
                                                         self._ee_name)
            (position, orientation) = self._tf_listener.lookupTransform(
                                                ref_frame, self._ee_name, time)
            tf_pose = PoseStamped()
            tf_pose.header.frame_id = ref_frame
            tf_pose.pose.position = Point(position[0], position[1],
                                          position[2])
            tf_pose.pose.orientation = Quaternion(orientation[0],
                                            orientation[1],
                                            orientation[2], orientation[3])
            return tf_pose
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logwarn('Something wrong with transform request: ' + str(e))
            return None

    def get_joint_state(self, joint_names=None):
        '''Returns position(s) for the requested or all arm joints

        Args:
            joint_names ([str], optional):

        Returns:
            [float]
        '''
        if joint_names is None:
            joint_names = self._joint_names

        if self._received_joint_names == []:
            rospy.logerr("No robot_state messages received!\n")
            return []

        positions = []
        self._lock.acquire()
        for joint_name in joint_names:
            if joint_name in self._received_joint_names:
                index = self._received_joint_names.index(joint_name)
                position = self._received_joint_poses[index]
                positions.append(position)
            else:
                rospy.logerr("Joint %s not found!", joint_name)
        self._lock.release()
        return positions

    def get_ik_for_ee(self, ee_pose, seed):
        ''' Finds the IK solution for given end effector pose

        Args:
            ee_pose (PoseStamped)

        Returns:
            [float]
        '''
        # rospy.loginfo("Solve ik: {}".format(ee_pose))
        joints = self._solve_ik(ee_pose)

        if joints is None:
            pass
        else:
            rollover = array((array(joints) - array(seed)) / pi, int)
            joints -= ((rollover + (sign(rollover) + 1) / 2) / 2) * 2 * pi

        return joints

    def move_to_pose(self, ee_pose):
        ''' Move arm to specified pose

        Args:
            ee_pose (Pose)

        Returns:
            bool
        '''
        ee_pose_stamped = None

        if type(ee_pose) is Pose:
            ee_pose_stamped = PoseStamped()
            ee_pose_stamped.pose = ee_pose
            ee_pose_stamped.header.frame_id = "base_link"

        elif type(ee_pose) is PoseStamped:
            ee_pose_stamped = ee_pose
        else:
            rospy.loginfo("Not a pose or pose stamped")
            return False

        self._move_group.set_pose_target(ee_pose_stamped)

        self._move_group.set_planning_time(1.0)

        plan = self._move_group.plan()

        if not plan.joint_trajectory.points:
            return False
        else:

            self._is_executing = True
            go = self._move_group.go(wait=True)
            self._is_executing = False
            rospy.loginfo("Go: {}".format(go))
            return True

    def get_gripper_state(self):
        '''Returns current gripper state

        Returns:
            GripperState.OPEN/CLOSED
        '''
        return self._gripper_state

    def open_gripper(self, pos=0.15, eff=100.0, wait=True):
        '''Opens gripper

        Args:
            pos (float)
            eff (float)
            wait (bool)
        '''
        self._send_gripper_command(pos, eff, wait)
        self._gripper_state = GripperState.OPEN

    def close_gripper(self, pos=0.0, eff=100.0, wait=True):
        '''Closes gripper

        Args:
            pos (float)
            eff (float)
            wait (bool)
        '''
        self._send_gripper_command(pos, eff, wait)
        self._gripper_state = GripperState.CLOSED

    def move_to_joints_plan(self, joints, velocities=None):
        '''Moves the arm to the desired joint angles using moveit

        Args:
            joints ([float])
        '''

        joint_state = JointState()
        joint_state.position = joints
        joint_state.name = self._joint_names
        # if velocities is None:
        #     velocities = [0] * len(joints)
        # joint_state.velocity = velocities

        try:
            self._move_group.set_joint_value_target(joint_state)
        except Exception as e:
            rospy.logerr("Moveit Error: {}".format(e))
            return False

        self._move_group.set_planning_time(1.0)

        plan = self._move_group.plan()

        if not plan.joint_trajectory.points:
            return False
        else:

            self._is_executing = True
            go = self._move_group.go(wait=True)
            self._is_executing = False
            rospy.loginfo("Go: {}".format(go))
            return True

    def move_to_joints(self, joints, times_to_joints, velocities=None):
        '''Moves the arm to the desired joint angles directly without planning

        Args:
            joints ([float])
            time_to_joint ([float])
            velocities ([float])
        '''
        self.un_relax_arm()

        trajectory = JointTrajectory()
        trajectory.header.stamp = (rospy.Time.now() +
                                             rospy.Duration(0.1))
        trajectory.joint_names = self._joint_names

        for i in range(len(joints)):
            trajectory.points.append(JointTrajectoryPoint(
                            positions=joints[i],
                            time_from_start=rospy.Duration(times_to_joints[i])))

        self._is_executing = True

        follow_goal = FollowJointTrajectoryGoal(trajectory=trajectory)
        self._traj_action_client.send_goal(follow_goal)
        self._traj_action_client.wait_for_result()
        self._is_executing = False
        self.relax_arm()

        result = self._traj_action_client.get_result()
        if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
            rospy.loginfo("arm joints move successful")
            return True
        else:
            rospy.loginfo("arm joints move not successful")
            return False

    def get_time_to_pose(self, target_pose):
        '''Returns the time to get to the arm pose held in target_pose.

        Args:
            target_pose (PoseStamped|None): A Pose holding the pose to
                move to, or None if the arm should not move.

        Returns:
            float|None: How long (in seconds) to allow for moving
                arm to the pose in target_pose, or None if the arm
                will not move.
        '''
        # Check whether arm will move at all.
        if target_pose is None:
            rospy.loginfo('\t' + 'arm will not move.')
            return None
        else:
            time_to_pose = Arm._get_time_bw_poses(
                self.get_ee_state(),
                target_pose
            )
            rospy.loginfo(
                '\tDuration until next frame for arm' +
                'arm : ' + str(time_to_pose))
            return time_to_pose

    def is_executing(self):
        '''Whether or not there is an ongoing action execution on the arm

        Returns:
            bool
        '''
        return self._is_executing

    def reset_movement_history(self):
        ''' Clears the saved history of arm movements'''
        self._last_unstable_time = rospy.Time.now()
        self._arm_movement = []

    def get_movement(self):
        '''Returns cumulative movement in recent history

        Returns:
            [float]
        '''
        return sum(self._arm_movement)

    # ##################################################################
    # Static methods: Internal ("private")
    # ##################################################################

    @staticmethod
    def _get_time_bw_poses(pose0, pose1, velocity=0.2):
        '''Determines how much time should be allowed for moving between
        pose0 and pose1 at velocity.

        Args:
            pose0 (PoseStamped)
            pose1 (PoseStamped)
            velocity (float, optional): Defaults to 0.2.

        Returns:
            float: How long (in seconds) to allow for moving between
                pose0 and pose1 and velocity.
        '''
        dist = Arm._get_distance_bw_poses(pose0, pose1)
        duration = dist / velocity
        return (
            DURATION_MIN_THRESHOLD if duration < DURATION_MIN_THRESHOLD
            else duration)

    @staticmethod
    def _get_distance_bw_poses(pose0, pose1):
        '''Returns the dissimilarity between two end-effector poses

        Args:
            pose0 (PoseStamped)
            pose1 (PoseStamped)

        Returns:
            float
        '''
        w_pos = 1.0
        w_rot = 0.2
        pos0 = array((pose0.pose.position.x, pose0.pose.position.y,
                     pose0.pose.position.z))
        pos1 = array((pose1.pose.position.x, pose1.pose.position.y,
                     pose1.pose.position.z))
        rot0 = array((pose0.pose.orientation.x, pose0.pose.orientation.y,
                      pose0.pose.orientation.z, pose0.pose.orientation.w))
        rot1 = array((pose1.pose.orientation.x, pose1.pose.orientation.y,
                      pose1.pose.orientation.z, pose1.pose.orientation.w))
        pos_dist = w_pos * norm(pos0 - pos1)
        rot_dist = w_rot * (1 - dot(rot0, rot1))

        if pos_dist > rot_dist:
            dist = pos_dist
        else:
            dist = rot_dist
        return dist

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _joint_states_cb(self, msg):
        '''Callback function that saves the joint positions when a
        joint_states message is received

        Args:
            msg (JointState): current joint positions for arm
        '''
        self._lock.acquire()

        for i, joint_name in enumerate(msg.name):
            if joint_name in self._received_joint_names:
                idx = self._received_joint_names.index(joint_name)
                self._received_joint_poses[idx] = msg.position[i]
            else:
                self._received_joint_names.append(joint_name)
                self._received_joint_poses.append(msg.position[i])

        self._lock.release()

    def _solve_ik(self, ee_pose):
        '''Gets the IK solution for end effector pose

        Args:
            ee_pose (PoseStamped)

        Returns:
            Pose
        '''

        self._move_group.set_pose_target(ee_pose)

        self._move_group.set_planning_time(1.0)

        plan = self._move_group.plan()

        if not plan.joint_trajectory.points:
            return None
        else:
            joint_names = plan.joint_trajectory.joint_names
            # rospy.loginfo("Plan: {}".format(plan))
            positions = plan.joint_trajectory.points[-1].positions
            return [positions[i] for i, x in enumerate(joint_names) \
                                            if x in self._joint_names]

    def _send_gripper_command(self, pos=0.115, eff=30.0, wait=True):
        '''Sets the position of the gripper

        Args:
            pose (float): Usually get this from echoing /joint_states
            eff (float)
            wait (bool)
        '''
        command = GripperCommandGoal()
        command.command.position = pos
        command.command.max_effort = eff
        self._gripper_client.send_goal(command)
        if wait:
            self._gripper_client.wait_for_result(rospy.Duration(5.0))

    def _update_gripper_state(self):
        '''Updates gripper state based on joint positions'''

        gripper_pos = self.get_joint_state([self._l_gripper_joint_name,
                                           self._r_gripper_joint_name])

        # Check if both fingers are further than 0.04m away from center.
        # This could be changed to a different threshold/check
        if gripper_pos != []:
            if gripper_pos[0] > 0.04 and gripper_pos[1] > 0.04:
                self._gripper_state = GripperState.OPEN
            else:
                self._gripper_state = GripperState.CLOSED
        else:
            rospy.logwarn('Could not update the gripper state.')

    def _record_arm_movement(self, movement):
        '''Records the sensed arm movement

        Args:
            movement (float): dissimilarity between current pose and
                                previous pose
        '''
        self._arm_movement = [movement] + self._arm_movement
        if len(self._arm_movement) > self._movement_buffer_size:
            self._arm_movement = \
                self._arm_movement[0:self._movement_buffer_size]


