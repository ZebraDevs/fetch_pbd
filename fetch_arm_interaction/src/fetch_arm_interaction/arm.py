''' Interface for controlling one arm '''
import moveit_commander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
import threading
import rospy
import actionlib
import tf
from numpy import array, sign, pi, dot
from numpy.linalg import norm
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from actionlib_msgs.msg import GoalStatus
from actionlib import SimpleActionClient
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped
from fetch_arm_interaction.msg import GripperState
from robot_controllers_msgs.msg import QueryControllerStatesAction, \
                                       QueryControllerStatesGoal, \
                                       ControllerState
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import moveit_commander

# The minimum time to allow for moving between poses.
DURATION_MIN_THRESHOLD = 0.5  # seconds

class Arm:
    ''' Interfacing with one arm for controlling mode and action execution'''

    _is_autorelease_on = False

    def __init__(self, tf_listener):
        # self.arm_index = arm_index
        self.tf_listener = tf_listener

        self.gripper_state = None

        self.l_gripper_joint_name = 'l_gripper_finger_joint'
        self.r_gripper_joint_name = 'r_gripper_finger_joint'

        self.ee_name = 'wrist_roll_link'
        self.joint_names = ['shoulder_pan_joint', 
                            'shoulder_lift_joint', 
                            'upperarm_roll_joint', 
                            'elbow_flex_joint', 
                            'forearm_roll_joint', 
                            'wrist_flex_joint', 
                            'wrist_roll_joint']

        self.all_joint_names = [] # what are these for?
        self.all_joint_poses = [] # what are these for?

        self.last_ee_pose = None
        self.movement_buffer_size = 40
        self.last_unstable_time = rospy.Time.now()
        self.arm_movement = []
        self.is_executing = False

        self.lock = threading.Lock()
        rospy.Subscriber('joint_states', JointState, self.joint_states_cb)

        # Change to function to turn off all of the right fetch controllers
        # switch_controller = 'pr2_controller_manager/switch_controller'
        # rospy.wait_for_service(switch_controller)
        # self.switch_service = rospy.ServiceProxy(switch_controller,
        #                                          SwitchController)
        # rospy.loginfo('Got response form the switch controller for '
        #               + self.side() + ' arm.')

        controller_states = "/query_controller_states"
        
        self.controller_client = actionlib.SimpleActionClient(controller_states, QueryControllerStatesAction)
        self.controller_client.wait_for_server()

        self.gravity_comp_controllers = ["arm_controller/gravity_compensation"]

        self.non_gravity_comp_controllers = list()
        self.non_gravity_comp_controllers.append("arm_controller/follow_joint_trajectory")
        self.non_gravity_comp_controllers.append("arm_with_torso_controller/follow_joint_trajectory")
        # self.non_gravity_comp_controllers.append("torso_controller/follow_joint_trajectory")
        # self.non_gravity_comp_controllers.append("head_controller/follow_joint_trajectory")
        # self.non_gravity_comp_controllers.append("head_controller/point_head")

        # Need to replace this with  MoveIt stuff similar to disco demo
        # i.e. move_group.MoveToJointPosition(...)
        #
        # # Create a trajectory action client
        # traj_controller_name = (self._side_prefix()
        #             + '_arm_controller/joint_trajectory_action')
        # self.traj_action_client = SimpleActionClient(
        #                 traj_controller_name, JointTrajectoryAction)
        # self.traj_action_client.wait_for_server()
        # rospy.loginfo('Got response form trajectory action server for '
        #               + self.side() + ' arm.')

        # Option #1, move joints with arm_controller/follow_joint_trajectory

        # Create a trajectory action client
        # traj_controller_name = 'arm_controller/follow_joint_trajectory'
        # self.traj_action_client = SimpleActionClient(
        #                 traj_controller_name, FollowJointTrajectoryAction)
        # self.traj_action_client.wait_for_server()
        # rospy.loginfo('Got response form trajectory action server for '
        #               + self.side() + ' arm.')

        # Option #2, move joints with MoveIt

        # self.move_group = MoveGroupInterface("arm", "base_link")

        # self.move_group.setPlannerId('RRTConnectkConfigDefault')
        # self.move_group.setPlanningTime(2.0)

        # self.move_group_plan = MoveGroupInterface("arm", "base_link", plan_only=True)
        # self.move_group.setPlannerId('RRTConnectkConfigDefault')
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.move_group.set_planning_time(2.0)
        self.move_group.set_planner_id('RRTConnectkConfigDefault')

        # Define ground plane
        # This creates objects in the planning scene that mimic the ground
        # If these were not in place gripper could hit the ground
        self.planning_scene = PlanningSceneInterface("base_link")
        self.planning_scene.removeCollisionObject("my_front_ground")
        self.planning_scene.removeCollisionObject("my_back_ground")
        self.planning_scene.removeCollisionObject("my_right_ground")
        self.planning_scene.removeCollisionObject("my_left_ground")
        self.planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
        self.planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
        self.planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
        self.planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

        # Set up Inverse Kinematics

        self.ik_srv = None
        self.ik_request = None
        self.ik_joints = None
        self.ik_limits = None
        self._setup_ik()

        gripper_controller_name = "gripper_controller/gripper_action"
        self.gripper_client = SimpleActionClient(gripper_controller_name,
                                                    GripperCommandAction)
        self.gripper_client.wait_for_server()
        # rospy.loginfo('Got response form gripper server for '
        #               + self.side() + ' arm.')
        self.check_gripper_state()

    def relax_arm(self):
        '''Turns on gravity compensation controller and turns off other controllers'''

        goal = QueryControllerStatesGoal()
    
        for controller in self.gravity_comp_controllers:
            state = ControllerState()
            state.name = controller
            state.state = state.RUNNING
            goal.updates.append(state)

        for controller in self.non_gravity_comp_controllers:
            state = ControllerState()
            state.name = controller
            state.state = state.STOPPED
            goal.updates.append(state)

        self.controller_client.send_goal(goal)

    def _setup_ik(self):
        '''Sets up services for inverse kinematics'''
        ik_srv_name = '/compute_ik'
        rospy.wait_for_service(ik_srv_name)
        self.ik_srv = rospy.ServiceProxy(ik_srv_name,
                                         GetPositionIK, persistent=True)
        rospy.loginfo('IK service has responded for arm.')

        robot = moveit_commander.RobotCommander()

        # Set up common parts of an IK request
        self.ik_request = GetPositionIKRequest()
        request = self.ik_request.ik_request
        request.timeout = rospy.Duration(2.0)
        group_name = 'arm'
        request.group_name = group_name
        self.ik_joints = self.joint_names
        self.ik_limits = [robot.get_joint(x).bounds() for x in self.ik_joints]
        request.ik_link_name = self.ee_name
        request.pose_stamped.header.frame_id = 'base_link'
        request.robot_state.joint_state.name = self.ik_joints
        request.robot_state.joint_state.position = [0] * len(self.joint_names)

    def get_ee_state(self, ref_frame='base_link'):
        ''' Returns end effector pose for the arm'''
        try:
            time = self.tf_listener.getLatestCommonTime(ref_frame,
                                                         self.ee_name)
            (position, orientation) = self.tf_listener.lookupTransform(
                                                ref_frame, self.ee_name, time)
            tf_pose = Pose()
            tf_pose.position = Point(position[0], position[1], position[2])
            tf_pose.orientation = Quaternion(orientation[0], orientation[1],
                                             orientation[2], orientation[3])
            return tf_pose
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logwarn('Something wrong with transform request: ' + str(e))
            return None

    def joint_states_cb(self, msg):
        '''Callback function that saves the joint positions when a
        joint_states message is received'''
        self.lock.acquire()

        for i, joint_name in enumerate(msg.name):
            if joint_name in self.all_joint_names:
                idx = self.all_joint_names.index(joint_name)
                self.all_joint_poses[idx] = msg.position[i]
            else:
                self.all_joint_names.append(joint_name)
                self.all_joint_poses.append(msg.position[i])

        # self.all_joint_names = msg.name
        # self.all_joint_poses = msg.position
        self.lock.release()

    def get_joint_state(self, joint_names=None):
        '''Returns position for the requested or all arm joints'''
        if joint_names is None:
            joint_names = self.joint_names

        if self.all_joint_names == []:
            rospy.logerr("No robot_state messages received!\n")
            return []

        positions = []
        self.lock.acquire()
        for joint_name in joint_names:
            if joint_name in self.all_joint_names:
                index = self.all_joint_names.index(joint_name)
                position = self.all_joint_poses[index]
                positions.append(position)
            else:
                rospy.logerr("Joint %s not found!", joint_name)
        self.lock.release()
        return positions

    def _solve_ik(self, ee_pose, seed=None):
        '''Gets the IK solution for end effector pose'''

        self.ik_request.ik_request.pose_stamped.pose = ee_pose

        self.move_group.set_pose_target(self.ik_request.ik_request.pose_stamped)

        self.move_group.set_planning_time(1.0)

        plan = self.move_group.plan()

        if not plan.joint_trajectory.points:
            return None
        else:
            joint_names = plan.joint_trajectory.joint_names
            # rospy.loginfo("Plan: {}".format(plan))
            positions = plan.joint_trajectory.points[-1].positions
            return [positions[i] for i, x in enumerate(joint_names) if x in self.joint_names]

    def move_to_pose(self, ee_pose):
        ee_pose_stamped = None

        if (type(ee_pose) is Pose):
            ee_pose_stamped = PoseStamped()
            ee_pose_stamped.pose = ee_pose
            ee_pose_stamped.header.frame_id = "base_link"

        elif (type(ee_pose) is PoseStamped):
            ee_pose_stamped = ee_pose
        else:
            rospy.loginfo("Not a pose or pose stamped")
            return False

        self.move_group.set_pose_target(ee_pose_stamped)

        self.move_group.set_planning_time(1.0)

        plan = self.move_group.plan()

        if not plan.joint_trajectory.points:
            return False
        else:

            self.is_executing = True
            go = self.move_group.go(wait=True)
            self.is_executing = False
            rospy.loginfo("Go: {}".format(go))
            return True



        # if seed is None:
        #     # If no see is specified for IK search, start search at midpoint
        #     seed = []
        #     for i in range(0, len(self.ik_joints)):
        #         seed.append((self.ik_limits[i][0] +
        #                      self.ik_limits[i][1]) / 2.0)
        # self.ik_request.ik_request.robot_state.joint_state.position = seed
        # rospy.loginfo(self.ik_request.ik_request.timeout.to_sec())

        # try:
        #     #rospy.loginfo('Sending IK request.')
        #     rospy.loginfo("Calling IK")
        #     t1 = rospy.Time.now()
        #     response = self.ik_srv(self.ik_request)
        #     t2 = rospy.Time.now()
        #     rospy.loginfo("Done IK. Took: {}".format((t2 - t1).to_sec()))

        #     if(response.error_code.val == response.error_code.SUCCESS):
        #         # The solution contains all robot joints, we only need the joints of one arm.
        #         response_names = response.solution.joint_state.name
        #         response_positions = response.solution.joint_state.position
        #         return [response_positions[i] for i, x in enumerate(response_names) if x in self.joint_names]
        #     else:
        #         return None
        # except rospy.ServiceException:
        #     rospy.logerr('Exception while getting the IK solution.')
        #     return None

    # def set_mode(self, mode):
    #     '''Releases or holds the arm by turning the controller on/off'''
    #     controller_name = self._side_prefix() + '_arm_controller'
    #     if mode == ArmMode.RELEASE:
    #         start_controllers = []
    #         stop_controllers = [controller_name]
    #         rospy.loginfo('Switching ' + str(self.side()) +
    #                       ' arm to the kinesthetic mode')
    #     elif mode == ArmMode.HOLD:
    #         start_controllers = [controller_name]
    #         stop_controllers = []
    #         rospy.loginfo('Switching ' + str(self.side()) +
    #                       ' to the Joint-control mode')
    #     else:
    #         rospy.logwarn('Unknown mode ' + str(mode) +
    #                       '. Keeping the current mode.')
    #         return

    #     try:
    #         self.switch_service(start_controllers, stop_controllers, 1)
    #         self.arm_mode = mode
    #     except rospy.ServiceException:
    #         rospy.logerr("Service did not process request")

    # def get_mode(self):
    #     '''Returns the current arm mode (released/holding)'''
    #     return self.arm_mode

    def _send_gripper_command(self, pos=0.049, eff=30.0, wait=True):
        '''Sets the position of the gripper'''
        command = GripperCommandGoal()
        command.command.position = pos
        command.command.max_effort = eff
        self.gripper_client.send_goal(command)
        if wait:
            self.gripper_client.wait_for_result(rospy.Duration(5.0))

    def is_gripper_moving(self):
        ''' Whether or not the gripper is in the process of opening/closing'''
        return (self.gripper_client.get_state() == GoalStatus.ACTIVE or
                self.gripper_client.get_state() == GoalStatus.PENDING)

    def is_gripper_at_goal(self):
        ''' Whether or not the gripper has reached its goal'''
        return (self.gripper_client.get_state() == GoalStatus.SUCCEEDED)

    def get_gripper_state(self):
        '''Returns current gripper state'''
        return self.gripper_state

    def get_gripper_position(self):
        gripper_pos = self.get_joint_state([self.l_gripper_joint_name, self.r_gripper_joint_name])
        if gripper_pos != []:
            return gripper_pos

    def check_gripper_state(self):
        '''Checks gripper state at the hardware level'''

        gripper_pos = self.get_joint_state([self.l_gripper_joint_name, self.r_gripper_joint_name])

        # Check if both fingers are further than 0.04m away from center.
        # This could be changed to a different threshold/check
        if gripper_pos != []:
            if gripper_pos[0] > 0.04 and gripper_pos[1] > 0.04 :
                self.gripper_state = GripperState.OPEN
            else:
                self.gripper_state = GripperState.CLOSED
        else:
            rospy.logwarn('Could not update the gripper state.')

    def open_gripper(self, pos=0.115, eff=100.0, wait=True):
        '''Opens gripper'''
        self._send_gripper_command(pos, eff, wait)
        self.gripper_state = GripperState.OPEN

    def close_gripper(self, pos=0.0, eff=100.0, wait=True):
        '''Closes gripper'''
        self._send_gripper_command(pos, eff, wait)
        self.gripper_state = GripperState.CLOSED

    def set_gripper(self, gripper_state):
        '''Sets gripper to the desired state'''
        if (gripper_state == GripperState.CLOSED):
            self.close_gripper()
        elif (gripper_state == GripperState.OPEN):
            self.open_gripper()

    def move_to_joints(self, joints, time_to_joint):
        '''Moves the arm to the desired joint angles'''

        ### Option 1, use arm_controller/follow_joint_trajectory

        # Setup the goal
        # traj_goal = FollowJointTrajectoryGoal()
        # traj_goal.trajectory.header.stamp = (rospy.Time.now() +
        #                                      rospy.Duration(0.1))
        # traj_goal.trajectory.joint_names = self.joint_names
        # velocities = [0] * len(joints)
        # traj_goal.trajectory.points.append(JointTrajectoryPoint(
        #                 positions=joints,
        #                 velocities=velocities,
        #                 time_from_start=rospy.Duration(time_to_joint)))

        # self.traj_action_client.send_goal(traj_goal)


        ### Option 2, use MoveIt to move to joint goals

        # for pose in joints:
        # if rospy.is_shutdown():
        #     break

        # Plans the joints in joint_names to angles in pose
        # self.is_executing = True
        self.move_group.moveToJointPosition(self.joint_names, joints, wait=False)

        # Since we passed in wait=False above we need to wait here
        self.move_group.get_move_action().wait_for_result()
        result = self.move_group.get_move_action().get_result()

        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Disco!")
            else:
                # If you get to this point please search for:
                # moveit_msgs/MoveItErrorCodes.msg
                rospy.logerr("Arm goal in state: %s",
                             self.move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")


    def get_time_to_pose(self, target_pose):
        '''Returns the time to get to the arm pose held in target_pose.

        Args:
            target_pose (Pose|None): A Pose holding the pose to
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

    @staticmethod
    def _get_time_bw_poses(pose0, pose1, velocity=0.2):
        '''Determines how much time should be allowed for moving between
        pose0 and pose1 at velocity.

        Args:
            pose0 (Pose)
            pose1 (Pose)
            velocity (float, optional): Defaults to 0.2.

        Returns:
            float: How long (in seconds) to allow for moving between
                pose0 and pose1 and velocity.
        '''
        dist = Arm.get_distance_bw_poses(pose0, pose1)
        duration = dist / velocity
        return (
            DURATION_MIN_THRESHOLD if duration < DURATION_MIN_THRESHOLD
            else duration)


    #TODO
    def is_executing(self):
        '''Whether or not there is an ongoing action execution on the arm'''
        # return (self.move_group.get_move_action().get_state() == GoalStatus.ACTIVE
        #         or self.move_group.get_move_action().get_state() == GoalStatus.PENDING)
        return self.is_executing

    #TODO
    def is_successful(self):
        '''Whetehr the execution succeeded'''
        # return (self.move_group.get_move_action().get_state() == GoalStatus.SUCCEEDED)
        return

    def get_ik_for_ee(self, ee_pose, seed):
        ''' Finds the IK solution for given end effector pose'''
        joints = self._solve_ik(ee_pose, seed)
        ## If our seed did not work, try once again with the default seed
        if joints is None:
            #rospy.logwarn('Could not find IK solution with preferred seed,' +
            #              'will try default seed.')
            joints = self._solve_ik(ee_pose)

        # print "Joints: {}".format(joints)
        # print "Seed: {}".format(seed)

        if joints is None:
            pass
            #rospy.logwarn('IK out of bounds, will use the seed directly.')
        else:
            rollover = array((array(joints) - array(seed)) / pi, int)
            joints -= ((rollover + (sign(rollover) + 1) / 2) / 2) * 2 * pi

        return joints

    @staticmethod
    def get_distance_bw_poses(pose0, pose1):
        '''Returns the dissimilarity between two end-effector poses'''
        w_pos = 1.0
        w_rot = 0.2
        pos0 = array((pose0.position.x, pose0.position.y, pose0.position.z))
        pos1 = array((pose1.position.x, pose1.position.y, pose1.position.z))
        rot0 = array((pose0.orientation.x, pose0.orientation.y,
                      pose0.orientation.z, pose0.orientation.w))
        rot1 = array((pose1.orientation.x, pose1.orientation.y,
                      pose1.orientation.z, pose1.orientation.w))
        pos_dist = w_pos * norm(pos0 - pos1)
        rot_dist = w_rot * (1 - dot(rot0, rot1))

        if (pos_dist > rot_dist):
            dist = pos_dist
        else:
            dist = rot_dist
        return dist

    def reset_movement_history(self):
        ''' Clears the saved history of arm movements'''
        self.last_unstable_time = rospy.Time.now()
        self.arm_movement = []

    def get_movement(self):
        '''Returns cumulative movement in recent history'''
        return sum(self.arm_movement)

    def _record_arm_movement(self, reading):
        '''Records the sensed arm movement'''
        self.arm_movement = [reading] + self.arm_movement
        if (len(self.arm_movement) > self.movement_buffer_size):
            self.arm_movement = self.arm_movement[0:self.movement_buffer_size]

    # def _is_arm_moved_while_holding(self):
    #     '''Checks if user is trying to move the arm while it is stiff'''
    #     threshold = 0.02
    #     if (self.get_mode() == ArmMode.HOLD
    #             and (len(self.arm_movement) == self.movement_buffer_size)
    #             and (self.get_movement() > threshold)):
    #         return True
    #     return False

    # def _is_arm_stable_while_released(self):
    #     '''Checks if the arm has been stable while being released'''
    #     movement_threshold = 0.02
    #     time_threshold = rospy.Duration(5.0)
    #     is_arm_stable = (self.get_movement() < movement_threshold)
    #     if (not is_arm_stable or self.get_mode() == ArmMode.HOLD):
    #         self.last_unstable_time = rospy.Time.now()
    #         return False
    #     else:
    #         if (rospy.Time.now() - self.last_unstable_time) > time_threshold:
    #             return True
    #         else:
    #             return False

    def update(self, is_executing):
        ''' Periodical update for one arm'''
        ee_pose = self.get_ee_state()
        if (ee_pose != None and self.last_ee_pose != None):
            self._record_arm_movement(Arm.get_distance_bw_poses(ee_pose,
                                                        self.last_ee_pose))
        self.last_ee_pose = ee_pose

        # if (not is_executing and Arm._is_autorelease_on):
        #     if (self._is_arm_moved_while_holding()):
        #         rospy.loginfo('Automatically releasing arm.')
        #         self.set_mode(ArmMode.RELEASE)

        #     if (self._is_arm_stable_while_released()):
        #         rospy.loginfo('Automatically holding arm.')
        #         self.set_mode(ArmMode.HOLD)
