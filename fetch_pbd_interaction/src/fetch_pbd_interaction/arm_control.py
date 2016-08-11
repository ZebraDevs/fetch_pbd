'''Control of the arm for action execution.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# System builtins
import threading

# ROS builtins
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from tf import TransformListener

# Local
from fetch_arm_control import Arm
from fetch_arm_control.msg import GripperState
from fetch_pbd_interaction.msg import ArmState, ExecutionStatus
from fetch_pbd_interaction.srv import MoveArm, MoveArmResponse, \
                                      GetEEPose, GetEEPoseResponse, \
                                      GetGripperState, \
                                      GetGripperStateResponse, \
                                      SetGripperState, \
                                      SetGripperStateResponse, \
                                      GetJointStates, GetJointStatesResponse, \
                                      GetArmMovement, GetArmMovementResponse, \
                                      MoveArmTraj, MoveArmTrajResponse

# ######################################################################
# Module level constants
# ######################################################################

# The minimum arm movement that 'counts' as moved.
# TODO(mbforbes): This is duplicated in arm.py. Use theirs.
ARM_MOVEMENT_THRESHOLD = 0.04

# How long to sleep between checking whether arms have successfully
# completed their movement.
MOVE_TO_JOINTS_SLEEP_INTERVAL = 0.01  # seconds

# How long to sleep between checking whether arms have successfully
# completed their trajectory.
TRAJECTORY_COMPLETE_SLEEP_INTERVAL = 0.01  # seconds

# How long to sleep between checking whether grippers have finished
# opening/closing.
GRIPPER_FINISH_SLEEP_INTERVAL = 0.01  # seconds


# ######################################################################
# Classes
# ######################################################################

class ArmControl:
    ''''Control of the arm for action execution.'''

    def __init__(self):

        # Initialize arm state.
        self._tf_listener = TransformListener()
        self._arm = Arm(self._tf_listener)
        self._arm.close_gripper()
        self._status = ExecutionStatus.NOT_EXECUTING

        rospy.Service('move_arm_to_joints_plan', MoveArm, self._move_to_joints_plan)
        rospy.Service('move_arm_to_joints', MoveArmTraj, self._move_to_joints)

        rospy.Service('move_arm_to_pose', MoveArm, self._move_to_pose)
        rospy.Service('start_move_arm_to_pose', MoveArm,
                      self._start_move_to_pose)

        rospy.Service('is_reachable', MoveArm, self._is_reachable)
        rospy.Service('is_arm_moving', GetArmMovement, self._is_arm_moving)


        rospy.Service('relax_arm', Empty, self._relax_arm)

        rospy.Service('reset_arm_movement_history', Empty,
                      self._reset_movement_history)

        rospy.Service('get_gripper_state', GetGripperState,
                      self._get_gripper_state)
        rospy.Service('get_ee_pose', GetEEPose, self._get_ee_pose)
        rospy.Service('get_joint_states', GetJointStates,
                      self._get_joint_states)

        rospy.Service('set_gripper_state', SetGripperState,
                      self._set_gripper_state)

        rospy.loginfo('Arm initialized.')

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def update(self):
        '''Periodic update for the arm.'''
        self._arm.update()

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _reset_movement_history(self, req):
        '''
        Args:
            req (EmptyRequest)
        Returns:
            EmptyResponse
        '''

        self._arm.reset_movement_history()
        return EmptyResponse()

    def _set_gripper_state(self, req):
        '''Set gripper to gripper_state
        (open or closed).

        Args:
            req (SetGripperStateRequest)

        Returns:
            SetGripperStateResponse
        '''
        gripper_state = req.gripper_state
        if gripper_state == self._arm.get_gripper_state():
            # Already in that mode; do nothing and return False.
            return SetGripperStateResponse()
        else:
            # Change gripper mode.
            if gripper_state == GripperState.OPEN:
                self._arm.open_gripper()
            else:
                self._arm.close_gripper()
            return SetGripperStateResponse()

    def _is_reachable(self, req):
        '''.

        Args:
            req (MoveArmRequest): The arm's state

        Returns:
            MoveArmResponse
        '''
        arm_state = req.arm_state
        solution, reachable = self._solve_ik_for_arm(arm_state)

        return MoveArmResponse(reachable)

    def _solve_ik_for_arm(self, arm_state, z_offset=0.0):
        '''Finds an  IK solution for a particular arm pose.

        Args:
            arm_state (ArmState): The arm's state,
            z_offset (float, optional): Offset to add to z-values of
                pose positions. Defaults to 0.0.

        Returns:
            (ArmState, bool): Tuple of

                the ArmState, which will be an updated ArmState object
                with IK solved if an IK solution was found. If IK was
                not found, what is returned will depend on whether the
                reference frame is relative (ArmState.OBJECT) or
                absolute (ArmState.ROBOT_BASE). If the reference frame
                is relative and no IK was found, an empty ArmState is
                returned. If the reference frame is absolute and no IK
                was found, then the original passed arm_state is
                returned;

                the bool, which is whether an IK solution was
                successfully found.
        '''
        # Ideally we need to find IK only if the frame is relative to an
        # object, but users can edit absolute poses in the GUI to make
        # them unreachable, so we try IK for absolute poses too.

        if arm_state.ref_type == ArmState.OBJECT:
            # Arm is relative.
            solution = ArmState()

            target_pose = self._tf_listener.transformPose(
                'base_link', arm_state.ee_pose)
            target_pose.pose.position.z = target_pose.pose.position.z + \
                                            z_offset

            # Try solving IK.
            target_joints = self._arm.get_ik_for_ee(
                target_pose, arm_state.joint_pose)

            # Check whether solution found.
            if target_joints is None:
                # No solution: RETURN EMPTY ArmState.
                rospy.loginfo('No IK for relative end-effector pose.')
                return solution, False
            else:
                # Found a solution; update the solution arm_state and
                # return.
                solution.ref_type = ArmState.ROBOT_BASE
                solution.ee_pose = target_pose
                solution.joint_pose = target_joints
                return solution, True
        elif arm_state.ref_type == ArmState.ROBOT_BASE:
            # Arm is absolute.

            target_pose = arm_state.ee_pose
            target_pose.pose.position.z = target_pose.pose.position.z + \
                                            z_offset

            # Try solving IK.
            target_joints = self._arm.get_ik_for_ee(
                target_pose, arm_state.joint_pose)
            if target_joints is None:
                # No IK found; return the original.
                rospy.logdebug('No IK for absolute end-effector pose.')
                return arm_state, False
            else:
                # IK found; fill in solution ArmState and return.
                solution = ArmState()
                solution.ref_type = ArmState.ROBOT_BASE
                solution.ee_pose = target_pose
                solution.joint_pose = target_joints
                return solution, True
        else:
            return arm_state, True

    def _get_joint_states(self, req):
        '''Get joint positions.

        Args:
            req (GetJointStatesRequest)

        Returns:
            GetJointStatesResponse
        '''

        return GetJointStatesResponse(self._arm.get_joint_state())

    def _get_gripper_state(self, req):
        ''' Get gripper status on the indicated side.

        Args:
            req (GetGripperStateRequest)

        Returns:
            GetGripperStateResponse
        '''
        return GetGripperStateResponse(self._arm.get_gripper_state())


    def _get_ee_pose(self, req):
        ''' Get current pose of the arm's end-effector on the indicated
        side.

        Args:
            req (GetEEPoseRequest)

        Returns:
            GetEEPoseResponse
        '''
        return GetEEPoseResponse(self._arm.get_ee_state())

    def _relax_arm(self, req):
        ''' Turns on gravity comp controller and turns off other controllers

        Args:
            req (EmptyRequest): Unused

        Returns:
            EmptyResponse
        '''

        self._arm.relax_arm()

        return EmptyResponse()

    def _start_move_to_pose(self, req):
        '''Creates a thread for moving to a target pose.

        Args:
            req (MoveArmRequest): Arm state that contains the pose to
                move to.
        '''
        thread = threading.Thread(
            group=None,
            target=self._move_to_pose,
            args=(req,),
            name='move_to_arm_state_thread'
        )
        thread.start()

        # Log
        rospy.loginfo('Started thread to move arm.')

        return MoveArmResponse(True)

    def _move_to_pose(self, req):
        '''The thread function that makes the arm move to the
        target end-effector pose (within arm_state).

        Args:
            req (MoveArmRequest): Arm state that contains the pose to
                move to.
        '''
        arm_state = req.arm_state
        rospy.loginfo("Move to pose")
        self._status = ExecutionStatus.EXECUTING

        # Should we transform pose to base_link?
        if self._arm.move_to_pose(arm_state.ee_pose):
            self._status = ExecutionStatus.SUCCEEDED
            success = True
        else:
            self._status = ExecutionStatus.NO_IK
            success = False

        self._arm.relax_arm()
        return MoveArmResponse(success)

    def _move_to_joints_plan(self, req):
        '''
        Makes the arms move to the joint positions contained in the
        passed arm states.

        This assumes that the joint positions are valid (i.e. IK has
        been called previously to set each ArmState's joints to good
        values).

        Args:
            req (MoveArmRequest)

        Returns:
            MoveArmResponse: Whether the arms successfully moved to the passed
                joint positions.
        '''
        # Estimate times to get to both poses.
        arm_state = req.arm_state

        # Move arms to target.

        self._status = ExecutionStatus.EXECUTING
        suc = self._arm.move_to_joints_plan(arm_state.joint_pose, arm_state.velocities)


        # Wait until both arms complete the trajectory.
        while self._arm.is_executing():
            rospy.sleep(MOVE_TO_JOINTS_SLEEP_INTERVAL)

        rospy.loginfo('\tArms reached target.')

        # Verify that both arms succeeded
        # DEBUG: remove

        if not suc:
            self._status = ExecutionStatus.NO_IK
            success = False
        else:
            self._status = ExecutionStatus.SUCCEEDED
            success = True

        self._arm.relax_arm()
        return MoveArmResponse(success)

    def _move_to_joints(self, req):
        '''
        Makes the arms move to the joint positions contained in the
        passed arm states.

        Note: This moves directly to the joint positions using interpolation

        This assumes that the joint positions are valid (i.e. IK has
        been called previously to set each ArmState's joints to good
        values).

        Args:
            req (MoveArmRequest)

        Returns:
            MoveArmResponse: Whether the arms successfully moved to the passed
                joint positions.
        '''
        # Estimate times to get to both poses.
        joints = []
        # times_to_poses = []
        # arm_state = req.arm_state[i]

        # time_to_pose = None

        # if arm_state is not None:
        #     time_to_pose = self._arm.get_time_to_pose(arm_state.ee_pose)
        for arm_state in req.arm_states:
            joints.append(arm_state.joint_pose)

        # If both arms are moving, adjust velocities and find most
        # moving arm. Look at it.

        # Move arms to target.
        suc = False
        self._status = ExecutionStatus.EXECUTING
        suc = self._arm.move_to_joints(
            joints, req.times)

        # Wait until both arms complete the trajectory.
        while self._arm.is_executing():
            rospy.sleep(MOVE_TO_JOINTS_SLEEP_INTERVAL)

        rospy.loginfo('\tArms reached target.')

        # Verify that both arms succeeded
        # DEBUG: remove

        if not suc:
            self._status = ExecutionStatus.NO_IK
            success = False
        else:
            self._status = ExecutionStatus.SUCCEEDED
            success = True

        self._arm.relax_arm()
        return MoveArmTrajResponse(success)


    def _is_arm_moving(self, req):
        '''
        Returns true is arm is moving

        Args:
            req (IsArmMovingRequest)

        Returns:
            IsArmMovingResponse: Whether the arm is currently moving
        '''

        if self._arm.get_movement() < ARM_MOVEMENT_THRESHOLD:
            return GetArmMovementResponse(False)
        else:
            return GetArmMovementResponse(True)
