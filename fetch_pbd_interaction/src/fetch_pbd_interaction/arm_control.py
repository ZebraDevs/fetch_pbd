'''Control of the two arms for action execution.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import roslib
roslib.load_manifest('fetch_pbd_interaction')
import rospy

# System builtins
import threading
import time

# ROS builtins
from geometry_msgs.msg import Pose, Point

# Local
import actionlib
from fetch_arm_interaction.arm import Arm
from fetch_arm_interaction.msg import GripperState
from fetch_pbd_interaction.msg import ArmState, ActionStep, ExecutionStatus
from fetch_social_gaze.msg import GazeGoal
from response import Response
from world import World
from robot_controllers_msgs.msg import QueryControllerStatesAction, \
                                       QueryControllerStatesGoal, \
                                       ControllerState


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
    '''Class for things related to moving arms.'''

    arm = None

    def __init__(self, tf_listener):
        
        # Initialize arm state.
        arm = Arm(tf_listener)
        ArmControl.arm = arm            
        arm.check_gripper_state()
        arm.close_gripper()

        # Initialize Arms (joint) state.
        self.attended_arm = -1
        self.action = None
        self.preempt = False
        self.z_offset = 0.0
        self.status = ExecutionStatus.NOT_EXECUTING
        rospy.loginfo('Arm initialized.')

    # ##################################################################
    # Static methods: Public (API)
    # ##################################################################

    # @staticmethod
    # def set_arm_mode(arm_index, mode):
    #     '''Set arm to stiff or relaxed.

    #     Args:
    #         arm_index (int): Side.RIGHT or Side.LEFT
    #         mode (int): ArmMode.RELEASE or ArmMode.HOLD

    #     Retruns:
    #         bool: Whether the mode was changed.
    #     '''
    #     if mode == Arms.arms[arm_index].arm_mode:
    #         # Already in that mode
    #         return False
    #     else:
    #         Arms.arms[arm_index].set_mode(mode)
    #         return True

    @staticmethod
    def set_gripper_state(gripper_state):
        '''Set gripper of arm_index (right or left) to gripper_state
        (open or closed).

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT
            gripper_state (int): GripperState.OPEN or
                GripperState.CLOSED

        Returns:
            bool: Whether the gripper state was changed. For example,
                asking a closed gripper to close will return False, but
                asking a closed gripper to open will return True.
        '''
        if gripper_state == ArmControl.get_gripper_state():
            # Already in that mode; do nothing and return False.
            return False
        else:
            # Change gripper mode.
            if gripper_state == GripperState.OPEN:
                ArmControl.arm.open_gripper()
            else:
                ArmControl.arm.close_gripper()
            return True

    @staticmethod
    def solve_ik_for_arm(arm_state, z_offset=0.0):
        '''Finds an  IK solution for a particular arm pose.

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT
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

        rospy.loginfo("Solving ik for arm")
        if arm_state.refFrame == ArmState.OBJECT:
            # Arm is relative.
            solution = ArmState()
            target_pose = World.transform(
                arm_state.ee_pose, arm_state.refFrameLandmark.name, 'base_link')
            target_pose.position.z = target_pose.position.z + z_offset

            # Try solving IK.
            target_joints = ArmControl.arm.get_ik_for_ee(
                target_pose, arm_state.joint_pose)

            # Check whether solution found.
            if target_joints is None:
                # No solution: RETURN EMPTY ArmState.
                rospy.logdebug('No IK for relative end-effector pose.')
                return solution, False
            else:
                # Found a solution; update the solution arm_state and
                # return.
                solution.refFrame = ArmState.ROBOT_BASE
                solution.ee_pose = Pose(
                    target_pose.position, target_pose.orientation)
                solution.joint_pose = target_joints
                return solution, True
        elif arm_state.refFrame == ArmState.ROBOT_BASE:
            # Arm is absolute.
            pos = arm_state.ee_pose.position
            target_position = Point(pos.x, pos.y, pos.z + z_offset)
            target_pose = Pose(target_position, arm_state.ee_pose.orientation)

            # Try solving IK.

            # rospy.loginfo("target_pose: {}, joint_pose: {}".format(target_pose, arm_state.joint_pose))
            target_joints = ArmControl.arm.get_ik_for_ee(
                target_pose, arm_state.joint_pose)
            if target_joints is None:
                # No IK found; return the original.
                rospy.logdebug('No IK for absolute end-effector pose.')
                return arm_state, False
            else:
                # IK found; fill in solution ArmState and return.
                solution = ArmState()
                solution.refFrame = ArmState.ROBOT_BASE
                solution.ee_pose = Pose(
                    arm_state.ee_pose.position, arm_state.ee_pose.orientation)
                solution.joint_pose = target_joints
                return solution, True
        else:
            return arm_state, True

    @staticmethod
    def is_condition_met(condition):
        '''Returns whether the given pre-condition or post-condition is
        currently met.

        Args:
            condition (Condition): The pre or post condition contained
                in the ActionStep.

        Returns:
            bool: Whether the given pre/post-condition is met.
        '''
        # TODO(mcakmak): Implement.
        return True

    @staticmethod
    def get_joint_state():
        '''Get joint positions.

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT

        Returns:
            [float]: Array of seven floats, the positions of all arm
                joints of the arm_index arm.
        '''
        return ArmControl.arm.get_joint_state()

    @staticmethod
    def get_gripper_state():
        ''' Get gripper status on the indicated side.

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT

        Returns:
            int: GripperState.OPEN or GripperState.CLOSED
        '''
        return ArmControl.arm.get_gripper_state()

    @staticmethod
    def get_ee_state():
        ''' Get current pose of the arm's end-effector on the indicated
        side.

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT

        Returns:
            Pose|None: Pose if success, None if there was a failure in
                looking up the transform to ref_frame.
        '''
        return ArmControl.arm.get_ee_state()

    # ##################################################################
    # Static methods: Internal ("private")
    # ##################################################################

    # @staticmethod
    # def _get_most_moving_arm():
    #     '''Determines which of the two arms has moved more in the recent
    #     past.

    #     See Arm.get_movement() for definition of "recent"

    #     Returns:
    #         int: Side.RIGHT or Side.LEFT, the most moving arm, or -1 if
    #             neither arm has sufficiently moved.
    #     '''
    #     # TODO(mbforbes): Refactor with SIDES.
    #     if (Arms.arms[Side.RIGHT].get_movement() < ARM_MOVEMENT_THRESHOLD and
    #             Arms.arms[Side.LEFT].get_movement() < ARM_MOVEMENT_THRESHOLD):
    #         return -1
    #     elif Arms.arms[Side.RIGHT].get_movement() < ARM_MOVEMENT_THRESHOLD:
    #         return Side.LEFT
    #     else:
    #         return Side.RIGHT


    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def is_executing(self):
        '''Return whether there is an ongoing action execution.

        Returns:
            bool
        '''
        return (self.status == ExecutionStatus.EXECUTING)

    def start_execution(self, action, z_offset=0.0):
        ''' Starts execution of action.

        This method spawns a new thread.

        Args:
            action (ProgrammedAction): The action to execute.
            z_offset (float): Amount to add to z-values of pose
                positions.
        '''
        # This will take long; create a thread.
        self.action = action.copy()
        self.preempt = False
        self.z_offset = z_offset
        thread = threading.Thread(
            group=None,
            target=self.execute_action,
            name='action_execution_thread'
        )
        thread.start()

    def stop_execution(self):
        '''Preempts an ongoing execution.'''
        self.preempt = True

    def solve_ik_for_action(self):
        '''Computes joint positions for all end-effector poses in the
        currently stored action.

        Returns:
            bool: Whether IK could be successfully solved for the
                action.
        '''
        # Go over steps of the action, checking the type for each and
        # solving IK.

        rospy.loginfo("Solving ik for action")
        for i in range(self.action.n_frames()):
            # See whether this step is an arm target step.
            if self.action.seq.seq[i].type == ActionStep.ARM_TARGET:
                # Solve IK for both arms.
                arm, has_solution = ArmControl.solve_ik_for_arm(
                    self.action.seq.seq[i].armTarget.arm,
                    self.z_offset
                )

                self.action.seq.seq[i].armTarget.arm = arm

                # If doesn't have a solution, we return false.
                if not has_solution:
                    return False

            # See whether this step is an arm trajectory step.
            if self.action.seq.seq[i].type == ActionStep.ARM_TRAJECTORY:
                # If it's an arm trajectory, we have to check all arm
                # targets within the trajectory.
                n_frames = len(self.action.seq.seq[i].armTrajectory.timing)
                for j in range(n_frames):
                    # Solve IK for both arms.
                    arm, has_solution = ArmControl.solve_ik_for_arm(
                        self.action.seq.seq[i].armTrajectory.arm[j],
                        self.z_offset
                    )

                    self.action.seq.seq[i].armTrajectory.arm[j] = arm

                    # If doesn't have a solution, we return false
                    if not has_solution:
                        return False
        # Because no steps returned False, at the point everything is
        # good and we can signal a complete IK solution.
        return True

    def start_move_to_pose(self, arm_state):
        '''Creates a thread for moving to a target pose.

        Args:
            arm_state (ArmState): Arm state that contains the pose to
                move to.
            arm_index (int): Side.RIGHT or Side.LEFT
        '''
        self.preempt = False
        thread = threading.Thread(
            group=None,
            target=self.move_to_pose,
            args=(arm_state),
            name='move_to_arm_state_thread'
        )
        thread.start()

        # Log
        rospy.loginfo('Started thread to move arm.')

    def move_to_pose(self, arm_state):
        '''The thread function that makes the arm_index arm move to the
        target end-effector pose (within arm_state).

        Args:
            arm_state (ArmState): Arm state that contains the pose to
                move to.
            arm_index (int): Side.RIGHT or Side.LEFT
        '''

        rospy.loginfo("Move to pose")
        self.status = ExecutionStatus.EXECUTING
        solution, has_solution = ArmControl.solve_ik_for_arm(arm_state)
        if has_solution:
            # Do the raw movement (this only moves arm_index arm).
            is_successful = self.move_to_joints(solution)

            # Set status based on what happened.
            if is_successful:
                self.status = ExecutionStatus.SUCCEEDED
            else:
                self.status = ExecutionStatus.OBSTRUCTED
        else:
            self.status = ExecutionStatus.NO_IK

    def execute_action(self):
        ''' Function to replay the demonstrated two-arm action of type
        ProgrammedAction (must already be saved in this object).'''
        self.status = ExecutionStatus.EXECUTING
        action_step = self.action.get_step(0)

        rospy.loginfo("Starting to execute action!!!")

        # Make sure the step exists.
        if action_step is None:
            rospy.logwarn("First step does not exist.")
            self.status = ExecutionStatus.CONDITION_ERROR
        # Check if the very first precondition is met.
        elif not ArmControl.is_condition_met(action_step.preCond):
            rospy.logwarn(
                'First precond is not met, first make sure the robot is' +
                'ready to execute action (hand object or free hands).')
            self.status = ExecutionStatus.CONDITION_ERROR
        else:
            # Check that all parts of the action are reachable
            if not self.solve_ik_for_action():
                rospy.logwarn('Problem finding IK solutions.')
                self.status = ExecutionStatus.NO_IK
            else:
                # Freeze both arms, then execute all steps in turn.
                self._loop_through_action_steps()

            ArmControl.arm.reset_movement_history()

            # If we haven't been preempted, we now report success.
            if self.status == ExecutionStatus.EXECUTING:
                self.status = ExecutionStatus.SUCCEEDED
                rospy.loginfo('Action execution has succeeded.')

    def move_to_joints(self, arm_state):
        '''
        Makes the arms move to the joint positions contained in the
        passed arm states.

        This subsumes that the joint positions are valid (i.e. IK has
        been called previously to set each ArmState's joints to good
        values).

        Args:
            r_arm (ArmState)
            l_arm (ArmState)

        Returns:
            bool: Whether the arms successfully moved to the passed
                joint positions.
        '''
        # Estimate times to get to both poses.
        time_to_pose = None

        if arm_state is not None:
            time_to_pose = self.arm.get_time_to_pose(arm_state.ee_pose)

        # If both arms are moving, adjust velocities and find most
        # moving arm. Look at it.
        is_moving = time_to_pose is not None

        Response.look_at_point(arm_state.ee_pose.position)

        # Move arms to target.
        if is_moving:
            ArmControl.arm.move_to_joints(
                arm_state.joint_pose, time_to_pose)

        # Wait until both arms complete the trajectory.
        while(ArmControl.arm.is_executing() and not self.preempt):
            rospy.sleep(MOVE_TO_JOINTS_SLEEP_INTERVAL)

        rospy.loginfo('\tArms reached target.')

        # Verify that both arms succeeded
        # DEBUG: remove
        suc = ArmControl.arm.is_successful()

        if (is_moving and not suc):
            return False
        else:
            return True


    @staticmethod
    def _get_most_moving_arm():
        '''Determines which of the two arms has moved more in the recent
        past.

        See Arm.get_movement() for definition of "recent"

        Returns:
            int: Side.RIGHT or Side.LEFT, the most moving arm, or -1 if
                neither arm has sufficiently moved.
        '''
        # TODO(mbforbes): Refactor with SIDES.
        if (ArmControl.arm.get_movement() < ARM_MOVEMENT_THRESHOLD):
            return -1
        else:
            # rospy.loginfo("Arm is moving!!!")
            return ArmControl.arm

    def update(self):
        '''Periodic update for the two arms.

        This is called regularly by the update loop in interaction.
        '''
        ArmControl.arm.update(self.is_executing())

        # This could be ArmControl.arm or something
        moving_arm = ArmControl._get_most_moving_arm() 
        if moving_arm != self.attended_arm and not self.is_executing():
            if moving_arm == -1:
                Response.perform_gaze_action(GazeGoal.LOOK_FORWARD)
            else:
                # moving_arm == Side.LEFT
                Response.perform_gaze_action(GazeGoal.FOLLOW_EE)
            self.attended_arm = moving_arm

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _loop_through_action_steps(self):
        '''Goes through the steps of the current action and moves to
        each.'''
        # Go over steps of the action
        for i in range(self.action.n_frames()):
            rospy.loginfo('Executing step ' + str(i))
            action_step = self.action.get_step(i)

            # Make sure step exists.
            if action_step is None:
                rospy.logwarn("Step " + str(i) + " does not exist.")
                self.status = ExecutionStatus.CONDITION_ERROR
                break
            # Check that preconditions are met
            elif not ArmControl.is_condition_met(action_step.preCond):
                rospy.logwarn(
                    '\tPreconditions of action step ' + str(i) + ' are not ' +
                    'satisfied. Aborting.')
                self.status = ExecutionStatus.CONDITION_ERROR
                break
            else:
                # Try executing.
                if not self._execute_action_step(action_step):
                    break

                # Finished executing; check that postconditions are met
                if ArmControl.is_condition_met(action_step.postCond):
                    rospy.loginfo('\tPost-conditions of the action are met.')
                else:
                    rospy.logwarn(
                        '\tPost-conditions of action step ' + str(i) +
                        ' are not satisfied. Aborting.')
                    self.status = ExecutionStatus.CONDITION_ERROR
                    break

            # Perhaps the execution was pre-empted by the user. Check
            # this before continuing onto the next step.
            if self.preempt:
                rospy.logwarn('\tExecution preempted by user.')
                self.status = ExecutionStatus.PREEMPTED
                break

            # Step completed successfully.
            rospy.loginfo('\tStep ' + str(i) + ' of action is complete.')

    def _execute_action_step(self, action_step):
        '''Executes the motion part of an action step.

        Args:
            action_step (ActionStep): The next action step to execute
                (represents either an arm target or trajectory).

        Returns:
            bool: Whether the step was successfully executed.
        '''
        # For each step, check step type.
        if action_step.type == ActionStep.ARM_TARGET:
            # Arm target.
            rospy.loginfo('\tWill perform arm target action step.')
            # Try moving to the joints.
            if not self.move_to_joints(action_step.armTarget.arm):
                # We may have been pre-empted.
                if self.preempt:
                    self.status = ExecutionStatus.PREEMPTED
                # Otherwise, we were obstructed.
                else:
                    self.status = ExecutionStatus.OBSTRUCTED
                # Regardless, couldn't get to the joints; return False.
                return False
        elif action_step.type == ActionStep.ARM_TRAJECTORY:
            # Arm trajectory.
            rospy.loginfo('\tWill perform arm trajectory action step.')
            # First move to the start frame.
            if not self.move_to_joints(
                    action_step.armTrajectory.arm[0]):
                # We may have been pre-empted.
                if self.preempt:
                    self.status = ExecutionStatus.PREEMPTED
                # Otherwise, we were obstructed.
                else:
                    self.status = ExecutionStatus.OBSTRUCTED
                # Regardless, couldn't move to the start frame; return
                # False.
                return False

            # Then execute the trajectory.
            ArmControl.arm.execute_joint_traj(
                action_step.armTrajectory.arm,
                action_step.armTrajectory.timing
            )

            # Wait until both arms complete the trajectory.
            while(ArmControl.arm.is_executing()  and not self.preempt):
                rospy.sleep(TRAJECTORY_COMPLETE_SLEEP_INTERVAL)
            rospy.loginfo('\tTrajectory complete.')

            # Verify that both arms succeeded.
            if (not ArmControl.arm.is_successful()):
                rospy.logwarn(
                    '\tAborting execution; arms failed to follow trajectory.')
                # We may have been pre-empted.
                if self.preempt:
                    self.status = ExecutionStatus.PREEMPTED
                # Otherwise, we were obstructed.
                else:
                    self.status = ExecutionStatus.OBSTRUCTED
                # Regardless, couldn't complete trajectory; return
                # False.
                return False

        # If hand action, do it for both sides.
        if (action_step.gripperAction.gripper.state !=
                ArmControl.arm.get_gripper_state()):
            # TODO(mbforbes): Make this logging better (output 'close'
            # or 'open' instead of numbers).
            rospy.loginfo(
                '\tWill perform right gripper action ' +
                str(action_step.gripperAction.gripper.state))
            ArmControl.arm.set_gripper(
                action_step.gripperAction.gripper.state)
            Response.perform_gaze_action(GazeGoal.FOLLOW_EE)

        # Wait for grippers to be done
        while (ArmControl.arm.is_gripper_moving()):
            rospy.sleep(GRIPPER_FINISH_SLEEP_INTERVAL)
        rospy.loginfo('\tHands done moving.')

        # Verify that both grippers succeeded
        if (not ArmControl.arm.is_gripper_at_goal()):
            rospy.logwarn('\tHand(s) did not fully close or open!')

        # Everything completed successfully!
        return True
