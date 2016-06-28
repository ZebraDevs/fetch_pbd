#!/usr/bin/env python

'''End-to-end tests for PbD.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
PKG = 'pr2_pbd_interaction'
import roslib
roslib.load_manifest(PKG)
import rospy

# System builtins
import sys
from collections import Counter

# ROS builtins
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

# ROS 3rd party
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from sound_play.msg import SoundRequest

# Local
from pr2_pbd_interaction.msg import Side, GuiCommand, GripperState
from pr2_pbd_speech_recognition.msg import Command
from pr2_pbd_interaction.srv import Ping
from robot_speech import RobotSpeech
import unittest


# ######################################################################
# Constants
# ######################################################################

# How long to wait before any tests are run to let the system start up.
# Note that we do explicitly wait until the interaction node is
# initialized, but we can't easily do this for all of the nodes that
# it depends on.
PAUSE_STARTUP = 8.0

# How long to pause when starting a new test before publishing messages.
# If we start publishing messages immediately they are dropped by ROS or
# unseen for some reason.
PAUSE_SECONDS = 2.0

# How long to allow for a command response to come in.
DEFAULT_CMD_RESP_TIMEOUT = 1.0

# The robot's social gazes take a long time in simulation. This can
# screw up things like recording object poses, as that code doesn't wait
# for the robot's head to fully look down before it tries to capture and
# use a point cloud. This is the amount we wait before calling a 'record
# object pose' command to allow the head movement to settle.
HEAD_SETTLING_WAIT_TIME = 5.0

# How long to allow for a "record object pose" operation.
RECORD_OBJECT_POSE_TIMEOUT = 20.0

# How long to allow for the "end of execution" reporting robot speech.
EXECUTION_END_RESPONSE_TIMEOUT = 10.0

# How long to wait in-between querying the recorded joint states for its
# value. Note that the joints themselves publish updates at ~90Hz.
JOINT_REFRESH_PAUSE_SECONDS = 0.1

# How long to wait in-between querying the robot speech response tracker
# for an update.
RESPONSE_REFRESH_PAUSE_SECONDS = 0.1

# We want to trim certain responses that we get because they have
# specific information (like the "5" in "Switched to action 5") that
# will mess up tests depending on the order they are run. This
# information is also not necessary to ensure correct execution, so it's
# OK to remove.
RESPONSE_TRIMS = [
    RobotSpeech.SKILL_CREATED,
    RobotSpeech.SWITCH_SKILL,
    RobotSpeech.ERROR_NEXT_SKILL,
    RobotSpeech.ERROR_PREV_SKILL,
    RobotSpeech.START_EXECUTION,
    RobotSpeech.EXECUTION_ERROR_NOIK,
    RobotSpeech.EXECUTION_ERROR_NOPOSES,
    # The following will have "Pose saved" sloppily appended to them
    # if an action has been created.
    RobotSpeech.RIGHT_HAND_OPENING,
    RobotSpeech.RIGHT_HAND_CLOSING,
    RobotSpeech.LEFT_HAND_OPENING,
    RobotSpeech.LEFT_HAND_CLOSING,
    RobotSpeech.STOPPED_RECORDING_MOTION,
]

# Sides (right and left)
SIDES = ['r', 'l']

# Joint postfixes: gripper
GRIPPER_JOINT_POSTFIX = '_gripper_joint'

# Joint postfixes: arm
ARM_CONTROL_JOINT_POSTFIXES = [
    '_shoulder_pan_joint',
    '_shoulder_lift_joint',
    '_upper_arm_roll_joint',
    '_elbow_flex_joint',
    '_forearm_roll_joint',
    '_wrist_flex_joint',
    '_wrist_roll_joint'
]

# All postfixes
ALL_JOINT_POSTFIXES = [GRIPPER_JOINT_POSTFIX] + ARM_CONTROL_JOINT_POSTFIXES

# Combine with the side (left or right) to get full names. This is a
# single flat list (because we don't nest brackets).
RELEVANT_JOINTS = [
    side + postfix for side in SIDES for postfix in ALL_JOINT_POSTFIXES]

# Joint settings---numbers that indicate joint status. Note that there
# is +/- 0.01 error in these.
GRIPPER_OPEN_POSITION = 0.08
GRIPPER_CLOSE_POSITION = 0.00
GRIPPER_EPSILON_POSITION = 0.01
GRIPPER_TOGGLE_TIME_SECONDS = 10.0

# Most arm joints are set to this position (or its negative) when moving
# the arms around.
ARM_OUT_PAN = 1.2
ARM_UP_FLEX = -1.0
ARM_UP_POSITION = 0.5

# No idea what is reasonable here; PbD seems to use 0.
ARM_MOVE_VELOCITY = 0.0

# How long to pause after arm movement to let it stabilize (stop
# swinging around)
ARM_MOVE_PAUSE = 2.0

# Portions (from 0.0 to 1.0) to move the arm 'up' in a simple test
# execution.
SIMPLE_EXECUTION_PORTIONS = [0.1, 0.3, 0.6, 0.9]

# We want to mirror joint positions for the right arm.
SIDE_MULS = {'l': 1.0, "r": -1.0}

# How long to wait for each step (saved pose) in an execution, in
# seconds.
EXECUTION_STEP_TIME = 6.0

# How long to wait for each step (trajectory) in an execution, in
# seconds.
TRAJ_STEP_TIME = EXECUTION_STEP_TIME + ARM_MOVE_PAUSE

# How close arm joints have to be to match.
ARM_EPSILON_POSITION = 0.02

# ROS topics
TOPIC_INT_PING = '/interaction_ping'
TOPIC_CMD = '/recognized_command'
TOPIC_GUICMD = '/gui_command'
TOPIC_JOINTS = '/joint_states'
TOPIC_SPEECH = '/robotsound'
ARM_CONTROLLER_POSTFIX = '_arm_controller/joint_trajectory_action'


# ######################################################################
# Classes
# ######################################################################

class TestEndToEnd(unittest.TestCase):
    '''End-to-end tests for PbD.'''

    # ##################################################################
    # Core test infrastructure
    # ##################################################################

    def setUp(self):
        '''Ensures there is a valid interaction instance and wait until
        it's ready to rumble.'''
        # Ensure the interaction node is ready.
        rospy.wait_for_service('/interaction_ping')

        # Initialize some state.
        self.joint_positions = {}
        for joint in RELEVANT_JOINTS:
            self.joint_positions[joint] = None

        # Set up map of arm control joint names.
        self.arm_control_joints = {}
        for side in SIDES:
            self.arm_control_joints[side] = [
                side + postfix for postfix in ARM_CONTROL_JOINT_POSTFIXES]

        # Set up Counter to track robot speech.
        self.speech_tracker = Counter()

        # Create our ROS message machinery.
        # Keep alive ping.
        self.ping_srv = rospy.ServiceProxy(TOPIC_INT_PING, Ping)
        # For publishing speech/GUI commands.
        self.command_pub = rospy.Publisher(TOPIC_CMD, Command)
        # For publishing GUI-only commands.
        self.gui_command_pub = rospy.Publisher(TOPIC_GUICMD, GuiCommand)
        # For tracking gripper/arm states.
        rospy.Subscriber(TOPIC_JOINTS, JointState, self.joint_states_cb)
        # For tracking robot speech (its responses).
        rospy.Subscriber(TOPIC_SPEECH, SoundRequest, self.speech_heard_cb)
        # Set up controllers to move arms.
        self.arm_controllers = {}
        for side in SIDES:
            controller_name = '/' + side + ARM_CONTROLLER_POSTFIX
            self.arm_controllers[side] = SimpleActionClient(
                controller_name, JointTrajectoryAction)
            rospy.loginfo('Waiting for ' + side + ' arm server.')
            self.arm_controllers[side].wait_for_server()
            rospy.loginfo('Got response from ' + side + ' arm server.')

        # Apparently need to wait a bit even here, or messages get
        # dropped.
        rospy.sleep(PAUSE_SECONDS)

    # ##################################################################
    # Tests
    # ##################################################################

    def test_a_noaction_branches(self):
        '''This ideally exercises the "sorry, no action created yet"
        code that prevents requests from going through.

        The name "_a_" is in this test so that it runs first. This is
        because in PbD, once you create actions, you can never delete
        them, and it's not worth tearing down / bringing up PbD for
        each test case (it's also difficult to launch ROS nodes from
        within testing code).

        If this test does not run first because the test infracture
        changes, the code should be changed to just publish commands
        rather than check their response (the commands should all
        'work', i.e. not crash the system, but their responses will
        change depending on the order of the tests).
        '''
        # Ensure things are ready to go.
        self.check_alive()

        # Switch to nonexistant action
        self.guicmd_assert_response(
            GuiCommand.SWITCH_TO_ACTION, 50, [RobotSpeech.ERROR_NO_SKILLS])

        # Switch to nonexistant step. Note: The robot does not respond
        # to this command, it just affects the GUI, so here we're just
        # testing for the lack of a crash.
        self.guicmd_assert_response(
            GuiCommand.SELECT_ACTION_STEP, 50, [RobotSpeech.ERROR_NO_SKILLS])

        # Switch around (nonexistant) actions
        self.cmd_assert_response(
            Command.NEXT_ACTION, [RobotSpeech.ERROR_NO_SKILLS])
        self.cmd_assert_response(
            Command.PREV_ACTION, [RobotSpeech.ERROR_NO_SKILLS])

        # Do naughty things within nonexistant action.
        self.cmd_assert_response(
            Command.DELETE_LAST_STEP, [RobotSpeech.ERROR_NO_SKILLS])
        self.cmd_assert_response(
            Command.DELETE_ALL_STEPS, [RobotSpeech.ERROR_NO_SKILLS])
        self.cmd_assert_response(
            Command.START_RECORDING_MOTION, [RobotSpeech.ERROR_NO_SKILLS])
        self.cmd_assert_response(
            Command.SAVE_POSE, [RobotSpeech.ERROR_NO_SKILLS])
        self.cmd_assert_response(
            Command.START_RECORDING_MOTION, [RobotSpeech.ERROR_NO_SKILLS])
        # No explicit check for record object pose, but it doesn't hurt
        rospy.sleep(HEAD_SETTLING_WAIT_TIME)
        self.cmd_assert_response(
            Command.RECORD_OBJECT_POSE, [
                RobotSpeech.START_STATE_RECORDED,
                RobotSpeech.OBJECT_NOT_DETECTED,
            ],
            RECORD_OBJECT_POSE_TIMEOUT
        )
        self.cmd_assert_response(
            Command.EXECUTE_ACTION, [RobotSpeech.ERROR_NO_SKILLS])

        # Now make a single action and try bad switches.
        self.cmd_assert_response(
            Command.CREATE_NEW_ACTION, [RobotSpeech.SKILL_CREATED])
        self.cmd_assert_response(
            Command.NEXT_ACTION, [RobotSpeech.ERROR_NEXT_SKILL])
        self.cmd_assert_response(
            Command.PREV_ACTION, [RobotSpeech.ERROR_PREV_SKILL])

        # Make sure nothing's crashed.
        self.check_alive()

    def test_stop_execution(self):
        '''Test name says it all. Extremely simple.'''
        # Ensure things are ready to go.
        self.check_alive()

        # Stop execution while not executing, no steps.
        self.cmd_assert_response(
            Command.CREATE_NEW_ACTION, [RobotSpeech.SKILL_CREATED])
        self.cmd_assert_response(
            Command.STOP_EXECUTION, [RobotSpeech.ERROR_NO_EXECUTION])

        # Try starting and stopping (shouldn't run as no steps).
        self.cmd_assert_response(
            Command.EXECUTE_ACTION, [RobotSpeech.EXECUTION_ERROR_NOPOSES])
        self.cmd_assert_response(
            Command.STOP_EXECUTION, [RobotSpeech.ERROR_NO_EXECUTION])

        # Make some steps, "stop."
        for i in range(4):
            self.cmd_assert_response(
                Command.SAVE_POSE, [RobotSpeech.STEP_RECORDED])
        self.cmd_assert_response(
            Command.STOP_EXECUTION, [RobotSpeech.ERROR_NO_EXECUTION])

        # We'll track the execution preempted response, but we get the
        # pre-count here to avoid a race condition.
        prev_stopped = self.build_prev_resp_map(
            [RobotSpeech.EXECUTION_PREEMPTED])

        # Now actually start executing and stop.
        self.cmd_assert_response(
            Command.EXECUTE_ACTION, [RobotSpeech.START_EXECUTION])
        rospy.sleep(1)  # Pause slightly for it to start.
        self.cmd_assert_response(
            Command.STOP_EXECUTION, [RobotSpeech.STOPPING_EXECUTION])

        # Make sure it reports execution preempted eventually.
        self.assert_response(prev_stopped, EXECUTION_END_RESPONSE_TIMEOUT)

        # Make sure nothing's crashed.
        self.check_alive()

    def test_freeze_relax_arm(self):
        '''Tests the freeze and relax arm functionality.'''
        # Ensure things are ready to go.
        self.check_alive()

        # Relax both (could be in either state to start).
        self.cmd_assert_response(
            Command.RELAX_RIGHT_ARM, [
                RobotSpeech.RIGHT_ARM_RELEASED,
                RobotSpeech.RIGHT_ARM_ALREADY_RELEASED
            ]
        )
        self.cmd_assert_response(
            Command.RELAX_LEFT_ARM, [
                RobotSpeech.LEFT_ARM_RELEASED,
                RobotSpeech.LEFT_ARM_ALREADY_RELEASED
            ]
        )

        # Relax *again*
        self.cmd_assert_response(
            Command.RELAX_RIGHT_ARM, [RobotSpeech.RIGHT_ARM_ALREADY_RELEASED])
        self.cmd_assert_response(
            Command.RELAX_LEFT_ARM, [RobotSpeech.LEFT_ARM_ALREADY_RELEASED])

        # Freeze both
        self.cmd_assert_response(
            Command.FREEZE_RIGHT_ARM, [RobotSpeech.RIGHT_ARM_HOLDING])
        self.cmd_assert_response(
            Command.FREEZE_LEFT_ARM, [RobotSpeech.LEFT_ARM_HOLDING])

        # Freeze *again*
        self.cmd_assert_response(
            Command.FREEZE_RIGHT_ARM,
            [RobotSpeech.RIGHT_ARM_ALREADY_HOLDING])
        self.cmd_assert_response(
            Command.FREEZE_LEFT_ARM,
            [RobotSpeech.LEFT_ARM_ALREADY_HOLDING])

        # Make sure nothing's crashed.
        self.check_alive()

    def test_gripper_open_close(self):
        '''Tests that issuing 'speech' commands to open and close the
        gripper puts them in the desired state.'''
        # Ensure things are ready to go.
        self.check_alive()

        # Check opening hands works by checking robot speech response
        # and gripper positions.
        self.cmd_assert_response(
            Command.OPEN_RIGHT_HAND, [
                RobotSpeech.RIGHT_HAND_ALREADY_OPEN,
                RobotSpeech.RIGHT_HAND_OPENING
            ]
        )
        self.cmd_assert_response(
            Command.OPEN_LEFT_HAND, [
                RobotSpeech.LEFT_HAND_ALREADY_OPEN,
                RobotSpeech.LEFT_HAND_OPENING
            ]
        )
        for joint in [side + GRIPPER_JOINT_POSTFIX for side in SIDES]:
            self.assertJointCloseWithinTimeout(
                joint,
                GRIPPER_OPEN_POSITION,
                GRIPPER_EPSILON_POSITION,
                GRIPPER_TOGGLE_TIME_SECONDS
            )

        # And close.
        self.cmd_assert_response(
            Command.CLOSE_RIGHT_HAND, [RobotSpeech.RIGHT_HAND_CLOSING])
        self.cmd_assert_response(
            Command.CLOSE_LEFT_HAND, [RobotSpeech.LEFT_HAND_CLOSING])
        for joint in [side + GRIPPER_JOINT_POSTFIX for side in SIDES]:
            self.assertJointCloseWithinTimeout(
                joint,
                GRIPPER_CLOSE_POSITION,
                GRIPPER_EPSILON_POSITION,
                GRIPPER_TOGGLE_TIME_SECONDS
            )

        # Make sure nothing's crashed.
        self.check_alive()

    def test_double_open_close(self):
        '''Tests that issuing a gripper open / close command twice
        doesn't break the robot and it stays in the desired state.
        '''
        # Ensure things are ready to go.
        self.check_alive()

        # Test settings (to avoid code duplication)
        positions = [GRIPPER_OPEN_POSITION, GRIPPER_CLOSE_POSITION]
        right_commands = [Command.OPEN_RIGHT_HAND, Command.CLOSE_RIGHT_HAND]
        right_first_responses = [
            [RobotSpeech.RIGHT_HAND_OPENING], [RobotSpeech.RIGHT_HAND_CLOSING]]
        right_second_responses = [
            [RobotSpeech.RIGHT_HAND_ALREADY_OPEN],
            [RobotSpeech.RIGHT_HAND_ALREADY_CLOSED]
        ]

        left_commands = [Command.OPEN_LEFT_HAND, Command.CLOSE_LEFT_HAND]
        left_first_responses = [
            [RobotSpeech.LEFT_HAND_OPENING], [RobotSpeech.LEFT_HAND_CLOSING]]
        left_second_responses = [
            [RobotSpeech.LEFT_HAND_ALREADY_OPEN],
            [RobotSpeech.LEFT_HAND_ALREADY_CLOSED]
        ]

        # Start out in known state so we avoid multiple of response
        # options later on. This must be the opposite of the command
        # we start out with in the tests.
        self.cmd_assert_response(
            Command.CLOSE_RIGHT_HAND, [
                RobotSpeech.RIGHT_HAND_ALREADY_CLOSED,
                RobotSpeech.RIGHT_HAND_CLOSING
            ]
        )
        self.cmd_assert_response(
            Command.CLOSE_LEFT_HAND, [
                RobotSpeech.LEFT_HAND_ALREADY_CLOSED,
                RobotSpeech.LEFT_HAND_CLOSING
            ]
        )

        # Check each opening & closing, and do each twice.
        for state_idx in range(len(positions)):
            # First open/close once.
            self.cmd_assert_response(
                right_commands[state_idx], right_first_responses[state_idx])
            self.cmd_assert_response(
                left_commands[state_idx], left_first_responses[state_idx])
            for joint in [side + GRIPPER_JOINT_POSTFIX for side in SIDES]:
                self.assertJointCloseWithinTimeout(
                    joint,
                    positions[state_idx],
                    GRIPPER_EPSILON_POSITION,
                    GRIPPER_TOGGLE_TIME_SECONDS
                )

            # Then, do the same thing again.
            self.cmd_assert_response(
                right_commands[state_idx], right_second_responses[state_idx])
            self.cmd_assert_response(
                left_commands[state_idx], left_second_responses[state_idx])
            for joint in [side + GRIPPER_JOINT_POSTFIX for side in SIDES]:
                # Note that here we do "After" to ensure it doesn't just
                # pass immediately. But we don't wait the full time
                # (half is plenty to detect any change).
                self.assertJointCloseAfter(
                    joint,
                    positions[state_idx],
                    GRIPPER_EPSILON_POSITION,
                    GRIPPER_TOGGLE_TIME_SECONDS / 2.0
                )

        # Make sure nothing's crashed.
        self.check_alive()

    def test_action_and_step_navigation(self):
        '''Tests creating / switching between actions, and creating /
        switching between steps.

        We could expose APIs within the system to query its state after
        issuing these commands, but at this point we're just going to
        exercise them and make sure things haven't crashed.
        '''
        # Ensure things are ready to go.
        self.check_alive()

        # Create / switch between actions.
        self.cmd_assert_response(
            Command.CREATE_NEW_ACTION, [RobotSpeech.SKILL_CREATED])
        self.cmd_assert_response(
            Command.CREATE_NEW_ACTION, [RobotSpeech.SKILL_CREATED])
        self.cmd_assert_response(
            Command.PREV_ACTION, [RobotSpeech.SWITCH_SKILL])
        self.cmd_assert_response(
            Command.NEXT_ACTION, [RobotSpeech.SWITCH_SKILL])

        # Try deleting last / all poses when there are none.
        self.cmd_assert_response(
            Command.DELETE_LAST_STEP, [RobotSpeech.SKILL_EMPTY])
        self.cmd_assert_response(
            Command.DELETE_ALL_STEPS, [RobotSpeech.SKILL_EMPTY])

        # Make some steps.
        for i in range(4):
            self.cmd_assert_response(
                Command.SAVE_POSE, [RobotSpeech.STEP_RECORDED])

        # Switch between the steps. We don't track responses for these
        # because there's no speech response, only a GUI change.
        # NOTE(mbforbes): Step arguments are weird. It is the 1-based
        # indexed step, times 2, plus either 0 or 1 for right/left arm.
        for step_no in [9, 6, 3, 8]:
            self.gui_command_pub.publish(
                GuiCommand(GuiCommand.SELECT_ACTION_STEP, step_no))

        # Navigate away/to the action, switch to a step
        self.cmd_assert_response(
            Command.PREV_ACTION, [RobotSpeech.SWITCH_SKILL])
        self.cmd_assert_response(
            Command.NEXT_ACTION, [RobotSpeech.SWITCH_SKILL])
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 3))

        # Delete a step and try to switch to it
        self.cmd_assert_response(
            Command.DELETE_LAST_STEP, [RobotSpeech.LAST_POSE_DELETED])
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 8))

        # Delete all steps and try to switch to one
        self.cmd_assert_response(
            Command.DELETE_ALL_STEPS, [RobotSpeech.SKILL_CLEARED])
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 2))

        # Final switching (depending on test run order, may switch to
        # different actions than we created here; hence, this comes last
        # in the test so we don't muck with previously created steps.
        # Not that it matters... but nothing here should depend on the
        # existence or non-existence of steps).
        self.guicmd_assert_response(
            GuiCommand.SWITCH_TO_ACTION, 1, [RobotSpeech.SWITCH_SKILL])
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 2))
        self.guicmd_assert_response(
            GuiCommand.SWITCH_TO_ACTION, 2, [RobotSpeech.SWITCH_SKILL])

        # Make sure nothing's crashed.
        self.check_alive()

    def test_simple_execution(self):
        '''Extremely simple execution test: just save poses in place and
        execute. Merely testing lack of system crash.'''
        # Ensure things are ready to go.
        self.check_alive()

        # Make an action and one pose.
        self.cmd_assert_response(
            Command.CREATE_NEW_ACTION, [RobotSpeech.SKILL_CREATED])
        self.cmd_assert_response(
            Command.SAVE_POSE, [RobotSpeech.STEP_RECORDED])

        # Executing here should say "no" because there aren't enough
        # poses.
        self.cmd_assert_response(
            Command.EXECUTE_ACTION, [RobotSpeech.EXECUTION_ERROR_NOPOSES])

        # We'll also want to make sure the action finishes. We create
        # the previous count before executing so there's no race
        # condition.
        prev_finished = self.build_prev_resp_map([RobotSpeech.EXECUTION_ENDED])

        # Make an action and execute. It should work now.
        self.cmd_assert_response(
            Command.SAVE_POSE, [RobotSpeech.STEP_RECORDED])
        self.cmd_assert_response(
            Command.EXECUTE_ACTION, [RobotSpeech.START_EXECUTION])

        # Make sure we hear "execution ended"
        self.assert_response(prev_finished, EXECUTION_STEP_TIME * 2)

        # Make sure nothing's crashed.
        self.check_alive()

    def test_moving_execution(self):
        '''Test moving the arms a few times and executing.'''
        # Ensure things are ready to go.
        self.check_alive()

        self.cmd_assert_response(
            Command.CREATE_NEW_ACTION, [RobotSpeech.SKILL_CREATED])

        # Move arms to several different increments, saving each time.
        for portion in SIMPLE_EXECUTION_PORTIONS:
            self.move_arms_up(portion)
            self.cmd_assert_response(
                Command.SAVE_POSE, [RobotSpeech.STEP_RECORDED])

        # Move arms to bottom position.
        self.move_arms_up(0.0)

        # We'll track the execution ended response, but we get the
        # pre-count here to avoid a race condition.
        prev_stopped = self.build_prev_resp_map(
            [RobotSpeech.EXECUTION_ENDED])

        # Execute!
        self.cmd_assert_response(
            Command.EXECUTE_ACTION, [RobotSpeech.START_EXECUTION])

        # Make sure the arms get there by checking one of the joints.
        side = SIDES[0]
        joint_name = self.arm_control_joints[side][0]
        # We expect to get to the final postion for that side.
        expected_position = (
            SIDE_MULS[side] * ARM_OUT_PAN * SIMPLE_EXECUTION_PORTIONS[-1])
        # ... and we're willing to wait for all steps.
        wait_time = EXECUTION_STEP_TIME * len(SIMPLE_EXECUTION_PORTIONS)
        self.assertJointCloseWithinTimeout(
            joint_name, expected_position, ARM_EPSILON_POSITION, wait_time)

        # Make sure it says that it's finished successfully.
        self.assert_response(prev_stopped, EXECUTION_END_RESPONSE_TIMEOUT)

        # Make sure nothing's crashed.
        self.check_alive()

    def test_open_close_execution(self):
        '''Test moving the arms a few times and saving poses with open/close
        hand, then executing.'''
        # Ensure things are ready to go.
        self.check_alive()

        # Make sure grippers are open, THEN create new action.
        self.cmd_assert_response(
            Command.OPEN_RIGHT_HAND, [
                RobotSpeech.RIGHT_HAND_ALREADY_OPEN,
                RobotSpeech.RIGHT_HAND_OPENING
            ]
        )
        self.cmd_assert_response(
            Command.OPEN_LEFT_HAND, [
                RobotSpeech.LEFT_HAND_ALREADY_OPEN,
                RobotSpeech.LEFT_HAND_OPENING
            ]
        )
        self.cmd_assert_response(
            Command.CREATE_NEW_ACTION, [RobotSpeech.SKILL_CREATED])

        # Move arms to several different increments, opening / closing
        # each time to save two poses.
        last_toggle = 'open'
        gripper_cmds = {
            'open': [
                (Command.OPEN_RIGHT_HAND, [RobotSpeech.RIGHT_HAND_OPENING]),
                (Command.OPEN_LEFT_HAND, [RobotSpeech.LEFT_HAND_OPENING]),
            ], 'close': [
                (Command.CLOSE_RIGHT_HAND, [RobotSpeech.RIGHT_HAND_CLOSING]),
                (Command.CLOSE_LEFT_HAND, [RobotSpeech.LEFT_HAND_CLOSING])
            ]
        }
        gripper_positions = {
            'open': GRIPPER_OPEN_POSITION,
            'close': GRIPPER_CLOSE_POSITION
        }

        for portion in SIMPLE_EXECUTION_PORTIONS:
            # First move the arms.
            self.move_arms_up(portion)

            # Save two poses by opening/closing each hand.
            last_toggle = 'close' if last_toggle == 'open' else 'open'
            cmd_pairs = gripper_cmds[last_toggle]
            for cmd, resp_arr in cmd_pairs:
                self.cmd_assert_response(cmd, resp_arr)

            # We wait for each to finish (otherwise we start moving
            # before they're fully open/closed).
            for joint in [side + GRIPPER_JOINT_POSTFIX for side in SIDES]:
                self.assertJointCloseWithinTimeout(
                    joint,
                    gripper_positions[last_toggle],
                    GRIPPER_EPSILON_POSITION,
                    GRIPPER_TOGGLE_TIME_SECONDS
                )

        # Move arms to bottom position.
        self.move_arms_up(0.0)

        # We'll also want to make sure the action finishes. We create
        # the previous count before executing so there's no race
        # condition.
        prev_finished = self.build_prev_resp_map([RobotSpeech.EXECUTION_ENDED])

        # Execute!
        self.cmd_assert_response(
            Command.EXECUTE_ACTION, [RobotSpeech.START_EXECUTION])

        # Make sure the arms get there by checking one of the joints.
        side = SIDES[0]
        joint_name = self.arm_control_joints[side][0]
        expected_position = (
            SIDE_MULS[side] * ARM_OUT_PAN * SIMPLE_EXECUTION_PORTIONS[-1])
        # Multiply by two as we saved two poses per each move
        wait_time = EXECUTION_STEP_TIME * len(SIMPLE_EXECUTION_PORTIONS) * 2
        self.assertJointCloseWithinTimeout(
            joint_name, expected_position, ARM_EPSILON_POSITION, wait_time)

        # Also check grippers in desired state. Because the execution
        # involves opening and closing hands each time, they might still
        # need the full time to open/close here (as moving arms is much
        # faster).
        expected_gripper_position = (
            GRIPPER_OPEN_POSITION if last_toggle == 'open'
            else GRIPPER_CLOSE_POSITION)
        for joint in [side + GRIPPER_JOINT_POSTFIX for side in SIDES]:
            self.assertJointCloseWithinTimeout(
                joint,
                expected_gripper_position,
                GRIPPER_EPSILON_POSITION,
                GRIPPER_TOGGLE_TIME_SECONDS
            )

        # Make sure we hear "execution ended".
        self.assert_response(prev_finished)

        # Make sure nothing's crashed.
        self.check_alive()

    def test_trajectory(self):
        '''Test recording trajectory of moving the arms a few times and
        executing.'''
        # Ensure things are ready to go.
        self.check_alive()

        # Create action, move arms to bottom position & save pose.
        self.cmd_assert_response(
            Command.CREATE_NEW_ACTION, [RobotSpeech.SKILL_CREATED])
        self.move_arms_up(0.0)
        self.cmd_assert_response(
            Command.SAVE_POSE, [RobotSpeech.STEP_RECORDED])

        # Try telling it to stop recording before it's started.
        self.cmd_assert_response(
            Command.STOP_RECORDING_MOTION,
            [RobotSpeech.MOTION_NOT_RECORDING])

        # Start recording motion
        self.cmd_assert_response(
            Command.START_RECORDING_MOTION,
            [RobotSpeech.STARTED_RECORDING_MOTION])

        # Move arms to several different increments. Try saying 'start
        # recording motion' in the middle to ensure it handles this.
        for portion in SIMPLE_EXECUTION_PORTIONS:
            self.move_arms_up(portion)
            self.cmd_assert_response(
                Command.START_RECORDING_MOTION,
                [RobotSpeech.ALREADY_RECORDING_MOTION])

        # Stop recording motion.
        self.cmd_assert_response(
            Command.STOP_RECORDING_MOTION,
            [RobotSpeech.STOPPED_RECORDING_MOTION])

        # Move arms to bottom position (we have saved a pose there so
        # this isn't necessary from an execution standpoint. However,
        # from a test standpoint, we do this so we don't immediately
        # detect that the trajectory as completed.
        self.move_arms_up(0.0)

        # We'll track the execution ended response, but we get the
        # pre-count here to avoid a race condition.
        prev_stopped = self.build_prev_resp_map(
            [RobotSpeech.EXECUTION_ENDED])

        # Execute!
        self.cmd_assert_response(
            Command.EXECUTE_ACTION, [RobotSpeech.START_EXECUTION])

        # Make sure the arms get there by checking one of the joints.
        side = SIDES[0]
        joint_name = self.arm_control_joints[side][0]
        # We expect to get to the final postion for that side.
        expected_position = (
            SIDE_MULS[side] * ARM_OUT_PAN * SIMPLE_EXECUTION_PORTIONS[-1])
        # ... and we're willing to wait for all steps. This is a
        # traejctory, so it takes more time per "step."
        wait_time = TRAJ_STEP_TIME * len(SIMPLE_EXECUTION_PORTIONS)
        self.assertJointCloseWithinTimeout(
            joint_name, expected_position, ARM_EPSILON_POSITION, wait_time)

        # Make sure it says that it's finished successfully.
        self.assert_response(prev_stopped, EXECUTION_END_RESPONSE_TIMEOUT)

        # Make sure nothing's crashed.
        self.check_alive()

    def test_relative_objects(self):
        '''Tests saved poses / execution where object relativeness
        should happen.

        This should only pass, though will only exercise relativeness
        code if there is an object the PR2 can see.

        It doesn't test moving the object around, so in a sense this is
        'exercising' relativeness rather than testing it. Adding some
        Gazebo calls to do this would be pretty cool!
        '''
        # Ensure things are ready to go.
        self.check_alive()

        self.cmd_assert_response(
            Command.CREATE_NEW_ACTION, [RobotSpeech.SKILL_CREATED])

        # Move arms out of the way so we can see objects.
        self.move_arms_up(1.0)

        # Look for objects (hopefully we find them...).
        rospy.sleep(HEAD_SETTLING_WAIT_TIME)
        self.cmd_assert_response(
            Command.RECORD_OBJECT_POSE, [
                RobotSpeech.START_STATE_RECORDED,
                RobotSpeech.OBJECT_NOT_DETECTED,
            ],
            RECORD_OBJECT_POSE_TIMEOUT
        )

        # Move arms to bottom position to start exercising relativeness
        # code immediately.
        self.move_arms_up(0.0)

        # Move arms to several different increments, saving each time.
        for portion in SIMPLE_EXECUTION_PORTIONS:
            self.move_arms_up(portion)
            self.cmd_assert_response(
                Command.SAVE_POSE, [RobotSpeech.STEP_RECORDED])

        # Move arms out of the way so we can see objects.
        self.move_arms_up(1.0)

        # We'll track the execution ended response, but we get the
        # pre-count here to avoid a race condition.
        prev_stopped = self.build_prev_resp_map(
            [RobotSpeech.EXECUTION_ENDED])

        # Execute! (We first wait for the head to settle, and during
        # execution allocate extra time for it to find objects).
        rospy.sleep(HEAD_SETTLING_WAIT_TIME)
        self.cmd_assert_response(
            Command.EXECUTE_ACTION,
            [RobotSpeech.START_EXECUTION],
            DEFAULT_CMD_RESP_TIMEOUT + RECORD_OBJECT_POSE_TIMEOUT
        )

        # Make sure the arms get there by checking one of the joints.
        side = SIDES[0]
        joint_name = self.arm_control_joints[side][0]
        # We expect to get to the final postion for that side.
        expected_position = (
            SIDE_MULS[side] * ARM_OUT_PAN * SIMPLE_EXECUTION_PORTIONS[-1])
        # ... and we're willing to wait for all steps.
        wait_time = EXECUTION_STEP_TIME * len(SIMPLE_EXECUTION_PORTIONS)
        self.assertJointCloseWithinTimeout(
            joint_name, expected_position, ARM_EPSILON_POSITION, wait_time)

        # Make sure it says that it's finished successfully. Because the
        # above might have retunred immedaitely given that we have to
        # move the arms up to detect objects before the start of the
        # execution, we give it the wait time as well.
        self.assert_response(
            prev_stopped, wait_time + EXECUTION_END_RESPONSE_TIMEOUT)

        # Make sure nothing's crashed.
        self.check_alive()

    def test_relative_objects_trajectory(self):
        '''Test moving the arms a few times and executing.'''
        # Ensure things are ready to go.
        self.check_alive()

        # Create action, record object pose, move arms to bottom
        # position & save pose.
        self.cmd_assert_response(
            Command.CREATE_NEW_ACTION, [RobotSpeech.SKILL_CREATED])
        self.move_arms_up(1.0)
        rospy.sleep(HEAD_SETTLING_WAIT_TIME)
        self.cmd_assert_response(
            Command.RECORD_OBJECT_POSE, [
                RobotSpeech.START_STATE_RECORDED,
                RobotSpeech.OBJECT_NOT_DETECTED,
            ],
            RECORD_OBJECT_POSE_TIMEOUT
        )
        self.move_arms_up(0.0)
        self.cmd_assert_response(
            Command.SAVE_POSE, [RobotSpeech.STEP_RECORDED])

        # Start recording motion
        self.cmd_assert_response(
            Command.START_RECORDING_MOTION,
            [RobotSpeech.STARTED_RECORDING_MOTION])

        # Move arms to several different increments.
        for portion in SIMPLE_EXECUTION_PORTIONS:
            self.move_arms_up(portion)

        # Stop recording motion.
        self.cmd_assert_response(
            Command.STOP_RECORDING_MOTION,
            [RobotSpeech.STOPPED_RECORDING_MOTION])

        # Move arms out of way for object pose recording.
        self.move_arms_up(1.0)

        # We'll track the execution ended response, but we get the
        # pre-count here to avoid a race condition.
        prev_stopped = self.build_prev_resp_map(
            [RobotSpeech.EXECUTION_ENDED])

        # Execute! (Note head settle time and object record time.)
        rospy.sleep(HEAD_SETTLING_WAIT_TIME)
        self.cmd_assert_response(
            Command.EXECUTE_ACTION,
            [RobotSpeech.START_EXECUTION],
            DEFAULT_CMD_RESP_TIMEOUT + RECORD_OBJECT_POSE_TIMEOUT
        )

        # Make sure the arms get there by checking one of the joints.
        side = SIDES[0]
        joint_name = self.arm_control_joints[side][0]
        # We expect to get to the final postion for that side.
        expected_position = (
            SIDE_MULS[side] * ARM_OUT_PAN * SIMPLE_EXECUTION_PORTIONS[-1])
        # ... and we're willing to wait for all steps. This is a
        # traejctory, so it takes more time per "step".
        wait_time = TRAJ_STEP_TIME * len(SIMPLE_EXECUTION_PORTIONS)
        self.assertJointCloseWithinTimeout(
            joint_name, expected_position, ARM_EPSILON_POSITION, wait_time)

        # Make sure it says that it's finished successfully. Because we
        # had to start with the arms up, the above might have returned
        # immedaitely! Therefore, we give it the entire trajectory
        # duration as well to timeout.
        self.assert_response(
            prev_stopped, wait_time + EXECUTION_END_RESPONSE_TIMEOUT)

        # Make sure nothing's crashed.
        self.check_alive()

    # ##################################################################
    # Helper methods
    # ##################################################################

    def check_alive(self):
        '''Ensures the interaction node is alive.'''
        self.ping_srv()
        self.assertTrue(True, "Interaction node should be alive.")

    def speech_heard_cb(self, sound_request):
        '''Called when the robot requests speech; tracks responses.

        Args:
            sound_request (SoundRequest): Request robot sent for speech
                to be played.
        '''
        # We only track things the robot is going to say (text).
        if sound_request.command == SoundRequest.SAY:
            # Text is passed in arg.
            text = sound_request.arg

            # If the text has number-specific information (e.g. which
            # action), we trim this before saving.
            for base in RESPONSE_TRIMS:
                if base in text:
                    text = base
                    break

            # Increment count.
            self.speech_tracker[text] += 1

    def joint_states_cb(self, joint_state):
        '''Tracks relevant joint states.

        Args:
            joint_state (JointState): Sensor messages published by the
                PR2, reporting the state of its joints.
        '''
        names = joint_state.name
        positions = joint_state.position
        for joint in RELEVANT_JOINTS:
            if joint in names:
                self.joint_positions[joint] = positions[names.index(joint)]

    def guicmd_assert_response(
            self, guicommand, arg, responses,
            timeout=DEFAULT_CMD_RESP_TIMEOUT):
        '''Issues a GuiCommand with arg and asserts that one of the
        valid responses is heard within timout seconds.

        Args:
            guicommand (str): One of GuiCommand.*
            arg (int): An argument to pass along with guicommand.
            responses ([str]): Each element is one of RobotSpeech.*
            timeout (float): How many seconds to wait before failing.
        '''
        # Get number of times responses were heard previously.
        prev_resp_map = self.build_prev_resp_map(responses)

        # Issue the command
        self.gui_command_pub.publish(GuiCommand(guicommand, arg))

        # Assert a responses is heard within the timeout.
        self.assert_response(prev_resp_map, timeout)

    def cmd_assert_response(
            self, command, responses, timeout=DEFAULT_CMD_RESP_TIMEOUT):
        '''Issues a command and asserts that one of the valid responses
        is heard within timout seconds.

        Args:
            command (str): One of Command.*
            responses ([str]): Each element is one of RobotSpeech.*
            timeout (float): How many seconds to wait before failing.
        '''
        # Get number of times responses were heard previously.
        prev_resp_map = self.build_prev_resp_map(responses)

        # Issue the command
        self.command_pub.publish(Command(command))

        # Assert a responses is heard within the timeout.
        self.assert_response(prev_resp_map, timeout)

    def build_prev_resp_map(self, responses):
        '''Builds and returns a map of heard speech strings to the
        number of times they were heard.

        Args:
            responses ([str]): Each element is one of RobotSpeech.*

        Returns:
            {str: int}: Mapping from each of responses to counts that
                they were previously heard.
        '''
        # Note that we don't need to use a Counter here because
        # self.speech_tracker is already implemented using one.
        prev_resp_map = {}
        for response in responses:
            prev_resp_map[response] = self.speech_tracker[response]
        return prev_resp_map

    def assert_response(
            self, prev_resp_map, timeout=DEFAULT_CMD_RESP_TIMEOUT):
        '''Asserts that at least one of any responses in keys of
        prev_resp_map is heard within timeout seconds by comparing to
        previous counts stored as values in prev_resp_map.

        Args:
            prev_resp_map ({str: int}): Mapping from each of responses
                to counts that they were previously heard.
            timeout (float): How many seconds to wait before failing.
        '''
        # Check if response count increased once before waiting for
        # timeout.
        if self.any_resp_inc(prev_resp_map):
            return

        # Wait for timeout, checking if response heard.
        timeout_dur = rospy.Duration(timeout)
        start = rospy.Time.now()
        while rospy.Time.now() - start < timeout_dur:
            if self.any_resp_inc(prev_resp_map):
                return
            rospy.sleep(RESPONSE_REFRESH_PAUSE_SECONDS)

        # Check one last time before failing.
        if self.any_resp_inc(prev_resp_map):
            return

        # Not heard! Fail.
        self.assertFalse(
            True,
            "Never heard any of expected responses: " +
            str(prev_resp_map.keys())
        )

    def any_resp_inc(self, prev_resp_map):
        '''Returns whether any response in responses was seen exaclty
        once more than is recorded in prev_resp_map.

        Args:
            prev_resp_map ({str: int}): Map of previous responses to
                their counts.
        '''
        for response, prev_count in prev_resp_map.iteritems():
            # Must match exactly one greater than recorded previously.
            if self.speech_tracker[response] == prev_count + 1:
                return True
        # None match.
        return False

    def are_floats_close(self, a, b, epsilon):
        '''Checks whether two floats are within epsilon of each
        other.

        Args:
            a (float): One number.
            b (float): The other number.
            epsilon (float): Acceptable wiggle room (+/-) between a and
                b.

        Returns:
            bool: Whether a and b are within epsilon of each other.
        '''
        # We try to do this in an overflow-friendly way, though it
        # probably isn't a big deal with our use cases and python.
        return a - epsilon <= b if a > b else b - epsilon <= a

    def move_arms_up(self, portion=1.0):
        '''This method moves the arms 'up', meaning most seven arm
        joints get twisted by some amount until they're moderately 'up'.
        The portion is how much towards this 'up' position to go to,
        where 0.0 is arms in front, and 1.0 is arms fully up.

        Args:
            portion (float): 0.0 <= portion <= 1.0. How much of the way
                up to move the arms.
        '''
        # Safety check.
        portion = 0.0 if portion <= 0.0 else portion
        portion = 1.0 if portion >= 1.0 else portion

        for side in SIDES:
            joints = self.arm_control_joints[side]  # For convenience.
            # We have some specific configurations for the joints:
            # - The the main shoulder pan joint (first) should be
            #       mirrored (so that arms swing out)
            # - The shoulder lift joint (second) should always be
            #       reversed (so arms both go up)
            # - The upper arm roll joint (third) should be mirrored, as
            #       normal.
            # - The elbow flex joint should be unmirrored and set to
            #       raise up, always going a little below 0.0.
            # - The remaining joints are mirrored, as normal
            pan_el = [ARM_OUT_PAN * SIDE_MULS[side] * portion]
            lift_el = [ARM_UP_POSITION * -1.0 * portion]
            uproll_el = [ARM_UP_POSITION * SIDE_MULS[side] * portion]
            elflex_el = [ARM_UP_FLEX * (portion + 0.2)]
            other_els = (
                [ARM_UP_POSITION * SIDE_MULS[side] * portion] *
                (len(joints) - 4))
            positions = pan_el + lift_el + uproll_el + elflex_el + other_els
            velocities = [ARM_MOVE_VELOCITY] * len(joints)

            goal = JointTrajectoryGoal()
            goal.trajectory.joint_names = joints
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=positions, velocities=velocities))
            self.arm_controllers[side].send_goal(goal)
            self.arm_controllers[side].wait_for_result()

        # Let arms stop shaking around.
        rospy.sleep(ARM_MOVE_PAUSE)

    def assertJointCloseAfter(
            self, joint_name, expected_val, epsilon, time):
        '''Asserts that the position of the joint given by joint_name
        is "close to" expected_val after time seconds.

        Note that this is different than "assertJointCloseWithinTimeout"
        as this method waits to assert until after time. This is useful
        if, for instance, you want to check something remains close to
        a value after a period of time.

        (Note that this method is named in camelCase to match the other
        assert methods in the unittest library).

        Args:
            joint_name (str): Name of the joint to query.
            expected_val (float): Position joint must reach "close to".
            epsilon (float): Wiggle room (+/-) of joint reaching
                expected_val.
            time (float): How many seconds to wait before asserting the
                joint is close to the expected_val position.
        '''
        rospy.sleep(time)
        self.assertTrue(
            self.are_floats_close(
                self.joint_positions[joint_name], expected_val, epsilon),
            "Joint %s isn't at its expected value %f" %
            (joint_name, expected_val)
        )

    def assertJointCloseWithinTimeout(
            self, joint_name, expected_val, epsilon, timeout):
        '''Asserts that the position of the joint given by joint_name
        reaches "close to" expected_val within timeout seconds.

        (Note that this method is named in camelCase to match the other
        assert methods in the unittest library).

        Args:
            joint_name (str): Name of the joint to query.
            expected_val (float): Position joint must reach "close to".
            epsilon (float): Wiggle room (+/-) of joint reaching
                expected_val.
            timeout (float): How many seconds the joint has to reach
                close to the expected_val position.
        '''
        # We check this generously by checking once before and after the
        # timeout. (Note that we record the start, minimum, maximum,
        # and end values for debugging.)
        start_val = self.joint_positions[joint_name]
        min_ = start_val
        max_ = start_val
        if self.are_floats_close(start_val, expected_val, epsilon):
            return

        # Check / sleep through timeout
        timeout_dur = rospy.Duration(timeout)
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < timeout_dur:
            val = self.joint_positions[joint_name]
            min_ = val if val < min_ else min_
            max_ = val if val > max_ else max_
            if self.are_floats_close(val, expected_val, epsilon):
                return
            rospy.sleep(JOINT_REFRESH_PAUSE_SECONDS)

        # Check once after timeout.
        end = self.joint_positions[joint_name]
        min_ = end if end < min_ else min_
        max_ = end if end > max_ else max_
        if self.are_floats_close(end, expected_val, epsilon):
            return

        # Didn't make it; fail the test!
        self.assertFalse(
            True,
            ("Joint %s never reached its expected value %f. Values seen " +
             "were: start: %f, end: %f, min: %f, max: %f.") %
            (joint_name, expected_val, start_val, end, min_, max_)
        )

# ######################################################################
# Program execution begins here
# ######################################################################

if __name__ == '__main__':
    rospy.init_node('test_endtoend')
    import rostest
    rospy.loginfo("Running tests in simulation.")
    rospy.loginfo("Waiting for system to start up.")
    rospy.sleep(PAUSE_STARTUP)
    rospy.loginfo("Done waiting for system to start up.")
    rostest.rosrun(
        PKG,  # package_name
        'test_end_to_end',  # test_name
        TestEndToEnd,  # test_case_class
    )
