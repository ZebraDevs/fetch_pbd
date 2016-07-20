'''Main interaction event handler. Receives speech and GUI commands and
sends events out to the system.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# ROS builtins
from visualization_msgs.msg import MarkerArray
from interactive_markers.interactive_marker_server import \
     InteractiveMarkerServer
from tf import TransformListener

# Local
from fetch_arm_control.msg import GripperState
from fetch_pbd_interaction.session import Session
from fetch_pbd_interaction.msg import ExecutionStatus, GuiCommand
from fetch_pbd_interaction.srv import Ping, PingResponse, GetObjectList
from fetch_pbd_speech_recognition.msg import Command
from fetch_pbd_interaction.msg import RobotSound, WorldState
from fetch_pbd_interaction.robot import Robot
from std_msgs.msg import String
from std_srvs.srv import Empty

# ######################################################################
# Module level constants
# ######################################################################

EXECUTION_Z_OFFSET = -0.00
BASE_LINK = 'base_link'
TOPIC_IM_SERVER = 'programmed_actions'


# ######################################################################
# Classes
# ######################################################################


class Interaction:
    '''Interaction is the multiplexer of commands received into system
    actions.

    Interaction receives speech commands (recognized_command) as well as
    GUI commands (gui_command) and sends these off into the system to be
    processed. Interaction holds a small amount of state to support
    recording trajectories.
    '''

    def __init__(self):

        # Create main components.
        self._tf_listener = TransformListener()
        self._robot = Robot(self._tf_listener)
        self._im_server = InteractiveMarkerServer(TOPIC_IM_SERVER)
        self._session = Session(self._robot, self._tf_listener,
                                self._im_server)

        # ROS publishers, subscribers, services
        self._viz_publisher = rospy.Publisher('visualization_marker_array',
                                              MarkerArray,
                                              queue_size=10)

        rospy.Subscriber('recognized_command', Command,
                         self._speech_command_cb)
        rospy.Subscriber('gui_command', GuiCommand, self._gui_command_cb)

        rospy.Subscriber('world_update', WorldState, self._world_update_cb)

        self._clear_world_objects_srv = \
                        rospy.ServiceProxy('clear_world_objects', Empty)
        self._update_world_srv = rospy.ServiceProxy('update_world',
                                                    GetObjectList)
        rospy.wait_for_service('clear_world_objects')
        rospy.wait_for_service('update_world')

        # Initialize trajectory recording state.
        self._is_recording_motion = False

        # Keep track of head state
        # TODO(sarah): Is this necessary or does the fact that the
        # "LOOK_DOWN" action is not interruptable cover this?
        self._looking_down = False

        # Command/callback pairs for input
        self._responses = {
            Command.TEST_MICROPHONE: self._test_microphone,
            Command.OPEN_HAND: self._open_hand,
            Command.CLOSE_HAND: self._close_hand,
            Command.STOP_EXECUTION: self._stop_execution,
            Command.DELETE_ALL_STEPS: self._delete_all_primitives,
            Command.DELETE_LAST_STEP: self._delete_last_primitive,
            Command.CREATE_NEW_ACTION: self._create_action,
            Command.EXECUTE_ACTION: self._execute_action,
            Command.NEXT_ACTION: self._next_action,
            Command.PREV_ACTION: self._previous_action,
            Command.SAVE_POSE: self._save_primitive,
            Command.RECORD_OBJECT_POSE: self._record_object_pose,
            Command.START_RECORDING_MOTION: self._start_recording,
            Command.STOP_RECORDING_MOTION: self._stop_recording
        }

        # Span off a thread to run the update loops.
        # threading.Thread(group=None,
        #                  target=self.update,
        #                  name='interaction_update_thread').start()


        # The PbD backend is ready.
        # This basically exists for tests that aren't actually written yet
        rospy.loginfo('Interaction initialized.')
        self._ping_srv = rospy.Service('interaction_ping', Ping,
                                       self._interaction_ping)

        # Make sure gravity compensation controllers are on before we start
        self._robot.relax_arm()

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def update(self):
        '''General update for the main loop.

        This is called continuously from the node.
        This pauses for 100ms at the end of every
        run before returning.
        '''

        arm_moving = self._robot.is_arm_moving()

        if not arm_moving and not self._looking_down:
            rospy.loginfo("Arm moving")
            self._robot.look_forward()
        else:
            if self._session.n_actions() > 0:
                current_action = self._session.get_current_action()

                action_status = current_action.get_status()

                if (action_status != ExecutionStatus.EXECUTING and
                        not self._looking_down):
                    self._robot.look_at_ee()
            elif not self._looking_down:
                self._robot.look_at_ee()

        # Update the current action if there is one.
        if self._session.n_actions() > 0:

            # Record trajectory primitive.
            if self._is_recording_motion:
                self._session.update_arm_trajectory()

            # If the objects in the world have changed, update the
            # action with them.
            current_action = self._session.get_current_action()

            action_status = current_action.get_status()

            # if action_status != ExecutionStatus.NOT_EXECUTING:
            #     self._arm_reset_publisher.publish(String(''))
            #     if action_status != ExecutionStatus.EXECUTING:
            #         self._end_execution()


    # ##################################################################
    # Internal ("private" methods)
    # ##################################################################


    def _world_update_cb(self, msg):
        ''' Respond to changes in world
        Right now these changes are mostly initiated by
        this class anyway but in case something else changes
        we want to keep track

        Args:
            msg (WorldState)
        '''
        current_action = self._session.get_current_action()
        if not current_action is None:
            current_action.update_objects()

    def _end_execution(self):
        '''Says a response and performs a gaze action for when an action
        execution ends.'''
        status = self._session.get_current_action().get_status()
        rospy.loginfo("Execution ended. Status: " + str(status))
        if status == ExecutionStatus.SUCCEEDED:
            # Execution completed successfully.
            self._robot.play_sound(RobotSound.EXECUTION_ENDED)
            self._robot.nod_head()
        elif status == ExecutionStatus.PREEMPTED:
            # Execution stopped early (preempted).
            # self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()
        else:
            # Couldn't solve for joint positions (IK).
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()

        # self._robot.relax_arm()
        self._session.get_current_action().end_execution()


    def _interaction_ping(self, req):
        '''This is the service that is provided so that external nodes
        know the interaction (i.e. PbD) is ready. This basically exists
        for tests that haven't been written yet for Fetch.

        Args:
            req (PingRequest): unused

        Returns:
            PingResponse: empty
        '''
        return PingResponse()


    # The following methods receive commands from speech / GUI and
    # process them. These are the multiplexers.

    def _speech_command_cb(self, command):
        '''Callback for when a "speech" command is received.

        Note that Commands can actually be received from speech OR the
        GUI. They are called "speech commands" because they include all
        commands that can be issued with speech. The few commands that
        can be issued through the GUI and not through speech come as
        GUICommands, and are processed below in _gui_command_cb(...).

        Args:
            command (Command): The command received from speech OR the
                GUI.
        '''
        # We extract the command string as we use it a lot.
        cmd = command.command
        if cmd in self._responses.keys():
            rospy.loginfo('\033[32m' + 'Calling response for command ' + cmd
                          + '\033[0m')
            response = self._responses[cmd]

            if not self._session.n_actions() > 0:
                response()
            elif ((self._session.get_current_action().get_status() !=
                    ExecutionStatus.EXECUTING) or cmd == Command.STOP_EXECUTION):
                response()
            else:
                rospy.logwarn(
                    'Ignoring speech command during execution: ' + cmd)
        else:
            rospy.logwarn('This command (' + cmd + ') is unknown.')

    def _gui_command_cb(self, command):
        '''Callback for when a GUICommand is received.

        Note that a GUICommand is not any command received from the GUI;
        it is specifically a command that is only possible from the GUI.
        Commands sent from the GUI that are also possible via speech are
        sent via Command messages and handled in the
        _speech_command_cb(...) function above.

        Args:
            command (GUICommand): The command received from the GUI.
        '''
        # We extract the command string as we use it a lot.
        cmd = command.command

        # Because the GUI commands involve selecting actions or primitives
        # within actions, we have two prerequisites: first, we cannot be
        # currently executing an action, and second, we must have at
        # least one action.
        is_executing = False
        if self._session.n_actions() > 0:
            if (self._session.get_current_action().get_status() ==
                ExecutionStatus.EXECUTING):
                is_executing = True

        if not is_executing:
            if cmd == GuiCommand.SWITCH_TO_ACTION:
                index = int(command.param)
                self._switch_to_action(index)

            elif cmd == GuiCommand.SELECT_ACTION_STEP:
                # Command: select a primitive in the current action.
                primitive_number = int(command.param)
                self._select_action_primitive(primitive_number)
            else:
                # Command: unknown. (Currently impossible.)
                rospy.logwarn('This command (' + cmd + ') is unknown.')
        else:
            # Currently executing; ignore command.
            rospy.logwarn('Ignoring GUI command during execution: ' + cmd)

    def _switch_to_action(self, index):
        '''Switches to an action that is already loaded in the session.

        The action is accessed by the index in the session's action list.
        The index is 0-based, so the first action is action 0.

        Args:
            index: int, the index into the session's action list to switch to.
        '''
        # Command: switch to a specified action.
        success = self._session.switch_to_action(
            index)
        if not success:
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()
        else:
            self._robot.play_sound(RobotSound.SUCCESS)
            self._robot.nod_head()

    def _select_action_primitive(self, primitive_number):
        '''Selects a primitive in the current action.

        Args:
            primitive_number: int, the index in the list of primitives for the current
            action.
        '''
        self._session.select_action_primitive(primitive_number)
        rospy.loginfo('Selected action primitive ' + str(primitive_number))

    # The following methods are selected from commands (either GUI or
    # speech) and then called from within a Response objects's
    # respond(...) function. They follow the same pattern of their
    # accepted and returned values.

    def _open_hand(self):
        '''Opens gripper'''
        # First, open the hand if it's closed.
        if self._robot.get_gripper_state() != GripperState.OPEN:
            # Hand was closed, now open.
            self._robot.set_gripper_state(GripperState.OPEN)
            if self._session.n_actions() > 0:
                # If we're currently programming, save that as a primitive
                self._session.add_arm_target_to_action()
                self._robot.play_sound(RobotSound.POSE_SAVED)

            self._robot.play_sound(RobotSound.OTHER)
            self._robot.look_at_ee(follow=False)

        else:
            # Hand was already open; complain.
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.look_at_ee(follow=False)


    def _close_hand(self):
        '''Closes gripper'''
        # First, close the hand if it's open.
        if self._robot.get_gripper_state() != GripperState.CLOSED:
            self._robot.set_gripper_state(GripperState.CLOSED)
            # Hand was open, now closed.
            if self._session.n_actions() > 0:
                # If we're currently programming, save that as a primitive.
                self._session.add_arm_target_to_action()
                self._robot.play_sound(RobotSound.POSE_SAVED)

            self._robot.play_sound(RobotSound.OTHER)
            self._robot.look_at_ee(follow=False)
        else:
            # Hand was already closed; complain.
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.look_at_ee(follow=False)

    def _create_action(self):
        '''Creates a new empty action.'''
        self._session.new_action()
        self._clear_world_objects_srv()
        self._robot.play_sound(RobotSound.CREATED_ACTION)
        self._robot.nod_head()

    def _next_action(self):
        '''Switches to next action.'''
        if self._session.n_actions() > 0:
            if self._session.next_action():
                self._robot.play_sound(RobotSound.SUCCESS)
                self._robot.nod_head()
            else:
                self._robot.play_sound(RobotSound.ERROR)
                self._robot.shake_head()
        else:
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()

    def _previous_action(self):
        '''Switches to previous action.'''
        if self._session.n_actions() > 0:
            if self._session.previous_action():
                self._robot.play_sound(RobotSound.SUCCESS)
                self._robot.nod_head()
            else:
                self._robot.play_sound(RobotSound.ERROR)
                self._robot.shake_head()
        else:
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()

    def _delete_last_primitive(self):
        '''Deletes last primitive of the current action.'''
        if self._session.n_actions() > 0:
            if self._session.n_primitives() > 0:
                self._session.delete_last_primitive()
                self._robot.play_sound(RobotSound.OTHER)
                self._robot.nod_head()
            else:
                self._robot.play_sound(RobotSound.ERROR)
                self._robot.shake_head()
        else:
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()

    def _delete_all_primitives(self):
        '''Deletes all primitives in the current action.'''
        if self._session.n_actions() > 0:
            if self._session.n_primitives() > 0:
                self._session.clear_current_action()
                self._robot.play_sound(RobotSound.ALL_POSES_DELETED)
                self._robot.nod_head()
            else:
                self._robot.play_sound(RobotSound.ERROR)
                self._robot.shake_head()
        else:
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()

    def _stop_execution(self):
        '''Stops ongoing execution after current primitive is finished'''
        status = self._session.get_current_action().get_status()
        if status == ExecutionStatus.EXECUTING:
            self._session.get_current_action().stop_execution()
            self._robot.play_sound(RobotSound.OTHER)
            # self._robot.nod_head()
        else:
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()

    def _start_recording(self):
        '''Starts recording continuous motion.'''
        if self._session.n_actions() > 0:
            if not self._is_recording_motion:
                self._is_recording_motion = True
                self._session.start_recording_arm_trajectory()
                self._robot.play_sound(RobotSound.START_TRAJECTORY)
                self._robot.nod_head()
            else:
                self._robot.play_sound(RobotSound.ERROR)
                self._robot.shake_head()
        else:
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()

    def _stop_recording(self):
        '''Stops recording continuous motion.'''
        if self._is_recording_motion:
            self._is_recording_motion = False

            self._session.stop_recording_arm_trajectory()
            self._robot.play_sound(RobotSound.POSE_SAVED)
            self._robot.nod_head()

        else:
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()

    def _save_primitive(self):
        '''Saves current arm state as an action primitive.'''
        if self._session.n_actions() > 0:
            self._session.add_arm_target_to_action()
            self._robot.play_sound(RobotSound.POSE_SAVED)
            self._robot.nod_head()
        else:
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()

    def _record_object_pose(self):
        '''Makes the robot look for a table and objects.'''
        self._looking_down = True
        self._robot.look_down()
        resp = self._update_world_srv()
        if resp.object_list:
            if self._session.n_actions() > 0:
                self._session.get_current_action().update_objects()
            self._robot.play_sound(RobotSound.SUCCESS)
            self._robot.nod_head()
        else:
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()

        self._looking_down = False

    def _test_microphone(self):
        '''Makes sound to confirm that microphone is working.'''

        self._robot.play_sound(RobotSound.MICROPHONE_WORKING)

    def _execute_action(self):
        '''Starts the execution of the current action.
        TODO(sarah): Currently requires > 1 primitive in order to execute.
                     Should be an easy fix, but not sure if there are
                     repercussions elsewhere.
        '''
        # We must have a current action.
        if self._session.n_actions() > 0:
            # We must have also recorded primitives (/poses/frames) in it.
            if self._session.n_primitives() > 1:
                # Save curent action and retrieve it.
                # self._session.save_current_action()

                # Now, see if we can execute.
                if self._session.get_current_action().is_object_required():
                    # We need an object; check if we have one.
                    self._robot.look_down()
                    resp = self._update_world_srv()
                    # objects = resp.objects
                    if resp.object_list:
                        # An object is required, and we got one. Execute.
                        self._session.get_current_action().update_objects()
                        self._session.get_current_action().start_execution(
                            EXECUTION_Z_OFFSET)
                    else:
                        # An object is required, but we didn't get it.
                        self._robot.play_sound(RobotSound.ERROR)
                        self._robot.shake_head()
                else:
                    # No object is required: start execution now.
                    self._session.get_current_action().start_execution(
                            EXECUTION_Z_OFFSET)

                # Reply: starting execution.
                self._robot.play_sound(RobotSound.STARTING_EXECUTION)
            else:
                # No primitives / poses / frames recorded.
                rospy.loginfo("No primitives recorded")
                self._robot.play_sound(RobotSound.ERROR)
                self._robot.shake_head()
        else:
            # No actions.
            rospy.loginfo("No actions recorded")

            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()



