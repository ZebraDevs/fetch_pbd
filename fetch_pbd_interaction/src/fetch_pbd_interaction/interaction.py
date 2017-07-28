'''Main interaction event handler. Receives speech and GUI commands and
sends events out to the system.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# System builtins
import threading

# ROS builtins
from visualization_msgs.msg import MarkerArray
from interactive_markers.interactive_marker_server import \
     InteractiveMarkerServer
from tf import TransformListener

# Local
from fetch_arm_control.msg import GripperState
from fetch_pbd_interaction.session import Session
from fetch_pbd_interaction.msg import ExecutionStatus, Landmark
from fetch_pbd_interaction.srv import Ping, PingResponse, GetObjectList, \
                                      GuiInput, GuiInputRequest, \
                                      GuiInputResponse
from fetch_pbd_interaction.msg import RobotSound, WorldState
from fetch_pbd_interaction.robot import Robot
from std_srvs.srv import Empty

# ######################################################################
# Module level constants
# ######################################################################

BASE_LINK = 'base_link'
TOPIC_IM_SERVER = '/fetch_pbd/programmed_actions'


# ######################################################################
# Classes
# ######################################################################


class Interaction:
    '''Interaction is the multiplexer of commands received into system
    actions.

    Interaction receives GUI input (gui_input) and sends
    these off into the system to be
    processed. Interaction holds a small amount of state to support
    recording trajectories.
    '''

    def __init__(self, grasp_suggestion_service, 
                grasp_feedback_topic, external_ee_link,
                to_file, from_file, play_sound, social_gaze):

        # Create main components.
        self._tf_listener = TransformListener()
        self._robot = Robot(self._tf_listener, play_sound, social_gaze)
        self._im_server = InteractiveMarkerServer(TOPIC_IM_SERVER)
        self._session = Session(
                        self._robot, self._tf_listener,
                        self._im_server,
                        to_file=to_file, 
                        from_file=from_file,
                        grasp_suggestion_service_name=grasp_suggestion_service,
                        grasp_feedback_topic=grasp_feedback_topic,
                        external_ee_link=external_ee_link)
        self._head_busy = False

        # ROS publishers, subscribers, services
        self._viz_publisher = rospy.Publisher(
                                '/fetch_pbd/visualization_marker_array',
                                MarkerArray,
                                queue_size=10)

        # rospy.Subscriber('gui_input', GuiInput, self._gui_input_cb)
        rospy.Service('/fetch_pbd/gui_input', GuiInput, self._gui_input_cb)

        rospy.Subscriber('/fetch_pbd/world_update', WorldState, 
                            self._world_update_cb)

        # Initialize trajectory recording state.
        self._is_recording_motion = False

        # Command/callback pairs for input
        self._responses = {
            # Action Creation/Navigation
            GuiInputRequest.CREATE_ACTION: self._create_action,
            GuiInputRequest.SWITCH_TO_ACTION: self._switch_to_action,
            GuiInputRequest.NEXT_ACTION: self._next_action,
            GuiInputRequest.PREV_ACTION: self._previous_action,
            GuiInputRequest.UPDATE_ACTION_NAME: self._update_action_name,
            GuiInputRequest.DELETE_ACTION: self._delete_action,
            GuiInputRequest.COPY_ACTION: self._copy_action,
            # Primitive Creation Navigation
            GuiInputRequest.SWITCH_PRIMITIVE_ORDER: self._switch_primitive_order,
            GuiInputRequest.DELETE_PRIMITIVE: self._delete_primitive,
            GuiInputRequest.COPY_PRIMITIVE: self._copy_primitive,
            GuiInputRequest.SELECT_PRIMITIVE: self._select_primitive,
            GuiInputRequest.DELETE_ALL_PRIMITIVES: self._delete_all_primitives,
            GuiInputRequest.DELETE_LAST_PRIMITIVE: self._delete_last_primitive,
            GuiInputRequest.HIDE_PRIMITIVE_MARKER: self._hide_primitive_marker,
            GuiInputRequest.SHOW_PRIMITIVE_MARKER: self._show_primitive_marker,
            # Programming
            GuiInputRequest.OPEN_HAND: self._open_hand,
            GuiInputRequest.CLOSE_HAND: self._close_hand,
            GuiInputRequest.RECORD_OBJECTS: self._record_objects,
            GuiInputRequest.SAVE_TARGET: self._save_target,
            GuiInputRequest.START_RECORDING_TRAJECTORY: \
                                self._start_recording_trajectory,
            GuiInputRequest.STOP_RECORDING_TRAJECTORY: self._stop_recording_trajectory,
            GuiInputRequest.POSE_EDITED: self._primitive_pose_edited,
            # Execution
            GuiInputRequest.STOP_EXECUTION: self._stop_execution,
            GuiInputRequest.EXECUTE_ACTION: self._execute_action,
            GuiInputRequest.EXECUTE_PRIMITIVE: self._execute_primitive,
        }

        # The PbD backend is ready.
        # This basically exists for tests that aren't actually written yet
        rospy.loginfo('Interaction initialized.')
        self._ping_srv = rospy.Service('/fetch_pbd/interaction_ping', Ping,
                                       self._interaction_ping)

        rospy.Subscriber('/fetch_pbd/add_grasp', Landmark,
                      self._add_grasp)

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

        self._head_busy = self._head_busy or self._session.head_busy()

        if not arm_moving and not self._head_busy:
            # rospy.loginfo("Arm moving")
            self._robot.look_forward()
        else:
            if self._session.n_actions() > 0:
                current_action = self._session.get_current_action()
                if not current_action is None:
                    action_status = current_action.get_status()

                    if (action_status != ExecutionStatus.EXECUTING and
                            not self._head_busy):
                        self._robot.look_at_ee()
            elif not self._head_busy:
                self._robot.look_at_ee()

        # Update the current action if there is one.
        if self._session.n_actions() > 0:
            self._session.publish_primitive_tf()

            # Record trajectory primitive.
            if self._is_recording_motion:
                self._session.update_arm_trajectory()

            # If the objects in the world have changed, update the
            # action with them.
            current_action = self._session.get_current_action()
            if not current_action is None:
                action_status = current_action.get_status()
                current_action.update_viz()
                # if action_status != ExecutionStatus.NOT_EXECUTING:
                #     # self._arm_reset_publisher.publish(String(''))
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

    def _gui_input_cb(self, req):
        '''Callback for when input is received from GUI

        Args:
            req (GuiInputRequest): The input received from GUI
        '''
        # We extract the command string as we use it a lot.
        cmd = req.command
        if cmd in self._responses.keys():
            rospy.loginfo('\033[32m' + 'Calling response for command ' + cmd
                          + '\033[0m')
            response = self._responses[cmd]

            if not self._session.n_actions() > 0:
                threading.Thread(group=None,
                    target=response,
                    args=(req,),
                    name='gui_response_thread').start()

            elif (self._session.get_current_action() is None and
                    (cmd == GuiInputRequest.CREATE_ACTION or
                    cmd == GuiInputRequest.COPY_ACTION or
                    cmd == GuiInputRequest.DELETE_ACTION or
                    cmd == GuiInputRequest.SWITCH_TO_ACTION)):
                threading.Thread(group=None,
                    target=response,
                    args=(req,),
                    name='gui_response_thread').start()

            elif self._session.get_current_action() is None:
                rospy.logwarn("No current action")

            elif ((self._session.get_current_action().get_status() !=
                    ExecutionStatus.EXECUTING) or
                    cmd == GuiInputRequest.STOP_EXECUTION):
                threading.Thread(group=None,
                    target=response,
                    args=(req,),
                    name='gui_response_thread').start()
            else:
                rospy.logwarn(
                    'Ignoring speech command during execution: ' + cmd)
        else:
            rospy.logwarn('This command (' + cmd + ') is unknown.')

        return GuiInputResponse()

    def _create_action(self, gui_input):
        '''Creates a new empty action.

        Args:
            gui_input (GuiInputRequest) : unused
        '''
        self._session.new_action()
        self._robot.play_sound(RobotSound.CREATED_ACTION)
        self._robot.nod_head()

    def _switch_to_action(self, gui_input):
        '''Switches to an action that is already loaded in the session.

        The action is accessed by the index in the session's action list.
        The index is 0-based, so the first action is action 0.

        Args:
            gui_input (GuiInputRequest) : contains the index into the session's action
                                   list to switch to.
        '''
        # Command: switch to a specified action.
        success = self._session.switch_to_action_by_index(int(gui_input.param))
        if not success:
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()
        else:
            self._robot.play_sound(RobotSound.SUCCESS)
            self._robot.nod_head()

    def _next_action(self, gui_input):
        '''Switches to next action.

        Args:
            gui_input (GuiInputRequest) : unused
        '''
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

    def _previous_action(self, gui_input):
        '''Switches to previous action.

        Args:
            gui_input (GuiInputRequest) : unused
        '''
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

    def _update_action_name(self, gui_input):
        '''Update name of action.

        Args:
            gui_input (GuiInputRequest) : contains new name of action
        '''
        self._session.update_action_name(gui_input.param)

    def _delete_action(self, gui_input):
        '''Deletes action with certain index

        Args:
            gui_input (GuiInputRequest) : contains index of action to delete
        '''
        self._session.delete_action(int(gui_input.param))

    def _copy_action(self, gui_input):
        '''Copies action with certain index

        Args:
            gui_input (GuiInputRequest) : contains index of action to copy
        '''
        self._session.copy_action(int(gui_input.param))
        self._robot.play_sound(RobotSound.CREATED_ACTION)
        self._robot.nod_head()

    def _switch_primitive_order(self, gui_input):
        '''Changes the order of primitives

        Args:
            gui_input (GuiInputRequest) : contains the previous and current indices
                                   of the moved primitive
        '''
        old_index = gui_input.list_params[0]
        new_index = gui_input.list_params[1]
        self._session.switch_primitive_order(old_index, new_index)

    def _delete_primitive(self, gui_input):
        '''Deletes primitive with certain index from current action

        Args:
            gui_input (GuiInputRequest) : contains index of primitive to delete
        '''
        self._session.delete_primitive(int(gui_input.param))

    def _copy_primitive(self, gui_input):
        '''Copies primitive with certain index from current action

        Args:
            gui_input (GuiInputRequest) : contains index of primitive to copy
        '''
        self._session.copy_primitive(int(gui_input.param))

    def _select_primitive(self, gui_input):
        '''Selects a primitive in the current action.

        Args:
            gui_input (GuiInputRequest) : contains index of primitive to select
        '''
        primitive_number = int(gui_input.param)
        self._session.select_action_primitive(primitive_number)

    def _delete_all_primitives(self, gui_input):
        '''Deletes all primitives in the current action.

        Args:
            gui_input (GuiInputRequest) : unused
        '''
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

    def _delete_last_primitive(self, gui_input):
        '''Deletes last primitive of the current action.

        Args:
            gui_input (GuiInputRequest) : unused
        '''
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

    def _hide_primitive_marker(self, gui_input):
        '''Hide marker with certain index

        Args:
            gui_input (GuiInputRequest) : contains index of marker to hide
        '''
        self._session.hide_primitive_marker(int(gui_input.param))

    def _show_primitive_marker(self, gui_input):
        '''Show marker with certain index

        Args:
            gui_input (GuiInputRequest) : contains index of marker to show
        '''
        self._session.show_primitive_marker(int(gui_input.param))

    def _open_hand(self, gui_input):
        '''Opens gripper

        Args:
            gui_input (GuiInputRequest) : unused
        '''
        self._head_busy = True
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
        self._head_busy = False

    def _close_hand(self, gui_input):
        '''Closes gripper

        Args:
            gui_input (GuiInputRequest) : unused
        '''
        # First, close the hand if it's open.
        self._head_busy = True
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
        self._head_busy = False

    def _record_objects(self, gui_input):
        '''Makes the robot look for a table and objects.

        Args:
            gui_input (GuiInputRequest) : unused
        '''
        self._head_busy = True
        if self._session.record_objects():
            self._robot.play_sound(RobotSound.SUCCESS)
            self._robot.nod_head()
        else:
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()
        self._head_busy = False

    def _save_target(self, gui_input):
        '''Saves current arm state as an action primitive (ArmTarget).

        Args:
            gui_input (GuiInputRequest) : unused
        '''
        self._head_busy = True
        if self._session.n_actions() > 0:
            self._session.add_arm_target_to_action()
            self._robot.play_sound(RobotSound.POSE_SAVED)
            self._robot.nod_head()
        else:
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()
        self._head_busy = False

    def _add_grasp(self, msg):
        '''Callback to add a grasp for the specified object to 
        the current action

        Args:
            msg (Landmark)
        '''
        rospy.loginfo("Attempting to add grasp")
        self._head_busy = True
        if self._session.n_actions() > 0:
            self._session.add_grasp_to_action(msg)
            self._robot.play_sound(RobotSound.POSE_SAVED)
            self._robot.nod_head()
        else:
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()
        self._head_busy = False

    def _start_recording_trajectory(self, gui_input):
        '''Starts recording continuous motion.

        Args:
            gui_input (GuiInputRequest) : unused
        '''
        self._head_busy = True
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
        self._head_busy = False

    def _stop_recording_trajectory(self, gui_input):
        '''Stops recording continuous motion.

        Args:
            gui_input (GuiInputRequest) : unused
        '''
        self._head_busy = True
        if self._is_recording_motion:
            self._is_recording_motion = False

            self._session.stop_recording_arm_trajectory()
            self._robot.play_sound(RobotSound.POSE_SAVED)
            self._robot.nod_head()

        else:
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()
        self._head_busy = False

    def _primitive_pose_edited(self, gui_input):
        '''Updates the primitive pose with input from the interface

        Args:
            gui_input (GuiInputRequest) : contains pose information
        '''
        primitive_number = int(gui_input.param)
        self._session.update_primitive_pose(primitive_number,
                                            gui_input.position,
                                            gui_input.orientation)

    def _execute_action(self, gui_input):
        '''Starts the execution of the current action.

        Args:
            gui_input (GuiInputRequest) : unused
        '''
        # We must have a current action.
        self._head_busy = True
        self._robot.play_sound(RobotSound.STARTING_EXECUTION)
        if self._session.execute_current_action():
            self._robot.play_sound(RobotSound.SUCCESS)
            rospy.loginfo("Action succeeded")
            self._robot.nod_head()
        else:
            self._robot.play_sound(RobotSound.ERROR)
            rospy.logwarn("Action failed")
            self._robot.shake_head()
        self._head_busy = False

    def _stop_execution(self, gui_input):
        '''Stops ongoing execution after current primitive is finished

        Args:
            gui_input (GuiInputRequest) : unused
        '''
        self._head_busy = True
        status = self._session.get_current_action().get_status()
        if status == ExecutionStatus.EXECUTING:
            self._session.get_current_action().stop_execution()
            self._robot.play_sound(RobotSound.OTHER)
            # self._robot.nod_head()
        else:
            self._robot.play_sound(RobotSound.ERROR)
            self._robot.shake_head()
        self._head_busy = False

    def _execute_primitive(self, gui_input):
        '''Execute primitive with certain index

        Args:
            gui_input (GuiInputRequest) : contains index of primitive to execute
        '''
        self._head_busy = True
        self._session.execute_primitive(int(gui_input.param))
        self._head_busy = False
