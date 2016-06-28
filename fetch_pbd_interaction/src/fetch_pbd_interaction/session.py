'''Everything related to the state of an experiment session.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import roslib
roslib.load_manifest('fetch_pbd_interaction')
import rospy

# System builtins
import datetime
import os
import threading
import yaml

# Local
from programmed_action import ProgrammedAction
from fetch_pbd_interaction.msg import ExperimentState
from fetch_pbd_interaction.srv import (GetExperimentState,
                                     GetExperimentStateResponse)

# ######################################################################
# Module level constants
# ######################################################################

# Constants regarding saving / loading experiment state to / from disk.
EXPERIMENT_DIR_PREFIX = '/data/experiment'
SAVE_FILENAME = 'experimentState.yaml'
YAML_KEY_NACTIONS = 'nProgrammedActions'
YAML_KEY_ACTION_LIST = 'actionList'
YAML_KEY_CURIDX = 'currentProgrammedActionIndex'

# ROS params, topics, services, etc.
PARAM_DATA_ROOT = '/fetch_pbd_interaction/dataRoot'
PARAM_EXP_NO = '/fetch_pbd_interaction/experimentNumber'
PARAM_IS_RELOAD = '/fetch_pbd_interaction/isReload'
PARAM_DATA_DIR = 'data_directory'

# ######################################################################
# Classes
# ######################################################################


class Session:
    '''This class holds and maintains experimental data.'''

    def __init__(self, object_list, db):
        '''
        Args:
            object_list ([Landmark]): List of Landmark (as defined by
                Landmark.msg), the current reference frames.
            db: An ActionDatabase object (db.py)
        '''
        # Private attirbutes.
        self._is_reload = rospy.get_param(PARAM_IS_RELOAD)
        self._exp_number = None
        self._selected_step = 0
        self._object_list = object_list
        self._db = db  # ActionDatabase handle

        # An in-memory, ordered list of action IDs we have created or loaded.
        # This is temporary and is lost when the program ends.
        # Used mainly for implementing previous/next actions.
        self._session_actions = []

        # Get the data directory by determining where to save via the
        # experiment number. The experiment number is loaded from a ROS
        # param (see launch file to alter; can also set via command line
        # specification).
        self._exp_number = rospy.get_param(PARAM_EXP_NO)
        self._data_dir = self._get_data_dir(self._exp_number)
        if not os.path.exists(self._data_dir):
            os.makedirs(self._data_dir)
        rospy.set_param(PARAM_DATA_DIR, self._data_dir)

        # Public attributes.
        self.actions = {}
        self.current_action_id = None

        # Reload the experiment state if specified.
        if self._is_reload:
            self._load_session_state(object_list)
            rospy.loginfo("Session state loaded.")

        # Create state publisher to broadcast state as well as service
        # to query it.
        self._state_publisher = rospy.Publisher('experiment_state',
                                                ExperimentState)
        rospy.Service('get_experiment_state', GetExperimentState,
                      self._get_experiment_state_cb)

        # Send initial broadcast of experiment state.
        self._update_experiment_state()

    # ##################################################################
    # Static methods: Internal ("private")
    # ##################################################################

    @staticmethod
    def _get_data_dir(exp_number):
        '''Returns the directory where action information is saved.

        Args:
            exp_number (int): The experiment number to use in
                determining the directory.
        '''
        return (rospy.get_param(PARAM_DATA_ROOT) + EXPERIMENT_DIR_PREFIX +
                str(exp_number) + '/')

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def select_action_step(self, step_id):
        ''' Makes the interactive marker for the indicated action step
        selected by showing the 6D controls.

        Args:
            step_id (int): ID of the step to select.
        '''
        self.actions[self.current_action_id].select_step(step_id)
        self._selected_step = step_id

    def save_session_state(self, is_save_actions=True):
        '''Saves the session state onto the disk.

        Args:
            is_save_actions (bool): Whether to have the actions
                themselves also write out to disk (as ROS bags).
        '''
        # Always save the light, high-level state, like number of
        # actions and current action index.
        exp_state = {}
        exp_state[YAML_KEY_NACTIONS] = self.n_actions()
        exp_state[YAML_KEY_ACTION_LIST] = self._session_actions
        exp_state[YAML_KEY_CURIDX] = self.current_action_id
        with open(self._data_dir + SAVE_FILENAME, 'w') as state_file:
            state_file.write(yaml.dump(exp_state))

        # If we're told to, have each action write its data out into a
        # ROS bag.
        if is_save_actions:
            for i in self.actions.keys():
                self.actions[i].save(self._data_dir)

    def new_action(self):
        '''Creates new action.'''
        if self.n_actions() > 0:
            self.get_current_action().reset_viz()
        time_str = datetime.datetime.now().strftime('%c')
        name = 'Untitled action {}'.format(time_str)
        action_id = self._db.insert_new(name)
        self.current_action_id = action_id
        action = ProgrammedAction(self.current_action_id, self._selected_step_cb)
        action.name = name
        self.actions.update({
            self.current_action_id: action
        })
        self._session_actions.append(action_id)
        self._update_experiment_state()

    def n_actions(self):
        '''Returns the number of actions programmed so far.

        Returns:
            int
        '''
        return len(self.actions)

    def get_current_action(self):
        '''Returns the current action.

        Returns:
            ProgrammedAction
        '''
        return self.actions[self.current_action_id]

    def clear_current_action(self):
        '''Removes all steps in the current action.'''
        if self.n_actions() > 0:
            current_action = self.actions[self.current_action_id]
            current_action.clear()
            self._db.update(self.current_action_id, current_action.to_msg())
        else:
            rospy.logwarn("Can't clear action: No actions created yet.")
        self._update_experiment_state()

    def save_current_action(self):
        '''Saves the current action onto disk.
        This does not save to the database because actions are automatically saved to the database.
        '''
        if self.n_actions() > 0:
            self.actions[self.current_action_id].save(self._data_dir)
            self.save_session_state(is_save_actions=False)
        else:
            rospy.logwarn("Can't save action: No actions created yet.")

    def add_step_to_action(self, step, object_list):
        '''Add a new step to the current action.

        Regardless of whether it succeeds in doing so, the object_list
        is now used.

        Args:
            step (ActionStep): The new step to add.
            object_list ([Landmark]): List of Landmark (as defined by
                Landmark.msg), the current reference frames.
        '''
        if self.n_actions() > 0:
            current_action = self.actions[self.current_action_id]
            current_action.add_action_step(step, object_list)
            self._db.update(self.current_action_id, current_action.to_msg())
        else:
            rospy.logwarn("Can't add step: No actions created yet.")
        self._object_list = object_list
        self._update_experiment_state()

    def delete_last_step(self):
        '''Removes the last step of the action.'''
        if self.n_actions() > 0:
            current_action = self.actions[self.current_action_id]
            current_action.delete_last_step()
            self._db.update(self.current_action_id, current_action.to_msg())
        else:
            rospy.logwarn("Can't delete last step: No actions created yet.")
        self._update_experiment_state()

    def load_action(self, db_id):
        '''Loads an action from the database.
        This effectively creates an action in the current session.

        Args:
            db_id: string, the ID of the action to load from the database.

        Returns:
            True on success, False otherwise.
        '''
        if self.n_actions() > 0:
            self.get_current_action().reset_viz()

        self.current_action_id = db_id
        if db_id not in self.actions:
            msg = self._db.find(db_id)
            if msg is None:
                return False
            self.actions.update({
                self.current_action_id: ProgrammedAction.from_msg(
                    msg, self.current_action_id, self._selected_step_cb)
            })
            self._session_actions.append(db_id)
        self._update_experiment_state()
        return True

    def switch_to_action_by_index(self, index, object_list):
        '''Switch to an action by the index in the current list of session actions.

        Args:
            index: The index in the current list of actions to switch to.

        Returns:
            bool, True on success or False otherwise.
        '''
        if index < 0 or index >= len(self._session_actions):
            return False
        action_id = self._session_actions[index]
        return self.switch_to_action(action_id, object_list)

    def switch_to_action(self, action_id, object_list):
        '''Switches to action_id action.

        Regardless of whether it succeeds in doing so, the object_list
        is now used.

        Args:
            action_id (string): The action ID in the database to switch to.
            object_list ([Landmark]): List of Landmark (as defined by
                Landmark.msg), the current reference frames.

        Returns:
            bool: Whether successfully switched to action_id action.
        '''
        if not self.load_action(action_id):
            rospy.logwarn(
                "Can't switch actions: failed to load action {}".format(
                    action_id))
            return False
        self.get_current_action().initialize_viz(object_list)
        self._object_list = object_list
        self._update_experiment_state()
        return True

    def next_action(self, object_list):
        '''Switches to the next action.

        Regardless of whether it succeeds in doing so, the object_list
        is now used.

        Args:
            object_list ([Landmark]): List of Landmark (as defined by
                Landmark.msg), the current reference frames.

        Returns:
            bool: Whether successfully switched to the next action.
        '''
        index = self._session_actions.index(self.current_action_id)
        if index == len(self._session_actions) - 1:
            rospy.logerr('Already on the last action.')
            return False
        action_id = self._session_actions[index + 1]
        return self.switch_to_action(action_id, object_list)

    def previous_action(self, object_list):
        '''Switches to the previous action.

        Regardless of whether it succeeds in doing so, the object_list
        is now used.

        Args:
            object_list ([Landmark]): List of Landmark (as defined by
                Landmark.msg), the current reference frames.

        Returns:
            bool: Whether successfully switched to the previous action.
        '''
        index = self._session_actions.index(self.current_action_id)
        if index == 0:
            rospy.logerr('Already on the first action.')
            return False
        action_id = self._session_actions[index - 1]
        return self.switch_to_action(action_id, object_list)

    def n_frames(self):
        '''Returns the number of steps in the current action, or 0 if
        there is no current action.

        Returns:
            int
        '''
        if self.n_actions() > 0:
            return self.actions[self.current_action_id].n_frames()
        else:
            rospy.logwarn(
                "Can't get number of frames (steps): No actions created yet.")
            return 0

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _selected_step_cb(self, selected_step):
        '''Updates the selected step when interactive markers are
        clicked on.

        Args:
            selected_step (int): ID of the step selected.
        '''
        self._selected_step = selected_step
        self._async_update_experiment_state()

    def _get_experiment_state_cb(self, __):
        '''Response to the experiment state service call.

        Args:
            __ (GetExperimentStateRequest): Unused.
        '''
        return GetExperimentStateResponse(self._get_experiment_state())

    def _async_update_experiment_state(self):
        '''Launches a new thread to asynchronously update experiment
        state.'''
        threading.Thread(group=None,
                         target=self._update_experiment_state,
                         name='experiment_state_publish_thread').start()

    def _update_experiment_state(self):
        '''Publishes a message with the latest state.'''
        state = self._get_experiment_state()
        self._state_publisher.publish(state)

    def _get_experiment_state(self):
        '''Creates and returns a message with the latest state.

        Returns:
            ExperimentState
        '''
        index = 0
        try:
            index = self._session_actions.index(self.current_action_id)
        except ValueError:
            pass
        # TODO(jstn): ExperimentState expects i_current_action to be a 1-based
        # index, which is why we have index+1 below. I think this is just to
        # make the PbD GUI label actions as "Action1", "Action2", etc. but
        # that's no excuse to use a 1-based index internally, too.
        return ExperimentState(
            self.n_actions(), index + 1, self.n_frames(), self._selected_step,
            self._get_gripper_states(),
            self._get_ref_frame_names(), self._object_list)

    def _get_ref_frame_names(self):
        '''Returns a list of the names of the reference frames for the
        steps of the current action for arm_index.

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT

        Returns:
            [str]
        '''
        # This can be called before anything's set up.
        if self.n_actions() < 1:
            return []
        # Once we've got an action, we can query / return things.
        action = self.actions[self.current_action_id]
        return action.get_ref_frame_names()

    def _get_gripper_states(self):
        '''Returns a list of the gripper states for the current action
        for arm_index arm.

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT

        Returns:
            [int]: Each item either GripperState.OPEN or
                GripperState.CLOSED.
        '''
        # This can be called before anything's set up.
        if self.n_actions() < 1:
            return []
        # Once we've got an action, we can query / return things.
        action = self.actions[self.current_action_id]
        return action.get_gripper_states()

    def _load_session_state(self, object_list):
        '''Loads the experiment state from disk.

        Args:
            object_list ([Landmark]): List of Landmark (as defined by
                Landmark.msg), the current reference frames.
        '''
        # Load data from YAML file into Python dictionary.
        with open(self._data_dir + SAVE_FILENAME, 'r') as state_file:
            exp_state = yaml.load(state_file)

        # Create all actions.
        n_actions = exp_state[YAML_KEY_NACTIONS]
        session_actions = exp_state[YAML_KEY_ACTION_LIST]
        for action_id in session_actions:
            self.actions[action_id] = ProgrammedAction(action_id,
                                                       self._selected_step_cb)
            # Load each action's data from a ROS bag.
            self.actions[action_id].load(self._data_dir)

        # Select the correct starting action.
        self.current_action_id = exp_state[YAML_KEY_CURIDX]

        # NOTE(mbforbes): The object_list is not saved to
        # self._object_list here because this method is only called from
        # the constructor, where the object_list is already saved.
        self.actions[self.current_action_id].initialize_viz(object_list)
