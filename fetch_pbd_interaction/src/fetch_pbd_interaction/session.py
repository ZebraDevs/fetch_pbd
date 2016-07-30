'''Everything related to the state of actions and primitives
in the current session
'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# System builtins
import datetime
import threading
import couchdb
import threading

# Local
from fetch_pbd_interaction.action import Action
from fetch_pbd_interaction.arm_target import ArmTarget
from fetch_pbd_interaction.arm_trajectory  import ArmTrajectory
from fetch_pbd_interaction.msg import ExperimentState
from fetch_pbd_interaction.srv import GetExperimentState, \
                                     GetExperimentStateResponse, \
                                     GetObjectList


# ######################################################################
# Classes
# ######################################################################

class Session:
    '''Everything related to the state of actions and primitives
    in the current session
    '''

    def __init__(self, robot, _tf_listener, im_server):
        '''
        Args:
            robot (Robot) : interface to lower level robot functionality
            tf_listener (TransformListener)
            im_server (InteractiveMarkerSerever)
        '''
        self._lock = threading.Lock()

        self._robot = robot
        self._tf_listener = _tf_listener
        self._im_server = im_server

        self._couch = couchdb.Server()
        try:
            self._db = self._couch['fetch_pbd']
        except:
            self._db = self._couch.create('fetch_pbd')

        # Dict (keys = ids, values = actions)
        self._actions = {}
        # Sorted list of action ids
        self._action_ids = []
        self._current_action_id = None
        self._selected_primitive = 0
        self._current_arm_trajectory = None

        # Publishers & Services
        self._state_publisher = rospy.Publisher('experiment_state',
                                                ExperimentState,
                                                queue_size=10)
        rospy.Service('get_experiment_state', GetExperimentState,
                      self._get_experiment_state_cb)
        self._get_object_list_srv = rospy.ServiceProxy('get_object_list',
                                                       GetObjectList)

        # Load saved actions
        self._load_session_state()
        rospy.loginfo("Session state loaded.")

        # Send initial broadcast of experiment state.
        self._update_experiment_state()

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def select_action_primitive(self, primitive_id):
        ''' Makes the interactive marker for the indicated action primitive
        selected by showing the 6D controls.

        Args:
            primitive_id (int): ID of the primitive to select.
        '''
        self._actions[self._current_action_id].select_primitive(primitive_id)
        self._selected_primitive = primitive_id


    def new_action(self, name=None):
        '''Creates new action.'''
        if self.n_actions() > 0:
            self.get_current_action().reset_viz()
            self._current_action_id = self.n_actions()
        else:
            self._current_action_id = 0

        time_str = datetime.datetime.now().strftime('%c')
        # action_id = self._db.insert_new(name)
        action = Action(self._robot,
                        self._tf_listener,
                        self._im_server,
                        self._selected_primitive_cb,
                        self._delete_primitive_cb,
                        self._current_action_id)
        if not name is None:
            action.set_name(name)
        else:
            action.set_name('Untitled action {}'.format(time_str))

        self._actions.update({
            self._current_action_id: action
        })
        self._action_ids.append(self._current_action_id)
        self._update_experiment_state()

    def n_actions(self):
        '''Returns the number of actions programmed so far.

        Returns:
            int
        '''
        return len(self._actions)

    def get_current_action(self):
        '''Returns the current action.

        Returns:
            Action
        '''
        self._lock.acquire()
        if self.n_actions() > 0:
            action = self._actions[self._current_action_id]
            self._lock.release()
            return action
        else:
            self._lock.release()
            return None

    def clear_current_action(self):
        '''Removes all primitives in the current action.'''
        if self.n_actions() > 0:
            current_action = self._actions[self._current_action_id]
            current_action.clear()
        else:
            rospy.logwarn("Can't clear action: No actions created yet.")
        self._update_experiment_state()

    def update_arm_trajectory(self):
        '''Saves current arm state into continuous trajectory.'''
        if self._current_arm_trajectory is not None:
            arm_state = self._robot.get_arm_state()
            gripper_state = self._robot.get_gripper_state()
            self._current_arm_trajectory.add_step(arm_state,
                                                       gripper_state)

    def start_recording_arm_trajectory(self):
        '''Creates ArmTrajectory primitive that will be updated
        by update_arm_trajectory
        '''
        current_action = self._actions[self._current_action_id]
        primitive_number = current_action.n_primitives()
        self._current_arm_trajectory = ArmTrajectory(self._robot,
                    self._tf_listener, self._im_server, primitive_number)

    def stop_recording_arm_trajectory(self):
        '''Finalises ArmTrajectory primitive.
        Updates the references frames and saves it to the db.
        '''

        self._current_arm_trajectory.choose_dominant_ref_frame()

        if self.n_actions() > 0:
            current_action = self._actions[self._current_action_id]
            current_action.add_primitive(self._current_arm_trajectory)
        else:
            rospy.logwarn("Can't add primitive: No actions created yet.")
        self._update_experiment_state()
        self._current_arm_trajectory = None

    def add_arm_target_to_action(self):
        '''Add a new ArmTarget primitive to the current action.'''

        if self.n_actions() > 0:
            arm_state = self._robot.get_arm_state()
            gripper_state = self._robot.get_gripper_state()
            rospy.loginfo("gripper_state: {}".format(gripper_state))
            current_action = self._actions[self._current_action_id]
            primitive_number = current_action.n_primitives()
            arm_target = ArmTarget(self._robot, self._tf_listener,
                                   self._im_server, arm_state,
                                   gripper_state, primitive_number)

            current_action.add_primitive(arm_target)
        else:
            rospy.logwarn("Can't add primitive: No actions created yet.")
        self._update_experiment_state()

    def delete_last_primitive(self):
        '''Removes the last primitive of the action.'''
        if self.n_actions() > 0:
            current_action = self._actions[self._current_action_id]
            current_action.delete_last_primitive()
        else:
            rospy.logwarn("Can't delete last primitive:" +
                          " No actions created yet.")
        self._update_experiment_state()

    def switch_to_action(self, index):
        '''Switches to action with index

        Args:
            index (int): The action id to switch to.

        Returns:
            bool: Whether successfully switched to index action.
        '''

        if index < 0 or index >= len(self._actions):
            rospy.logwarn("Index out of bounds: {}".format(index))
            return False

        if self.n_actions() > 0:
            self.get_current_action().reset_viz()

        if not index in self._actions:
            rospy.logwarn(
                "Can't switch actions: failed to load action {}".format(
                    index))
            return False

        self._current_action_id = index

        self.get_current_action().initialize_viz()
        self._update_experiment_state()
        return True

    def next_action(self):
        '''Switches to the next action.

        Returns:
            bool: Whether successfully switched to the next action.
        '''
        index = self._action_ids.index(self._current_action_id)
        if index == len(self._actions) - 1:
            rospy.logerr('Already on the last action.')
            return False
        action_id = self._action_ids[index + 1]
        return self.switch_to_action(action_id)

    def previous_action(self):
        '''Switches to the previous action.

        Returns:
            bool: Whether successfully switched to the previous action.
        '''
        index = self._action_ids.index(self._current_action_id)
        if index == 0:
            rospy.logerr('Already on the first action.')
            return False
        action_id = self._action_ids[index - 1]
        return self.switch_to_action(action_id)

    def n_primitives(self):
        '''Returns the number of primitives in the current action, or 0 if
        there is no current action.

        Returns:
            int
        '''
        if self.n_actions() > 0:
            return self._actions[self._current_action_id].n_primitives()
        else:
            rospy.logwarn(
                "Can't get number of primitives: No actions created yet.")
            return 0

    def update_action_name(self, name):
        '''Update name of current action

        Args:
            name (string)
        '''
        self.get_current_action().set_name(name)
        self._update_experiment_state()

    def copy_action(self, action_id):
        '''Make a copy of action

        Args:
            action_id (int)
        '''
        new_id = self.n_actions()

        map_fun = '''function(doc) {
                     emit(doc.id, doc);
                  }'''

        results = self._db.query(map_fun)

        # Load data from db into Action objects.
        for result in results:
            if int(result.value['id']) == action_id:
                action = Action(self._robot, self._tf_listener, self._im_server,
                           self._selected_primitive_cb, self._delete_primitive_cb)
                result.value['id'] = new_id
                action.build_from_json(result.value)
                name = action.get_name()
                action.set_name("Copy of " + name)
                self._actions[new_id] = action
                self._action_ids.append(new_id)

        self._current_action_id = new_id
        self._update_experiment_state()


    def copy_primitive(self, primitive_number):
        '''Make a copy of a primitive

        Args:
            primitive (int)
        '''
        # new_id = self.n_actions()

        map_fun = '''function(doc) {
                     emit(doc.id, doc);
                  }'''

        results = self._db.query(map_fun)

        # Load data from db into Action objects.
        new_num = self.n_primitives()
        for result in results:
            if int(result.value['id']) == self._current_action_id:
                for idx, primitive in enumerate(result.value['seq']):
                    if idx == primitive_number:
                        if primitive.has_key('arm_target'):
                            target = primitive['arm_target']
                            primitive_copy = ArmTarget(self._robot,
                                                       self._tf_listener,
                                                       self._im_server)
                            primitive_copy.build_from_json(target)
                            primitive_copy.set_primitive_number(new_num)
                            break

                        elif primitive.has_key('arm_trajectory'):
                            target = primitive['arm_trajectory']
                            primitive_copy = ArmTrajectory(self._robot,
                                                           self._tf_listener,
                                                           self._im_server)
                            primitive_copy.build_from_json(target)
                            primitive_copy.set_primitive_number(new_num)
                            break

        self._actions[self._current_action_id].add_primitive(primitive_copy)

        self._actions[self._current_action_id].update_viz()

        self._update_experiment_state()

    def delete_action_current_action(self):
        '''Delete current action'''
        self.delete_action(self._current_action_id)

    def delete_action(self, action_id):
        '''Deletes action by id

        Args:
            action_id (string|int)
        '''
        self._lock.acquire()
        rospy.loginfo("actions: {}".format(self._actions))

        if int(action_id) == self._current_action_id:
            if len(self._action_ids) == 1:
                self._current_action_id = None
                self._actions = {}
                self._action_ids = []
                action_id_str = str(action_id)
                if action_id_str in self._db:
                    self._db.delete(self._db[action_id_str])
                self._lock.release()
                return

        action_id_str = str(action_id)
        rospy.loginfo("actions: {}".format(self._actions))
        del self._actions[int(action_id)]
        if action_id_str in self._db:
            self._db.delete(self._db[action_id_str])

        # new_actions = {}
        for a_id in range(int(action_id) + 1, len(self._action_ids)):
            action = self._actions[a_id]
            action.decrease_id()
            del self._actions[a_id]
            rospy.loginfo("id: {}".format(action.get_action_id()))
            self._actions[action.get_action_id()] = action

            if str(a_id) in self._db:
                self._db.delete(self._db[str(a_id)])

            self._update_db_with_action(action)

        self._action_ids.pop()
        if int(action_id) == self._current_action_id:
            self._current_action_id = self._action_ids[-1]
        elif int(action_id) < self._current_action_id:
            self._current_action_id = self._current_action_id - 1
        self._lock.release()

        rospy.loginfo("actions: {}".format(self._actions))
        self._update_experiment_state()

    def switch_primitive_order(self, old_index, new_index):
        '''Change the order of primitives in action

        Args:
            old_index (int)
            new_index (int)
        '''
        self._lock.acquire()
        action = self._actions[self._current_action_id]
        action.switch_primitive_order(old_index, new_index)
        self._lock.release()

    def delete_primitive(self, primitive_number):
        '''Delete specified primitive

        Args:
            primitive_number (int) : Number of primitive to be deleted
        '''
        self._actions[self._current_action_id].delete_primitive(primitive_number)

    def execute_primitive(self, primitive_number):
        '''Execute specified primitive

        Args:
            primitive_number (int) : Number of primitive to execute
        '''
        self._actions[self._current_action_id].execute_primitive(primitive_number)

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _update_db_with_action(self, action):
        '''Adds action to db if it does not already exist.
        Or if it exists, delete the existing entry and replace with
        update version

        Args:
            action (Action)
        '''
        json = action.get_json()
        rospy.loginfo("json: {}".format(json))
        action_id_str = str(action.get_action_id())
        if action_id_str in self._db:
            self._db.delete(self._db[action_id_str])
            self._db[action_id_str] = json
        else:
            self._db[action_id_str] = json

    def _update_db_with_current_action(self):
        '''Adds current action to db if it does not already exist.
        Or if it exists, delete the existing entry and replace with
        update version
        '''
        self._update_db_with_action(self.get_current_action())


    def _selected_primitive_cb(self, selected_primitive):
        '''Updates the selected primitive when interactive markers are
        clicked on.

        Args:
            selected_primitive (int): ID of the primitive selected.
        '''
        self._selected_primitive = selected_primitive
        self._async_update_experiment_state()

    def _delete_primitive_cb(self):
        '''Updates the db when primitive deleted.
        '''
        self._async_update_experiment_state()

    def _get_experiment_state_cb(self, req):
        '''Response to the experiment state service call.

        Args:
            req (GetExperimentStateRequest): Unused.
        Returns:
            GetExperimentStateResponse
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
        if len(self._actions) > 0:
            self._update_db_with_current_action()
        state = self._get_experiment_state()
        self._state_publisher.publish(state)

    def _get_experiment_state(self):
        '''Creates and returns a message with the latest state.

        Returns:
            ExperimentState
        '''
        index = self._current_action_id

        # Should the GUI retrieve this information itself?
        object_list = self._get_object_list_srv().object_list

        return ExperimentState(
            self.n_actions(), index, self.n_primitives(),
            self._selected_primitive,
            self._get_action_names(),
            self._get_ref_frame_names(),
            self._get_primitive_names(),
            [],
            object_list)

    def _get_action_names(self):
        '''Return the names of all of the actions in the session

        Returns:
            [string]
        '''
        name_list = []
        for action_id in self._action_ids:
            name_list.append(self._actions[action_id].get_name())

        return name_list

    def _get_ref_frame_names(self):
        '''Returns a list of the names of the reference frames for the
        primitives of the current action.

        Returns:
            [str]
        '''
        # This can be called before anything's set up.
        if self.n_actions() < 1:
            return []
        # Once we've got an action, we can query / return things.
        action = self._actions[self._current_action_id]
        return action.get_ref_frame_names()

    def _get_primitive_names(self):
        '''Returns a list of the names of the
        primitives of the current action.

        Returns:
            [str]
        '''
        # This can be called before anything's set up.
        if self.n_actions() < 1:
            return []
        # Once we've got an action, we can query / return things.
        action = self._actions[self._current_action_id]
        return action.get_primitive_names()

    def _load_session_state(self):
        '''Loads the experiment state from couchdb database.'''
        # It retrieves actions from db sorted by their integer ids.
        map_fun = '''function(doc) {
                     emit(doc.id, doc);
                  }'''

        results = self._db.query(map_fun)

        # Load data from db into Action objects.
        for result in results:
            action = Action(self._robot, self._tf_listener, self._im_server,
                       self._selected_primitive_cb, self._delete_primitive_cb)
            action.build_from_json(result.value)
            self._actions[result.value['id']] = action
            self._action_ids.append(int(result.value['id']))

        if len(self._actions) > 0:
            # Select the starting action as the action with the largest id
            self._current_action_id = self._action_ids[-1]

            self._actions[self._current_action_id].initialize_viz()
