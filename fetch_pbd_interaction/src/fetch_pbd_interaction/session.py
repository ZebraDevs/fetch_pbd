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
import json

# ROS builtins
from tf import TransformBroadcaster
from std_srvs.srv import Empty as EmptySrv
from std_msgs.msg import Empty as EmptyMsg
from std_msgs.msg import String
from rail_manipulation_msgs.srv import SuggestGrasps
from geometry_msgs.msg import PoseStamped

# Local
from fetch_pbd_interaction.action import Action
from fetch_pbd_interaction.arm_target import ArmTarget
from fetch_pbd_interaction.arm_trajectory  import ArmTrajectory
from fetch_pbd_interaction.grasp  import Grasp
from fetch_pbd_interaction.msg import SessionState, ExecutionStatus, \
                                        Landmark
from fetch_pbd_interaction.srv import GetSessionState, \
                                     GetSessionStateResponse, \
                                     GetObjectList

# ######################################################################
# Classes
# ######################################################################

class Session:
    '''Everything related to the state of actions and primitives
    in the current session
    '''

    def __init__(self, robot, _tf_listener, im_server, 
                from_file=None, to_file=None, 
                grasp_suggestion_service_name=None,
                grasp_feedback_topic=None,
                external_ee_link=None):
        '''
        Args:
            robot (Robot) : interface to lower level robot functionality
            tf_listener (TransformListener)
            im_server (InteractiveMarkerSerever)
        '''
        self._lock = threading.Lock()
        self._from_file = from_file
        self._to_file = to_file
        self._grasp_suggestion_service = grasp_suggestion_service_name
        self._grasp_feedback_topic = grasp_feedback_topic
        self._external_ee_link = external_ee_link
        
        self._json = {}
        self._robot = robot
        self._tf_listener = _tf_listener
        self._tf_broadcaster = TransformBroadcaster()

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
        # -1 means None selected, used because SessionState.msg uses int8
        self._selected_primitive = -1
        self._current_arm_trajectory = None
        self._marker_visibility = []
        self._head_busy = False

        self._actions_disabled = []

        # Publishers & Services
        self._state_publisher = rospy.Publisher('/fetch_pbd/session_state',
                                                SessionState,
                                                queue_size=10)
        self._status_publisher = rospy.Publisher('/fetch_pbd/fetch_pbd_status',
                                                String,
                                                queue_size=10)
        rospy.Service('/fetch_pbd/get_session_state', GetSessionState,
                      self._get_session_state_cb)
        
        self._get_object_list_srv = rospy.ServiceProxy(
                                                '/fetch_pbd/get_object_list',
                                                GetObjectList)
        self._update_world_srv = rospy.ServiceProxy('/fetch_pbd/update_world',
                                                    GetObjectList)
        rospy.wait_for_service('/fetch_pbd/update_world')
        rospy.loginfo("Got update_world service.")


        self._clear_world_objects_srv = \
                        rospy.ServiceProxy('/fetch_pbd/clear_world_objects', 
                                            EmptySrv)
        rospy.wait_for_service('/fetch_pbd/clear_world_objects')
        rospy.loginfo("Got clear_world_objects service.")

        self._load_session_state()
        rospy.loginfo("Session state loaded.")

        # Send initial broadcast of experiment state.
        self._update_session_state()

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def head_busy(self):
        '''Returns true if head is busy
        
        Returns:
            bool
        '''
        if self.n_actions() < 1 or self._current_action_id is None:
            return False or self._head_busy
        action = self._actions[self._current_action_id]
        return action.head_busy() or self._head_busy

    def select_action_primitive(self, primitive_id):
        ''' Makes the interactive marker for the indicated action primitive
        selected by showing the 6D controls.

        Args:
            primitive_id (int): ID of the primitive to select.
        '''
        # If already selected, un-select, else select
        rospy.loginfo("new: {}, selected: {}".format(primitive_id, 
                        self._selected_primitive))
        if primitive_id == self._selected_primitive:
            is_selected = False
        else:
            is_selected = True
        self._actions[self._current_action_id].select_primitive(primitive_id, 
                                                                is_selected)

    def new_action(self, name=None):
        '''Creates new action.'''
        self._clear_world_objects_srv()
        if self.n_actions() > 0:
            if not self._current_action_id is None:
                self.get_current_action().reset_viz()
            self._current_action_id = self.n_actions()
        else:
            self._current_action_id = 0

        # action_id = self._db.insert_new(name)
        action = Action(self._robot,
                        self._tf_listener,
                        self._im_server,
                        self._selected_primitive_cb,
                        self._action_change_cb,
                        self._current_action_id,
                        self._grasp_suggestion_service,
                        self._grasp_feedback_topic,
                        self._external_ee_link)
        if not name is None:
            action.set_name(name)
        else:
            time_str = datetime.datetime.now().strftime('%c')
            action.set_name("Untitled action {}".format(time_str))

        self._actions.update({
            self._current_action_id: action
        })
        self._actions_disabled.append(False)
        self._action_ids.append(self._current_action_id)
        self._marker_visibility = [True] * self.n_primitives()
        self._update_session_state()

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
        if self.n_actions() > 0 and not self._current_action_id is None:
            action = self._actions[self._current_action_id]
            return action
        else:
            return None

    def clear_current_action(self):
        '''Removes all primitives in the current action.'''
        if self.n_actions() > 0:
            current_action = self._actions[self._current_action_id]
            current_action.clear()
        else:
            rospy.logwarn("Can't clear action: No actions created yet.")
        self._update_session_state()

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
        if self.n_actions() > 0:
            current_action = self._actions[self._current_action_id]
            current_action.add_primitive(self._current_arm_trajectory, False)
            self.publish_primitive_tf()
            current_action.make_primitive_marker(-1)
        else:
            rospy.logwarn("Can't add primitive: No actions created yet.")
        self._update_session_state()
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
        self._update_session_state()

    def add_grasp_to_action(self, landmark):
        '''Add a grasp primitive to the current action.

        Args:
            landmark (Landmark)
        '''
        if self.n_actions() > 0:
            current_action = self._actions[self._current_action_id]
            primitive_number = current_action.n_primitives()
            grasp = Grasp(self._robot, self._tf_listener, 
                              self._im_server, 
                              self._grasp_suggestion_service,
                              self._grasp_feedback_topic,
                              self._external_ee_link, 
                              landmark,
                              primitive_number)
            current_action.add_primitive(grasp)            
        else:
            rospy.logwarn("Can't add grasp: No actions created yet.")
        self._update_session_state()

    def delete_last_primitive(self):
        '''Removes the last primitive of the action.'''
        if self.n_actions() > 0:
            current_action = self._actions[self._current_action_id]
            current_action.delete_last_primitive()
        else:
            rospy.logwarn("Can't delete last primitive:" +
                          " No actions created yet.")
        self._update_session_state()

    def switch_to_action_by_index(self, index):
        '''Switches to action with index

        Args:
            index (int): The action id to switch to.

        Returns:
            bool: Whether successfully switched to index action.
        '''
        self._im_server.clear()
        self._im_server.applyChanges()
        self._lock.acquire()
        self._selected_primitive = -1

        if index < 0 or index >= len(self._actions):
            rospy.logwarn("Index out of bounds: {}".format(index))
            return False

        if self.n_actions() > 0 and not self._current_action_id is None:
            self.get_current_action().reset_viz()

        if not index in self._actions:
            rospy.logwarn(
                "Can't switch actions: failed to load action {}".format(
                    index))
            return False
        else:
            rospy.loginfo(
                "Switching to action {}".format(
                    index))

        self._current_action_id = index
        self._clear_world_objects_srv()
        self._lock.release()
        try:
            rospy.wait_for_message('/fetch_pbd/action_loaded', EmptyMsg, timeout=1000)
        except Exception, e:
            rospy.logwarn("Timed out waiting for frontend to respond")
        self.get_current_action().initialize_viz()
        self._update_session_state()
        self.publish_primitive_tf()
        action = self._actions[self._current_action_id]
        for i in range(self.n_primitives()):
            try:
                self._tf_listener.waitForTransform("base_link",
                             "primitive_" + str(i),
                             rospy.Time.now(),
                             rospy.Duration(5.0))
                rospy.loginfo("primitive: {}".format(action.get_primitive(i)))
                action.get_primitive(i).update_viz(False)
            except Exception, e:
                rospy.loginfo("Frame primitive_" + str(i) +
                              " is not available.")
        return True

    def switch_to_action_by_name(self, name):
        '''Switches to action with name

            Args:
                name (str): The action name to switch to.

            Returns:
                bool: Whether successfully switched to index action.
        '''
        names = self._get_action_names()
        if name in names:
            index = names.index(name)
            return self.switch_to_action_by_index(index)
        else:
            return False

    def next_action(self):
        '''Switches to the next action.

        Returns:
            bool: Whether successfully switched to the next action.
        '''
        index = self._action_ids.index(self._current_action_id)
        if index == len(self._actions) - 1:
            rospy.logerr("Already on the last action.")
            return False
        action_id = self._action_ids[index + 1]
        return self.switch_to_action_by_index(action_id)

    def previous_action(self):
        '''Switches to the previous action.

        Returns:
            bool: Whether successfully switched to the previous action.
        '''
        index = self._action_ids.index(self._current_action_id)
        if index == 0:
            rospy.logerr("Already on the first action.")
            return False
        action_id = self._action_ids[index - 1]
        return self.switch_to_action_by_index(action_id)

    def n_primitives(self):
        '''Returns the number of primitives in the current action, or 0 if
        there is no current action.

        Returns:
            int
        '''
        if self.n_actions() > 0 and not self._current_action_id is None:
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
        if not self._current_action_id is None:
            self.get_current_action().set_name(name)
            self._update_session_state()

    def copy_action(self, action_id):
        '''Make a copy of action

        Args:
            action_id (int)
        '''
        self._clear_world_objects_srv()

        new_id = self.n_actions()

        map_fun = '''function(doc) {
                     emit(doc.id, doc);
                  }'''

        results = self._db.query(map_fun)

        # Load data from db into Action objects.
        for result in results:
            if int(result.value['id']) == action_id:
                action = Action(self._robot, self._tf_listener, self._im_server,
                           self._selected_primitive_cb, self._action_change_cb,
                           grasp_suggestion_service=self._grasp_suggestion_service,
                           grasp_feedback_topic=self._grasp_feedback_topic,
                           external_ee_link=self._external_ee_link)
                result.value['id'] = new_id
                action.build_from_json(result.value)
                name = action.get_name()
                action.set_name("Copy of " + name)
                self._actions[new_id] = action
                self._action_ids.append(new_id)

        self._current_action_id = new_id
        self._update_session_state()


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
                        elif primitive.has_key('grasp'):
                            target = primitive['grasp']
                            primitive_copy = Grasp(self._robot,
                                               self._tf_listener,
                                               self._im_server,
                                               self._grasp_suggestion_service,
                                               self._grasp_feedback_topic,
                                               self._external_ee_link)
                            primitive_copy.build_from_json(target)
                            primitive_copy.set_primitive_number(new_num)
                            break

        self._actions[self._current_action_id].add_primitive(primitive_copy)

        self._actions[self._current_action_id].update_viz()

        self._update_session_state()

    def delete_action_current_action(self):
        '''Delete current action'''
        self.delete_action(self._current_action_id)

    def delete_action(self, action_id):
        '''Deletes action by id

        Args:
            action_id (string|int)
        '''
        if self.n_actions > 0:
            self._lock.acquire()
            rospy.loginfo('Deleting action')
            if int(action_id) == self._current_action_id:
                if len(self._action_ids) == 1:
                    self._current_action_id = None
                    self._actions = {}
                    self._action_ids = []
                    action_id_str = str(action_id)
                    if action_id_str in self._db:
                        self._db.delete(self._db[action_id_str])
                    self._lock.release()
                    self._update_session_state()
                    return
                else:
                    self._current_action_id-=1

            action_id_str = str(action_id)
            # rospy.loginfo(self._actions)
            if action_id in self._actions:
                del self._actions[int(action_id)]
            if action_id_str in self._db:
                self._db.delete(self._db[action_id_str])

            # new_actions = {}
            for a_id in range(int(action_id) + 1, len(self._actions)):
                action = self._actions[a_id]
                action.decrease_id()
                del self._actions[a_id]
                rospy.loginfo("id: {}".format(action.get_action_id()))
                self._actions[action.get_action_id()] = action

                if str(a_id) in self._db:
                    self._db.delete(self._db[str(a_id)])

                self._update_db_with_action(action)

            if len(self._action_ids) > 0:
                self._action_ids.pop()
            if int(action_id) == self._current_action_id:
                self._current_action_id = self._action_ids[-1]
            elif int(action_id) < self._current_action_id:
                self._current_action_id = self._current_action_id - 1
            self._lock.release()
            self._update_session_state()

    def switch_primitive_order(self, old_index, new_index):
        '''Change the order of primitives in action

        Args:
            old_index (int)
            new_index (int)
        '''
        rospy.loginfo("Switching primitive order")

        self._lock.acquire()
        action = self._actions[self._current_action_id]
        action.switch_primitive_order(old_index, new_index)
        self._update_db_with_current_action()
        self._lock.release()
        self.publish_primitive_tf()
        action = self._actions[self._current_action_id]
        primitives = action.get_primitives()
        for primitive in primitives:
            primitive.update_viz(False)
        self._update_session_state()

    def delete_primitive(self, primitive_number):
        '''Delete specified primitive

        Args:
            primitive_number (int) : Number of primitive to be deleted
        '''
        # self._lock.acquire()
        action = self._actions[self._current_action_id]
        action.delete_primitive(primitive_number)
        # self._lock.release()
        # self._update_db_with_current_action()

    def hide_primitive_marker(self, primitive_number):
        '''Hide marker with primitive_number

        Args:
            primitive_number (int)
        '''
        # self._lock.acquire()
        # self._marker_visibility[primitive_number] = False
        action = self._actions[self._current_action_id]
        action.delete_primitive_marker(primitive_number)
        self._update_session_state()
        # self._lock.release()

    def show_primitive_marker(self, primitive_number):
        '''Show marker with primitive_number

        Args:
            primitive_number (int)
        '''
        # self._lock.acquire()
        # self._marker_visibility[primitive_number] = True
        action = self._actions[self._current_action_id]
        action.make_primitive_marker(primitive_number)
        self._update_session_state()
        # self._lock.release()

    def execute_primitive(self, primitive_number):
        '''Execute primitive with number

        Args:
            primitive_number (int)
        '''
        rospy.loginfo("Executing primitive")

        action = self._actions[self._current_action_id]
        primitive = action.get_primitive(primitive_number)
        if primitive.is_object_required():
            # We need an object; check if we have one.
            self._head_busy = True
            rospy.loginfo("Object required for execution")
            self._robot.look_down()
            resp = self._update_world_srv()
            if resp.object_list:
                rospy.loginfo("Object list not empty")
                # An object is required, and we got one. Execute.
                self.get_current_action().update_objects()
                success, msg = primitive.check_pre_condition()
                if not success:
                    rospy.logwarn(
                        "\tPreconditions of primitive " + 
                        str(primitive.get_name()) + " are not " +
                        "satisfied. " + msg)
                    self._status_publisher.publish(
                        String("Preconditions of primitive " + 
                        str(primitive.get_name()) +
                        " are not satisfied. " + msg))
                primitive.execute()
            else:
                rospy.logwarn("Needs object(s) but none available")
                self._status_publisher.publish(
                        "Primitive {}".format(primitive.get_name()) +
                        "requires an object but none are available")
            self._head_busy = False
        else:
            success, msg = primitive.check_pre_condition()
            if not success:
                rospy.logwarn(
                        "\tPreconditions of primitive " + 
                        str(primitive.get_name()) + " are not " +
                        "satisfied. " + msg)
                self._status_publisher.publish(
                    String("Preconditions of primitive " + 
                    str(primitive.get_name()) +
                    " are not satisfied. " + msg))
            primitive.execute()

    def record_objects(self):
        '''Records poses of objects
        '''
        self._robot.look_down()
        resp = self._update_world_srv()
        if resp.object_list:
            if self.n_actions() > 0:
                self.get_current_action().update_objects()
            return  True
        else:
            return False

    def execute_current_action(self):
        '''Executes current action

        Returns:
            bool
        '''
        if self.n_actions() > 0 and not self._current_action_id is None:
            # We must have also recorded primitives (/poses/frames) in it.
            if self.n_primitives() > 0:
                # Save curent action and retrieve it.
                # self._session.save_current_action()

                # Now, see if we can execute.
                if self.get_current_action().is_object_required():
                    # We need an object; check if we have one.
                    rospy.loginfo("An object is required for this action")
                    self._robot.look_down()
                    resp = self._update_world_srv()
                    self._robot.look_forward()

                    # objects = resp.objects
                    if resp.object_list:
                        # An object is required, and we got one. Execute.
                        action = self.get_current_action()
                        action.update_objects()
                        action.start_execution()
                        # Wait a certain max time for execution to finish
                        for i in range(1000):
                            status = action.get_status()
                            if status != ExecutionStatus.EXECUTING:
                                break
                            if i % 10 == 0:
                                rospy.loginfo("Still executing")
                            rospy.sleep(0.1)
                        if status == ExecutionStatus.SUCCEEDED:
                            action.end_execution()
                            return True
                        else:
                            action.end_execution()
                            return False

                    else:
                        # An object is required, but we didn't get it.
                        rospy.logwarn("An object is required for" + 
                                        " this action but none were found")
                        return False
                else:
                    # No object is required: start execution now.
                    action = self.get_current_action()
                    action.start_execution()

                    for i in range(1000):
                        
                        status = action.get_status()
                        if status != ExecutionStatus.EXECUTING:
                            break
                        if i % 10 == 0:
                            rospy.loginfo("Still executing")
                        rospy.sleep(0.1)
                    if status == ExecutionStatus.SUCCEEDED:
                        action.end_execution()
                        return True
                    else:
                        rospy.logwarn("Execution failed, with status: {}".format(status))
                        action.end_execution()
                        return False
            else:
                # No primitives / poses / frames recorded.
                rospy.logwarn("No primitives recorded")
                return False
        else:
            # No actions.
            rospy.logwarn("No current action")
            return False

    def publish_primitive_tf(self):
        '''Publish tf frame for each primitive of current action'''
        if not self._current_action_id is None:
            # self._lock.acquire()
            action = self._actions[self._current_action_id]
            primitives = action.get_primitives()
            # self._lock.release()
            for primitive in primitives:
                self._publish_primitive_tf(primitive)

    def update_primitive_pose(self, primitive_number, position, orientation):
        '''Update pose of primitive given by primitive_number

        Args:
            primitive_number (int)
            position (Point)
            orientation (OrientationRPY)
        '''
        action = self._actions[self._current_action_id]
        action.update_primitive_pose(primitive_number, position, orientation)

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _get_marker_visibility(self):
        '''Get the visibility of the markers

        Returns:
            [bool]
        '''
        if self._current_action_id is None:
            return []
        return self._actions[self._current_action_id].get_marker_visibility()

    def _update_db_with_action(self, action, db_file=None):
        '''Adds action to db if it does not already exist.
        Or if it exists, delete the existing entry and replace with
        update version

        Args:
            action (Action)
        '''
        rospy.loginfo("Updating database")
        if db_file is None:
            db_file = self._to_file
        action_json = action.get_json()
        # rospy.loginfo("json: {}".format(json))
        action_id_str = str(action.get_action_id())
        if action_id_str in self._db:

            self._db.delete(self._db[action_id_str])
            self._db[action_id_str] = action_json
        else:
            self._db[action_id_str] = action_json
        if db_file:
            self._json[action.get_action_id()] = action_json
            with open(db_file, 'w') as to_file:
                json.dump(self._json, to_file)

    def _update_db_with_current_action(self):
        '''Adds current action to db if it does not already exist.
        Or if it exists, delete the existing entry and replace with
        update version
        '''
        action = self.get_current_action()
        if not action is None:
            self._update_db_with_action(action)


    def _selected_primitive_cb(self, selected_primitive):
        '''Updates the selected primitive when interactive markers are
        clicked on.

        Args:
            selected_primitive (int): ID of the primitive selected.
        '''
        rospy.loginfo("Setting primitive to {}".format(selected_primitive))
        self._selected_primitive = selected_primitive

        self._async_update_session_state()
        rospy.loginfo("Done setting primitive to {}".format(selected_primitive))

    def _action_change_cb(self):
        '''Updates the db when primitive deleted.
        '''
        self.publish_primitive_tf()
        self._async_update_session_state()
        rospy.loginfo("Session state updates")

    def _get_session_state_cb(self, req):
        '''Response to the experiment state service call.

        Args:
            req (GetSessionStateRequest): Unused.
        Returns:
            GetSessionStateResponse
        '''
        return GetSessionStateResponse(self._get_session_state())

    def _async_update_session_state(self):
        '''Launches a new thread to asynchronously update experiment
        state.'''
        threading.Thread(group=None,
                         target=self._update_session_state,
                         name='session_state_publish_thread').start()

    def _update_session_state(self):
        '''Publishes a message with the latest state.'''
        rospy.loginfo("Session state update")
        if len(self._actions) > 0:
            self._lock.acquire()
            self._update_db_with_current_action()
            self._lock.release()
        state = self._get_session_state()
        self._state_publisher.publish(state)
        rospy.loginfo("Session state updated")

    def _get_session_state(self):
        '''Creates and returns a message with the latest state.

        Returns:
            SessionState
        '''
        rospy.loginfo("Getting session state")
        index = self._current_action_id

        # Should the GUI retrieve this information itself?
        object_list = self._get_object_list_srv().object_list
        positions, orientations = self._get_primitive_positions_orientations()
        rospy.loginfo("Got positions and orientations")

        return SessionState(
            self.n_actions(),
            index,
            self.n_primitives(),
            self._selected_primitive,
            self._get_action_names(),
            self._get_ref_frame_names(),
            self._get_primitive_names(),
            self._get_actions_disabled(),
            self._get_marker_visibility(),
            self._get_primitives_editable(),
            [],
            object_list,
            positions,
            orientations)

    def _get_action_names(self):
        '''Return the names of all of the actions in the session

        Returns:
            [string]
        '''
        name_list = []
        for key in self._actions:

            name_list.append(self._actions[key].get_name())

        return name_list

    def _get_ref_frame_names(self):
        '''Returns a list of the names of the reference frames for the
        primitives of the current action.

        Returns:
            [str]
        '''
        # This can be called before anything's set up.
        if self.n_actions() < 1 or self._current_action_id is None:
            return []
        # Once we've got an action, we can query / return things.
        action = self._actions[self._current_action_id]
        return action.get_ref_frame_names()

    def _get_primitive_positions_orientations(self):
        '''Returns positions and orientations of primitives

        Returns:
            Point[], OrientationRPY[]
        '''
        if self.n_actions() < 1 or self._current_action_id is None:
            return [], []
        # Once we've got an action, we can query / return things.
        action = self._actions[self._current_action_id]
        return action.get_primitive_positions_orientations()

    def _get_primitive_names(self):
        '''Returns a list of the names of the
        primitives of the current action.

        Returns:
            [str]
        '''
        # This can be called before anything's set up.
        if self.n_actions() < 1 or self._current_action_id is None:
            return []
        # Once we've got an action, we can query / return things.
        action = self._actions[self._current_action_id]
        return action.get_primitive_names()

    def _get_actions_disabled(self):
        '''Returns whether each action is disabled currently 
        (due to grasp suggestion not being available)

        Returns:
            [bool]
        '''
        if not self._actions_disabled:
            return [True] * self.n_actions()
        return self._actions_disabled

    def _get_primitives_editable(self):
        '''Returns list of whether primitive poses are editable

        Returns:
            [bool]
        '''
        if self.n_actions() < 1 or self._current_action_id is None:
            return []
        # Once we've got an action, we can query / return things.
        action = self._actions[self._current_action_id]
        return action.get_primitives_editable()

    def _load_session_state(self):
        '''Loads the experiment state from couchdb database or json file.'''
        # It retrieves actions from db sorted by their integer ids.

        map_fun = '''function(doc) {
                     emit(doc.id, doc);
                  }'''

        results = self._db.query(map_fun)

        # Load data from db into Action objects.
        for result in results:
            action = Action(self._robot, self._tf_listener, self._im_server,
                       self._selected_primitive_cb, self._action_change_cb, 
                       grasp_suggestion_service=self._grasp_suggestion_service,
                       grasp_feedback_topic=self._grasp_feedback_topic,
                       external_ee_link=self._external_ee_link)
            success = action.build_from_json(result.value)
            self._actions_disabled.append(not success)
            self._actions[int(result.value['id'])] = action
            self._action_ids.append(int(result.value['id']))
            self._update_db_with_action(action)

        if self._from_file:
            time_str = datetime.datetime.now().strftime('%c')
            for key in self._actions:
                action = self._actions[key]
                self._update_db_with_action(action, time_str + '.json')

            del self._couch['fetch_pbd']
            self._db = self._couch.create('fetch_pbd')
            self._actions = {}
            self._action_ids = []
            with open(self._from_file) as json_file:
                self._json = json.load(json_file)

            keys = self._json.keys()
            keys.sort()
            self._actions_disabled = []
            for key in keys:
                action = Action(self._robot, self._tf_listener, self._im_server,
                       self._selected_primitive_cb, self._action_change_cb,
                       grasp_suggestion_service=self._grasp_suggestion_service,
                       grasp_feedback_topic=self._grasp_feedback_topic,
                       external_ee_link=self._external_ee_link)
                success = action.build_from_json(self._json[key])
                self._actions_disabled.append(not success)
                self._actions[int(self._json[key]['id'])] = action
                self._action_ids.append(int(self._json[key]['id']))
                self._update_db_with_action(action)

        if True in self._actions_disabled:
            self._status_publisher.publish(String("Some actions have been " +
                       "disabled because they require grasp suggestion."))

        # if len(self._actions) > 0:
        #     # Select the starting action as the action with the largest id
        #     self._current_action_id = self._action_ids[-1]

        #     # self._actions[self._current_action_id].initialize_viz()

    def _publish_primitive_tf(self, primitive, parent="base_link"):
        ''' Publishes a TF for primitive

        Args:
            primitive (Primitive)
            parent (str): The parent reference frame.
        '''
        try:
            marker_pose = primitive.get_absolute_marker_pose()
            # rospy.loginfo("Publishing primitive TF")
            if marker_pose:
                pose = self._tf_listener.transformPose('base_link', marker_pose)
                position = pose.pose.position
                orientation = pose.pose.orientation
                pos = (position.x, position.y, position.z)
                rot = (orientation.x, orientation.y, 
                        orientation.z, orientation.w)
                name = "primitive_" + str(primitive.get_number())
                # TODO(mbforbes): Is it necessary to change the position
                # and orientation into tuples to send to TF?
                self._tf_broadcaster.sendTransform(
                    pos, rot, rospy.Time.now(), name, parent)
        except Exception, e:
            rospy.logwarn(str(e))
