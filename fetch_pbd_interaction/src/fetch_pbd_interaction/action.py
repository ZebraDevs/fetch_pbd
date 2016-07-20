'''The in-program representation of a programmed action.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# System builtins
import threading

# ROS builtins
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

# Local
from fetch_pbd_interaction.arm_target import ArmTarget
from fetch_pbd_interaction.arm_trajectory import ArmTrajectory
from fetch_pbd_interaction.msg import ExecutionStatus

# ######################################################################
# Module level constants
# ######################################################################

# Marker properties for little arrows drawn between consecutive steps.
LINK_MARKER_LIFETIME = rospy.Duration()
LINK_SCALE = Vector3(0.01, 0.03, 0.01)
LINK_COLOR = ColorRGBA(0.8, 0.8, 0.8, 0.3)  # sort of light gray

# ROS topics, etc.
TOPIC_MARKERS = 'visualization_marker_array'

# TODO(sarah): Is this necessary?
BASE_LINK = 'base_link'

# ######################################################################
# Classes
# ######################################################################


class Action:
    '''Holds information for one action.'''

    # TODO(sarah) : Probably get rid of this. Should the class get passed a
    # shared marker publisher from the Session or each instance should have
    # its own?
    _marker_publisher = None

    def __init__(self, robot, tf_listener, im_server, primitive_click_cb,
                 delete_primitive_cb, action_id=None):
        '''
        Args:
            robot (Robot) : interface to lower level robot functionality
            tf_listener (TransformListener)
            im_server (InteractiveMarkerSerever)
            primitive_click_cb (function(int)): The function to call when a
                step is clicked on (normally in the GUI). The function
                should take the step number of the step
            action_id (int, optional): The index of this action.

        '''
        # Initialize a bunch of state.
        self._name = ''  # Human-friendly name for this action.
        self._im_server = im_server
        self._seq = []
        self._action_id = action_id
        self._robot = robot
        self._primitive_click_cb = primitive_click_cb
        self._delete_primitive_cb = delete_primitive_cb
        self._status = ExecutionStatus.NOT_EXECUTING
        self._preempt = False
        self._z_offset = 0.0
        self._tf_listener = tf_listener

        # Markers to connect consecutive action steps together
        self._link_markers = {}

        # TODO(sarah): Understand this note better
        # NOTE(mbforbes): It appears that this is locking manipulation
        # of the internal sequence (self._seq). There have been race
        # conditions involving this (e.g. marker_click_cb(...)).
        #
        # In general, be aware the other code calling these methods
        # with data about this class (like how many steps it holds)
        # is bad because that means the outside code is assuming that it
        # knows about state internal to this class, and that information
        # may not be true by the time the code here gets executed. This
        # is because there are several callbacks that trigger here so
        # we must reason asyncronously.
        #
        # Unless the information you have (e.g. about the number of
        # steps that exist) was learned while this lock was acquired,
        # you cannot assume it is true.
        self._lock = threading.Lock()

        if Action._marker_publisher is None:
            Action._marker_publisher = rospy.Publisher(TOPIC_MARKERS,
                                                                 MarkerArray,
                                                                 queue_size=10)


    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def get_action_id(self):
        ''' Returns action_id

        Returns:
            int
        '''
        return self._action_id

    def get_json(self):
        '''Return json for this action for saving to db

        Returns:
            dict
        '''
        json = {}
        json['name'] = self._name
        json['id'] = self._action_id
        json['seq'] = []
        for primitive in self._seq:
            json['seq'].append(primitive.get_json())

        return json

    def build_from_json(self, json):
        '''Fills out action using information using json from db

        Args:
            dict : json/dict retrieved from couchdb
        '''
        self._action_id = json['id']
        self._name = json['name']
        for primitive in json['seq']:
            if primitive.has_key('arm_target'):
                target = primitive['arm_target']
                primitive = ArmTarget(self._robot, self._tf_listener,
                              self._im_server)
                primitive.build_from_json(target)

            elif primitive.has_key('arm_trajectory'):
                target = primitive['arm_trajectory']
                primitive = ArmTrajectory(self._robot, self._tf_listener,
                                  self._im_server)
                primitive.build_from_json(target)

            self.add_primitive(primitive)

        self.reset_viz()

    def start_execution(self, z_offset=0.0):
        ''' Starts execution of action.

        This method spawns a new thread.

        Args:
            z_offset (float): Amount to add to z-values of pose
                positions.
        '''
        # This will take long; create a thread.
        self._preempt = False
        self._z_offset = z_offset
        thread = threading.Thread(
            group=None,
            target=self._execute_action,
            name='action_execution_thread'
        )
        thread.start()

    def stop_execution(self):
        ''' Indicate that user wants to preempt action execution '''
        self._preempt = True

    def end_execution(self):
        ''' Indicate that execution status can reset to
            ExecutionStatus.NOT_EXECUTING
        '''
        self._status = ExecutionStatus.NOT_EXECUTING

    def get_status(self):
        '''Return execution status of action

        Returns:
            ExecutionStatus.EXECUTING|NOT_EXECUTING|...etc
        '''
        return self._status

    def set_status(self, status):
        '''Set execution status of action

        Args:
            status (ExecutionStatus.EXECUTING|NOT_EXECUTING|...etc)
        '''
        self._status = status


    def add_primitive(self, primitive):
        '''Add primitive to action.

        Args:
            primitive (Primitive)
        '''
        self._lock.acquire()
        rospy.loginfo("Adding primitive")

        self._seq.append(primitive)

        primitive.make_marker(
            self.marker_click_cb,  # marker_click_cb
            self._delete_primitive
        )

        self._update_links()

        self._update_markers()

        self._lock.release()

    def update_objects(self):
        '''For each primitive, updates the reference frames based on
        the locations of objects in the world
        '''
        self._lock.acquire()
        self._update_markers()
        for primitive in self._seq:
            primitive.update_ref_frames()
        self._lock.release()

    def n_primitives(self):
        '''Returns the number of primitives in this action.

        Returns:
            int
        '''
        return len(self._seq)

    def reset_viz(self):
        '''Removes all visualization from Rviz relating to this action.'''
        self._lock.acquire()

        # Destroy the action step markers.
        for primitive in self._seq:
            rospy.loginfo("Deleting marker")
            primitive.delete_marker()

        # Mark the links for destruction.
        for i in self._link_markers.keys():
            self._link_markers[i].action = Marker.DELETE

        # Publish the link destructions.
        m_array = MarkerArray()
        for i in self._link_markers.keys():
            m_array.markers.append(self._link_markers[i])
        self._marker_publisher.publish(m_array)

        # rospy.sleep(2.0)
        self._link_markers = {}
        self._lock.release()

    def marker_click_cb(self, primitive_number, is_selected):
        '''Callback for when one of the markers is clicked.
        Selects clicked marker and unselects others.

        Args:
            primitive_number (int)
            is_selected(bool): Whether the marker was
                selected (True) or de-selected (False).

        '''
        self._lock.acquire()
        for primitive in self._seq:
            # If we match the one we've clicked on, select it.
            if primitive.get_primitive_number() == primitive_number:
                primitive.set_control_visible(is_selected)
                primitive.update_viz()
            else:
                # Otherwise, deselect it.
                if primitive.is_control_visible():
                    primitive.set_control_visible(False)
                    primitive.update_viz()

        # If we selected it, really click on it.
        if is_selected:
            self._primitive_click_cb(primitive_number)
        self._lock.release()

    def select_primitive(self, primitive_number):
        '''Makes the interactive marker for the indicated primitive
        selected by showing the 6D controls.

        Args:
            primitive_number (int)
        '''
        self.marker_click_cb(primitive_number, True)

    def initialize_viz(self):
        '''Initialize visualization.'''

        rospy.loginfo("Initialising viz for: {}".format(self.get_action_id()))
        self._lock.acquire()
        for i in range(len(self._seq)):
            primitive = self._seq[i]

            # Construct the markers.
            primitive.make_marker(
                self.marker_click_cb,  # marker_click_cb
                self._delete_primitive,
            )

            self._update_links()

        self._update_markers()
        self._lock.release()

    def delete_last_primitive(self):
        '''Deletes the last primitive of the action.'''
        self._lock.acquire()
        self._delete_primitive(len(self._seq) - 1)
        self._lock.release()

    def is_object_required(self):
        '''Returns whether this action has any primitives that are relative
        to objects in the world (instead of absolute).

        Returns:
            bool
        '''
        is_required = False
        self._lock.acquire()
        for primitive in self._seq:

            is_required = primitive.is_object_required()
            if is_required:
                break
        self._lock.release()
        return is_required

    def get_ref_frame_names(self):
        '''Returns the names of the reference frame objects for all
        action primitives.

        Returns:
            [str]
        '''
        self._lock.acquire()
        ref_frame_names = []
        for primitive in self._seq:
            ref_frame_names += [primitive.get_ref_name()]
        self._lock.release()
        return ref_frame_names

    def get_primitive(self, index):
        '''Returns primitive of the action based on index.

        Args:
            index (int): Index (0-based) of primitive to return.

        Returns:
            Primitive|None: Returns None if no such step exists.
        '''
        # NOTE(mbforbes): For this lock to be meaningful, we have to
        # check that the index is valid within it.
        self._lock.acquire()
        n_primitives = len(self._seq)
        if index < 0 or index >= n_primitives:
            rospy.logerr("Requested primitive index " + str(index) +
                         ", but only have " + str(n_primitives) +
                         " primitives.")
            requested_primitive = None
        else:
            requested_primitive = self._seq[index]
        self._lock.release()
        return requested_primitive

    def update_viz(self):
        '''Updates the visualization of the action.'''
        self._lock.acquire()
        self._update_links()
        m_array = MarkerArray()
        for i in self._link_markers.keys():
            m_array.markers.append(self._link_markers[i])
        self._marker_publisher.publish(m_array)
        self._lock.release()

    def clear(self):
        '''Clears the action.'''
        self.reset_viz()
        self._lock.acquire()
        self._seq = []
        self._link_markers = dict()
        self._lock.release()

    # ##################################################################
    # Static methods: Internal ("private")
    # ##################################################################

    @staticmethod
    def _get_link(primitive0, primitive1, marker_id):
        '''Returns a marker representing a link b/w two consecutive
        action steps (both must already exist).

        Args:
            primitive0 (Primitive)
            primitive1 (Primitive)
            marker_id (int) : id for link marker between to primitives

        Returns:
            Marker|None
        '''
        start = primitive0.get_absolute_position(use_final=True)
        end = primitive1.get_absolute_position(use_final=False)
        if not start is None and not end is None:
            return Marker(type=Marker.ARROW,
                          id=marker_id,
                          lifetime=LINK_MARKER_LIFETIME,
                          scale=LINK_SCALE,
                          header=Header(frame_id=BASE_LINK),
                          color=LINK_COLOR,
                          points=[start, end])
        else:
            return None

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _is_condition_met(self, condition):
        '''Returns whether the given pre-condition or post-condition is
        currently met. Actually not implemented right now.

        Args:
            condition (Condition): The pre or post condition for the primitive

        Returns:
            bool: Whether the given pre/post-condition is met.
        '''
        # TODO(mcakmak): Implement.
        return True


    def _execute_action(self):
        ''' Function to replay the demonstrated action.'''
        self._status = ExecutionStatus.EXECUTING
        primitive = self.get_primitive(0)

        rospy.loginfo("Starting to execute action!!!")

        # Make sure the primitive exists.
        if primitive is None:
            rospy.logwarn("First primitive does not exist.")
            self._status = ExecutionStatus.CONDITION_ERROR
        # Check if the very first precondition is met.
        # Not actually implemented right now.
        elif not self._is_condition_met(primitive.get_pre_condition()):
            rospy.logwarn(
                'First precond is not met, first make sure the robot is' +
                'ready to execute action (hand object or free hands).')
            self._status = ExecutionStatus.CONDITION_ERROR
        else:
            # Check that all parts of the action are reachable
            if not self._is_action_reachable():
                rospy.logwarn('Problem finding IK solutions.')
                self._status = ExecutionStatus.NO_IK
            else:
                self._loop_through_primitives()

            self._robot.reset_arm_movement_history()

            # If we haven't been preempted, we now report success.
            if self._status == ExecutionStatus.EXECUTING:
                self._status = ExecutionStatus.SUCCEEDED
                rospy.loginfo('Action execution has succeeded.')

    def _is_action_reachable(self):
        '''Make sure that action is possible to execute entire action'''
        for i in range(len(self._seq)):
            primitive = self.get_primitive(i)
            if primitive is None:
                rospy.logwarn("Primitive " + str(i) + " does not exist.")
                break
            else:
                if not primitive.is_reachable():
                    return False
        return True

    def _loop_through_primitives(self):
        '''Goes through the primitives of the current action and moves to
        each.
        '''
        # Go over primitives of the action
        for i in range(self.n_primitives()):
            rospy.loginfo('Executing primitive ' + str(i))
            primitive = self.get_primitive(i)

            # Make sure primitive exists.
            if primitive is None:
                rospy.logwarn("Primitive " + str(i) + " does not exist.")
                self._status = ExecutionStatus.CONDITION_ERROR
                break
            # Check that preconditions are met (doesn't do anything right now)
            elif not self._is_condition_met(primitive.get_pre_condition()):
                rospy.logwarn(
                    '\tPreconditions of primitive ' + str(i) + ' are not ' +
                    'satisfied. Aborting.')
                self._status = ExecutionStatus.CONDITION_ERROR
                break
            else:
                # Try executing.
                self._status = ExecutionStatus.EXECUTING
                if not primitive.execute():
                    self._status = ExecutionStatus.NO_IK
                    break
                else:
                    self._status = ExecutionStatus.SUCCEEDED

                # Finished executing; check that postconditions are met
                if self._is_condition_met(primitive.get_post_condition()):
                    rospy.loginfo('\tPost-conditions of the action are met.')
                else:
                    rospy.logwarn(
                        '\tPost-conditions of action primitive ' + str(i) +
                        ' are not satisfied. Aborting.')
                    self._status = ExecutionStatus.CONDITION_ERROR
                    break

            # Perhaps the execution was pre-empted by the user. Check
            # this before continuing onto the next primitive.
            if self._preempt:
                rospy.logwarn('\tExecution preempted by user.')
                self._status = ExecutionStatus.PREEMPTED
                break

            # Primitive completed successfully.
            rospy.loginfo('\tPrimitive ' + str(i) + ' of action is complete.')

    def _update_markers(self):
        '''Updates the markers after a change.'''
        for primitive in self._seq:
            primitive.update_viz()

    def _delete_primitive(self, to_delete):
        '''Deletes a primitive from the action.

        NOTE(mbforbes): The lock should be acquired before calling this
        method.

        Args:
            to_delete (int): The index of the primitive to delete.
        '''
        rospy.loginfo('Deleting step: ' + str(to_delete))

        self._seq[to_delete].delete_marker()
        for i in range(to_delete + 1, self.n_primitives()):
            self._seq[i].decrease_id()
        self._seq.pop(to_delete)

        self.update_viz()
        self._delete_primitive_cb()

    def _update_links(self):
        '''Updates the visualized links b/w action primitives.'''
        current_num_links = len(self._link_markers)
        new_num_links = len(self._seq) - 1

        self._link_markers = {}

        for i in range(new_num_links):
            link_marker = Action._get_link(self._seq[i],
                                           self._seq[i + 1],
                                           i)
            if not link_marker is None:
                self._link_markers[i] = link_marker
        if (current_num_links - new_num_links) > 0:
            for i in range(new_num_links, current_num_links):
                if i in self._link_markers:
                    self._link_markers[i].action = Marker.DELETE

