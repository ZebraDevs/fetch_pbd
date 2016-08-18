'''The in-program representation of a programmed action.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# System builtins
import threading

# ROS builtins
from geometry_msgs.msg import Vector3, PoseStamped, Quaternion
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
import tf

# Local
from fetch_pbd_interaction.arm_target import ArmTarget
from fetch_pbd_interaction.arm_trajectory import ArmTrajectory
from fetch_pbd_interaction.msg import ExecutionStatus, OrientationRPY, \
                                      ArmState, Landmark

# ######################################################################
# Module level constants
# ######################################################################

# Marker properties for little arrows drawn between consecutive primitives.
LINK_MARKER_LIFETIME = rospy.Duration()
LINK_SCALE = Vector3(0.01, 0.03, 0.03)
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
                 action_change_cb, action_id=None):
        '''
        Args:
            robot (Robot) : interface to lower level robot functionality
            tf_listener (TransformListener)
            im_server (InteractiveMarkerSerever)
            primitive_click_cb (function(int)): The function to call when a
                primitive is clicked on (normally in the GUI). The function
                should take the number of the primitive
            action_id (int, optional): The index of this action.

        '''
        # Initialize a bunch of state.
        self._name = ''  # Human-friendly name for this action.
        self._im_server = im_server
        self._seq = []
        self._action_id = action_id
        self._robot = robot
        self._primitive_click_cb = primitive_click_cb
        self._action_change_cb = action_change_cb
        self._status = ExecutionStatus.NOT_EXECUTING
        self._preempt = False
        self._z_offset = 0.0
        self._tf_listener = tf_listener
        self._primitive_counter = 0
        # self._marker_visibility = []

        # Markers to connect consecutive primitives together
        self._link_markers = {}

        # TODO(sarah): Understand this note better
        # NOTE(mbforbes): It appears that this is locking manipulation
        # of the internal sequence (self._seq). There have been race
        # conditions involving this (e.g. marker_click_cb(...)).
        #
        # In general, be aware the other code calling these methods
        # with data about this class (like how many primitives it holds)
        # is bad because that means the outside code is assuming that it
        # knows about state internal to this class, and that information
        # may not be true by the time the code here gets executed. This
        # is because there are several callbacks that trigger here so
        # we must reason asyncronously.
        #
        # Unless the information you have (e.g. about the number of
        # primitives that exist) was learned while this lock was acquired,
        # you cannot assume it is true.
        self._lock = threading.Lock()

        if Action._marker_publisher is None:
            Action._marker_publisher = rospy.Publisher(TOPIC_MARKERS,
                                                                 MarkerArray,
                                                                 queue_size=10,
                                                                 latch=True)


    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def get_action_id(self):
        ''' Returns action_id

        Returns:
            int
        '''
        return self._action_id

    def set_action_id(self, action_id):
        ''' Returns action_id

        Args:
            action_id (int)
        '''
        self._action_id = action_id

    def set_name(self, name):
        '''Sets human-readable name for action

        Args:
            name (string)
        '''
        self._name = name

    def get_name(self):
        '''Returns human-readable name for action

        Returns
            (string)
        '''
        return self._name

    def get_json(self):
        '''Return json for this action for saving to db

        Returns:
            dict
        '''
        json = {}
        json['name'] = self._name
        json['primitive_counter'] = self._primitive_counter
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
        self._primitive_counter = json['primitive_counter']
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

            self.add_primitive(primitive, False, False)

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


    def add_primitive(self, primitive, add_marker=True, add_name=True):
        '''Add primitive to action.

        Args:
            primitive (Primitive)
            add_marker (bool)
            add_name (bool)
        '''
        self._lock.acquire()
        rospy.loginfo("Adding primitive")
        if add_name:
            primitive.set_name("primitive_" + str(self._primitive_counter))
            self._primitive_counter += 1
        primitive.add_marker_callbacks(
                self.select_primitive,  # marker_click_cb
                self.delete_primitive,
                self._primitive_pose_change,
                self._action_change_cb
            )
        self._seq.append(primitive)

        if add_marker:

            # self._marker_visibility.append(True)
            primitive.show_marker()

            self._update_links()

            self._update_markers()
            self._lock.release()
            self.update_viz()
        else:
            # self._marker_visibility.append(False)
            primitive.hide_marker()
            self._lock.release()
        # rospy.loginfo("marker viz: {}".format(self._marker_visibility))

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

        # Destroy the primitive markers.
        for primitive in self._seq:
            rospy.loginfo("Deleting marker")
            primitive.hide_marker()
        self._im_server.clear()
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

    def delete_primitive_marker(self, primitive_number):
        '''Delete marker with certain index

        Args:
            primitive_number (int)
        '''
        # self._marker_visibility[primitive_number] = False
        if self.n_primitives() > 0:
            primitive = self._seq[primitive_number]
            primitive.hide_marker()

    def make_primitive_marker(self, primitive_number):
        '''Delete marker with certain index

        Args:
            primitive_number (int)
        '''
        # self._marker_visibility[primitive_number] = True
        primitive = self._seq[primitive_number]
        primitive.show_marker()

    def get_marker_visibility(self):
        '''Returns visibility status of primitive markers

        Returns:
            [bool]
        '''
        marker_visibility = []
        for primitive in self._seq:
            marker_visibility += [primitive.marker_visible()]
        return marker_visibility

    def select_primitive(self, primitive_number, is_selected):
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
                primitive.select(is_selected)
                primitive.update_viz()
            else:
                # Otherwise, deselect it.
                if primitive.is_control_visible():
                    primitive.select(False)
                    primitive.update_viz()

        # If we selected it, really click on it.
        if is_selected:
            self._primitive_click_cb(primitive_number)
        else:
            self._primitive_click_cb(-1)
        self._lock.release()
        self.update_viz()

    def initialize_viz(self):
        '''Initialize visualization.'''

        rospy.loginfo("Initialising viz for: {}".format(self.get_action_id()))
        # self._lock.acquire()
        # self._marker_visibility = [True] * len(self._seq)
        for i in range(len(self._seq)):
            primitive = self._seq[i]

            # Construct the markers.
            primitive.show_marker()

            self._update_links()

        self._update_markers()
        # self._lock.release()
        self.update_viz()

    def delete_last_primitive(self):
        '''Deletes the last primitive of the action.'''
        self.delete_primitive(len(self._seq) - 1)

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
            ref_frame_names += [primitive.get_ref_frame_name()]
        self._lock.release()
        return ref_frame_names

    def get_primitive_positions_orientations(self):
        '''Returns the positions and orientations of primitives

        Returns:
            Point[], OrientationRPY[]
        '''
        self._lock.acquire()
        positions = []
        orientations = []
        for primitive in self._seq:
            pose = primitive.get_relative_pose()
            quaternion = (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            rpy = OrientationRPY(euler[0], euler[1], euler[2])
            positions += [pose.pose.position]
            orientations += [rpy]
        self._lock.release()
        return positions, orientations

    def get_primitive_names(self):
        '''Returns the names of primitives.

        Returns:
            [str]
        '''
        self._lock.acquire()
        names = []
        for primitive in self._seq:
            names += [primitive.get_name()]
        self._lock.release()
        return names

    def get_primitives_editable(self):
        '''Returns list of whether primitive poses are editable

        Returns:
            [bool]
        '''
        self._lock.acquire()
        editable = []
        for primitive in self._seq:
            editable += [primitive.pose_editable()]
        self._lock.release()
        return editable

    def update_primitive_pose(self, primitive_number, position, orientation):
        '''Update pose of primitive given by primitive_number

        Args:
            primitive_number (int)
            position (Point)
            orientation (OrientationRPY)
        '''

        frame_id = self.get_ref_frame_names()[primitive_number]
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose.position = position
        roll = orientation.r
        pitch = orientation.p
        yaw = orientation.y
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        pose_stamped.pose.orientation = Quaternion(quat[0],
                                                   quat[1],
                                                   quat[2],
                                                   quat[3])
        primitive = self._seq[primitive_number]
        primitive.set_pose(pose_stamped)
        self._primitive_pose_change()

    def get_primitives(self):
        '''Return list of primitives

        Returns:
            [Primitive]
        '''
        self._lock.acquire()
        primitives = self._seq
        self._lock.release()
        return primitives

    def get_primitive(self, index):
        '''Returns primitive of the action based on index.

        Args:
            index (int): Index (0-based) of primitive to return.

        Returns:
            Primitive|None: Returns None if no such primitive exists.
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
        # rospy.loginfo("link markers: {}".format(self._link_markers))
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

    def decrease_id(self):
        '''Decrement the action's id by one'''
        self._action_id = self._action_id - 1

    def switch_primitive_order(self, old_index, new_index):
        '''Change the order of primitives in action

        Args:
            old_index (int)
            new_index (int)
        '''
        self._lock.acquire()
        primitive = self._seq.pop(old_index)
        self._seq.insert(new_index, primitive)
        self._lock.release()
        self.update_viz()

    def delete_primitive(self, to_delete):
        '''Deletes a primitive from the action.

        NOTE(mbforbes): The lock should be acquired before calling this
        method.

        Args:
            to_delete (int): The index of the primitive to delete.
        '''
        self._lock.acquire()
        if (to_delete + 1) < self.n_primitives():
            rospy.loginfo("Abs pose: {}".format(self._seq[to_delete + 1].get_absolute_pose()))
        self._seq[to_delete].hide_marker()
        for i in range(to_delete + 1, self.n_primitives()):
            rospy.loginfo("Frame name: {}, {}, {}".format(i, self._seq[i].get_ref_frame_name(), self._seq[i]._number))

            self._seq[i].decrease_id()
        rospy.loginfo("Got here 1")

        if self.n_primitives() > (to_delete + 1):
            next_primitive = self._seq[to_delete + 1]
            if next_primitive.get_ref_type() == ArmState.PREVIOUS_TARGET:
                if to_delete == 0:
                    next_primitive.change_ref_frame(ArmState.ROBOT_BASE,
                                                    Landmark())
                else:
                    pose = next_primitive.get_absolute_pose()
                    rospy.loginfo("Abs pose: {}".format(pose))
                    rospy.loginfo("Frame name: {}, {}".format(next_primitive.get_ref_frame_name(), next_primitive._number))


                    new_pose = self._tf_listener.transformPose(
                        next_primitive.get_ref_frame_name(),
                        pose)
                    next_primitive.set_pose(new_pose)
                    rospy.loginfo("Got here 2")
        self._seq.pop(to_delete)
        # self._marker_visibility.pop(to_delete)
        self._update_links()
        self._lock.release()
        self.update_viz()
        rospy.loginfo("Got here first")

        self._action_change_cb()

    def execute_primitive(self, to_execute):
        '''Execute specified primitive

        Args:
            to_execute (int)
        '''
        self._seq[to_execute].execute()


    # ##################################################################
    # Static methods: Internal ("private")
    # ##################################################################

    @staticmethod
    def _get_link(primitive0, primitive1, marker_id):
        '''Returns a marker representing a link b/w two consecutive
        primitives (both must already exist).

        Args:
            primitive0 (Primitive)
            primitive1 (Primitive)
            marker_id (int) : id for link marker between to primitives

        Returns:
            Marker|None
        '''
        start = primitive0.get_absolute_marker_position(use_final=True)
        end = primitive1.get_absolute_marker_position(use_final=False)
        if start == end:
            return None
        elif not start is None and not end is None:
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

    def _primitive_pose_change(self):
        '''Update links when primitive pose changes'''
        # self._lock.acquire()
        for idx, primitive in enumerate(self._seq):
            primitive.update_viz()
        # self._lock.release()
        self.update_viz()

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
        for idx, primitive in enumerate(self._seq):
            primitive.update_viz()

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
                self._link_markers[i] = Marker(id=i, action=Marker.DELETE)

        if new_num_links == 0:
            marker = Action._get_link(self._seq[0],
                                           self._seq[0], 0)

            if not marker is None:
                self._link_markers[0] = marker
                self._link_markers[0].action = Marker.DELETE

