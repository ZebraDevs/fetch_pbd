'''Class for representing objects'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# ROS builtins
from interactive_markers.menu_handler import MenuHandler

# Local
from fetch_pbd_interaction.msg import Landmark

# ######################################################################
# Classes
# ######################################################################

class WorldLandmark:
    '''Class for representing objects'''

    def __init__(self, remove_object_cb, generate_grasps_cb, pose, index, dimensions,
                 is_recognized, point_cloud):
        '''
        Args:
            remove_object_cb (callback) : execute when removing objects
            pose (Pose): Position of bounding box
            index (int): For naming object in world (e.g. "thing 0")
            dimensions (Vector3): Size of bounding box
            is_recognized (bool): Result of object recognition.
        '''

        self._remove_object_cb = remove_object_cb
        self._generate_grasps_cb = generate_grasps_cb
        self._point_cloud = point_cloud
        self._assigned_name = None

        self.index = index
        self.is_recognized = is_recognized
        self.object = Landmark(self.get_name(), pose,
                               dimensions, point_cloud)
        self.menu_handler = MenuHandler()
        self.int_marker = None
        self.is_removed = False
        self.menu_handler.insert('Remove from scene', callback=self.remove)
        self.menu_handler.insert('Generate grasps for object', callback=self.get_grasps)

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def get_name(self):
        '''Return this object's name.

        Returns:
            str
        '''
        if self._assigned_name is None:
            if self.is_recognized:
                return 'object ' + str(self.index)
            else:
                return 'thing ' + str(self.index)
        else:
            return self._assigned_name

    def remove(self, feedback):
        '''Function for removing object from the world.

        Args:
            feedback (InteractiveMarkerFeedback): Unused
        '''
        rospy.loginfo('Will remove object: ' + self.get_name())
        self._remove_object_cb(self.object.name)

    def get_grasps(self, feedback):
        '''Function to trigger generation of grasps for the object

        Args:
            feedback (InteractiveMarkerFeedback): Unused
        '''
        rospy.loginfo('Generating grasps for object: ' + self.get_name())
        self._generate_grasps_cb(self.object.name)

    # TODO(mbforbes): Re-implement object recognition or remove
    # this dead code.

    # def assign_name(self, name):
    #     '''Function for assigning a different name to this object.

    #     Args:
    #         name (str): The new name.
    #     '''
    #     self._assigned_name = name
    #     self.object.name = name

    # def decrease_index(self):
    #     '''Function to decrese object index.'''
    #     self.index -= 1
