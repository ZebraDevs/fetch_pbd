from interactive_markers.menu_handler import MenuHandler
from fetch_pbd_interaction.msg import Landmark
import rospy


class WorldLandmark:
    '''Class for representing objects'''

    def __init__(self, pose, index, dimensions, is_recognized):
        '''
        Args:
            pose (Pose): Position of bounding box
            index (int): For naming object in world (e.g. "thing 0")
            dimensions (Vector3): Size of bounding box
            is_recognized (bool): Result of object recognition.
        '''
        self.index = index
        self.assigned_name = None
        self.is_recognized = is_recognized
        self.object = Landmark(Landmark.TABLE_TOP, self.get_name(), pose,
                               dimensions)
        self.menu_handler = MenuHandler()
        self.int_marker = None
        self.is_removed = False
        self.menu_handler.insert('Remove from scene', callback=self.remove)

    def get_name(self):
        '''Return this object's name.

        Returns:
            str
        '''
        if self.assigned_name is None:
            if self.is_recognized:
                return 'object ' + str(self.index)
            else:
                return 'thing ' + str(self.index)
        else:
            return self.assigned_name

    def remove(self, __):
        '''Function for removing object from the world.

        Args:
            __ (???): Unused
        '''
        rospy.loginfo('Will remove object: ' + self.get_name())
        self.is_removed = True

    # TODO(mbforbes): Re-implement object recognition or remove
    # this dead code.

    # def assign_name(self, name):
    #     '''Function for assigning a different name to this object.

    #     Args:
    #         name (str): The new name.
    #     '''
    #     self.assigned_name = name
    #     self.object.name = name

    # def decrease_index(self):
    #     '''Function to decrese object index.'''
    #     self.index -= 1
