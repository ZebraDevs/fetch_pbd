#!/usr/bin/env python
'''Provides service interface to the robot's retrieve/update the robot's
knowledge of the world. 
'''
# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# Local
from fetch_pbd_interaction import World

# ######################################################################
# Main Function
# ######################################################################

if __name__ == '__main__':

    # Register as a ROS node.
    rospy.init_node('pbd_world', anonymous=True)

    world = World()

    while(not rospy.is_shutdown()):
        world.update()
        # This is the pause between update runs. Note that this doesn't
        # guarantee an update rate, only that there is this amount of
        # pause between udpates.
        rospy.sleep(0.1)
