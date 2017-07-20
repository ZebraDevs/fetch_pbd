#!/usr/bin/env python
'''This runs the "core" of the PbD backend. 
It handles the interactions between the GUI and the state of the world
and recorded actions. 
It coordinates responses from the head and arms of the robot. 
'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# Local
from fetch_pbd_interaction import Interaction

# ######################################################################
# Main Function
# ######################################################################

if __name__ == '__main__':

    # Register as a ROS node.
    rospy.init_node('interaction', anonymous=True)
    grasp_suggestion_service = rospy.get_param('~grasp_suggestion_service')
    external_ee_link = rospy.get_param('~grasp_suggestion_ee_link')

    # Run the system    
    interaction = Interaction(grasp_suggestion_service, external_ee_link)

    while not rospy.is_shutdown():
        interaction.update()

        # This is the pause between update runs. Note that this doesn't
        # guarantee an update rate, only that there is this amount of
        # pause between udpates.
        rospy.sleep(0.1)
