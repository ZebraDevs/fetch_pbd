#!/usr/bin/env python

'''Helper script for running PbD end-to-end tests on the real robot.

- Running: robot (start up PbD):
    - $ robot claim; robot start
    - $ roslaunch pr2_pbd_interaction pbd_backend.launch coverage:=true

- Running: desktop (run this file):
    - $ realrobot  # point ROS to c1
    - $ roscd pr2_pbd_interaction; ./test/test_endtoend_realrobot.py

Note that this file can probably also be run on the real robot. (Just
run the last desktop command on c1 instead of the desktop.)
'''

import rospy
from unittest import main


if __name__ == '__main__':
    rospy.init_node('test_endtoend_realrobot')
    main('test_endtoend')
