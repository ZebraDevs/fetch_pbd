#!/usr/bin/env python

import roslib
roslib.load_manifest('fetch_social_gaze')
import rospy

from std_msgs.msg import String
import time, sys
from numpy import *
from timeit import Timer
import numpy.linalg
import actionlib
from scipy.ndimage.filters import *
from std_srvs.srv import Empty, EmptyResponse
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import *
#from fetch_mechanism_msgs.msg import *
#from fetch_mechanism_msgs.srv import *
#from fetch_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import *
from tf import TransformListener

from fetch_social_gaze.msg import *


class SocialGaze:

    gaze_goal_strs = {
        GazeGoal.LOOK_FORWARD: 'LOOK_FORWARD',
        GazeGoal.FOLLOW_EE: 'FOLLOW_EE',
        GazeGoal.GLANCE_EE: 'GLANCE_EE',
        GazeGoal.NOD: 'NOD',
        GazeGoal.SHAKE: 'SHAKE',
        GazeGoal.FOLLOW_FACE: 'FOLLOW_FACE',
        GazeGoal.LOOK_AT_POINT: 'LOOK_AT_POINT',
        GazeGoal.LOOK_DOWN: 'LOOK_DOWN',
        GazeGoal.NOD_ONCE: 'NOD_ONCE ',
        GazeGoal.SHAKE_ONCE: 'SHAKE_ONCE',
    }
    '''This maps gaze goal constants to human-readable forms.'''

    def __init__(self):
        self.defaultLookatPoint = Point(1, 0, 1.05)
        self.downLookatPoint = Point(0.5, 0, 0.5)
        self.targetLookatPoint = Point(1, 0, 1.05)
        self.currentLookatPoint = Point(1, 0, 1.05)

        self.currentGazeAction = GazeGoal.LOOK_FORWARD
        self.prevGazeAction = self.currentGazeAction
        self.prevTargetLookatPoint = array(self.defaultLookatPoint)

        # Some constants
        self.doNotInterrupt = [GazeGoal.NOD,
                               GazeGoal.SHAKE]
        self.nodPositions = [Point(1, 0, 0.80), Point(1, 0, 1.10)]
        self.shakePositions = [Point(1, 0.2, 1.05), Point(1, -0.2, 1.05)]
        self.nNods = 5
        self.nShakes = 5

        self.nodCounter = 5
        self.shakeCounter = 5
        self.facePos = None

        ## Action client for sending commands to the head.
        self.headActionClient = SimpleActionClient(
            '/head_controller/point_head', PointHeadAction)
        self.headActionClient.wait_for_server()
        rospy.loginfo('Head action client has responded.')

        self.headGoal = PointHeadGoal()
        self.headGoal.target.header.frame_id = 'base_link'
        self.headGoal.min_duration = rospy.Duration(1.0)
        self.headGoal.target.point = Point(1, 0, 1)

        ## Client for receiving detected faces
        #self.faceClient = SimpleActionClient('face_detector_action', FaceDetectorAction)

        ## Service client for arm states
        self.tfListener = TransformListener()

        ## Server for gaze requested gaze actions
        self.gazeActionServer = actionlib.SimpleActionServer(
            'gaze_action', GazeAction, self.executeGazeAction, False)
        self.gazeActionServer.start()

    ## Callback function to receive ee states and face location
    def getEEPos(self):

        fromFrame = '/base_link'
        toFrame = '/wrist_roll_link'

        try:
            t = self.tfListener.getLatestCommonTime(fromFrame, toFrame)
            (position, rotation) = self.tfListener.lookupTransform(fromFrame,
                                                                   toFrame, t)
        except:
            rospy.logerr('Could not get the end-effector pose.')
        #objPoseStamped = PoseStamped()
        #objPoseStamped.header.stamp = t
        #objPoseStamped.header.frame_id = '/base_link'
        #objPoseStamped.pose = Pose()
        #relEEPose = self.tfListener.transformPose(toFrame, objPoseStamped)
        return Point(position[0], position[1], position[2])

    def getFaceLocation(self):
        connected = self.faceClient.wait_for_server(rospy.Duration(1.0))
        if connected:
            fgoal = FaceDetectorGoal()
            self.faceClient.send_goal(fgoal)
            self.faceClient.wait_for_result()
            f = self.faceClient.get_result()
            ## If there is one face follow that one, if there are more than one, follow the closest one
            closest = -1
            if len(f.face_positions) == 1:
                closest = 0
            elif len(f.face_positions) > 0:
                closest_dist = 1000

            for i in range(len(f.face_positions)):
                dist = f.face_positions[i].pos.x * f.face_positions[i].pos.x + f.face_positions[i].pos.y * f.face_positions[i].pos.y + f.face_positions[i].pos.z * f.face_positions[i].pos.z
                if dist < closest_dist:
                    closest = i
                    closest_dist = dist

            if closest > -1:
                self.facePos = array([f.face_positions[closest].pos.x,
                                      f.face_positions[closest].pos.y,
                                      f.face_positions[closest].pos.z])
            else:
                rospy.logwarn('No faces were detected.')
                self.facePos = self.defaultLookatPoint
        else:
            rospy.logwarn(
                'Not connected to the face server, cannot follow faces.')
            self.facePos = self.defaultLookatPoint

    ## Callback function for receiving gaze commands
    def executeGazeAction(self, goal):
        rospy.loginfo("Got a head goal: {}".format(goal.action))
        command = goal.action
        if (self.doNotInterrupt.count(self.currentGazeAction) == 0):
            if (self.currentGazeAction != command or
                command == GazeGoal.LOOK_AT_POINT):
                self.isActionComplete = False
                if (command == GazeGoal.LOOK_FORWARD):
                    self.targetLookatPoint = self.defaultLookatPoint
                elif (command == GazeGoal.LOOK_DOWN):
                    self.targetLookatPoint = self.downLookatPoint
                elif (command == GazeGoal.NOD):
                    self.nNods = 5
                    self.startNod()
                elif (command == GazeGoal.SHAKE):
                    self.nShakes = 5
                    self.startShake()
                elif (command == GazeGoal.NOD_ONCE):
                    self.nNods = 5
                    self.startNod()
                elif (command == GazeGoal.SHAKE_ONCE):
                    self.nShakes = 5
                    self.startShake()
                elif (command == GazeGoal.GLANCE_EE):
                    self.startGlance()
                elif (command == GazeGoal.LOOK_AT_POINT):
                    self.targetLookatPoint = goal.point
                rospy.loginfo('\tSetting gaze action to: ' +
                              SocialGaze.gaze_goal_strs[command])
                self.currentGazeAction = command

                while (not self.isActionComplete):
                    time.sleep(0.1)
                self.currentGazeAction = None
                # Perturb the head goal so it gets updated in the update loop.
                self.headGoal.target.point.x += 1
                self.gazeActionServer.set_succeeded()
            else:
                self.gazeActionServer.set_aborted()
                rospy.logwarn("Aborting head goal because not right type")

        else:
            self.gazeActionServer.set_aborted()
            rospy.logwarn("Aborting head goal")

    def isTheSame(self, current, target):
        diff = target - current
        dist = linalg.norm(diff)
        return (dist < 0.0001)

    def filterLookatPosition(self, current, target):
        speed = 0.02
        diff = self.point2array(target) - self.point2array(current)
        dist = linalg.norm(diff)
        if (dist > speed):
            step = dist / speed
            return self.array2point(self.point2array(current) + diff / step)
        else:
            return target

    def startNod(self):
        self.prevTargetLookatPoint = self.targetLookatPoint
        self.prevGazeAction = str(self.currentGazeAction)
        self.nodCounter = 0
        self.targetLookatPoint = self.nodPositions[0]

    def startGlance(self):
        self.prevTargetLookatPoint = self.targetLookatPoint
        self.prevGazeAction = str(self.currentGazeAction)
        self.glanceCounter = 0
        self.targetLookatPoint = self.getEEPos()

    def startShake(self):
        self.prevTargetLookatPoint = self.targetLookatPoint
        self.prevGazeAction = str(self.currentGazeAction)
        self.shakeCounter = 0
        self.targetLookatPoint = self.shakePositions[0]

    def point2array(self, p):
        return array((p.x, p.y, p.z))

    def array2point(self, a):
        return Point(a[0], a[1], a[2])

    def getNextNodPoint(self, current, target):
        if (self.isTheSame(self.point2array(current),
                           self.point2array(target))):
            self.nodCounter += 1
            if (self.nodCounter == self.nNods):
                self.currentGazeAction = self.prevGazeAction
                return self.prevTargetLookatPoint
            else:
                return self.nodPositions[self.nodCounter % 2]
        else:
            return target

    def getNextGlancePoint(self, current, target):
        if (self.isTheSame(self.point2array(current),
                           self.point2array(target))):
            self.glanceCounter = 1
            self.currentGazeAction = self.prevGazeAction
            return self.prevTargetLookatPoint
        else:
            return target

    def getNextShakePoint(self, current, target):
        if (self.isTheSame(self.point2array(current),
                           self.point2array(target))):
            self.shakeCounter += 1
            if (self.shakeCounter == self.nNods):
                self.currentGazeAction = self.prevGazeAction
                return self.prevTargetLookatPoint
            else:
                return self.shakePositions[self.shakeCounter % 2]
        else:
            return target

    def update(self):

        isActionPossiblyComplete = True

        # rospy.loginfo("Current action: {}".format(self.currentGazeAction))
        if (self.currentGazeAction == GazeGoal.FOLLOW_EE):
            self.targetLookatPoint = self.getEEPos()

        elif (self.currentGazeAction == GazeGoal.FOLLOW_FACE):
            self.getFaceLocation()
            self.targetLookatPoint = self.facePos

        elif (self.currentGazeAction == GazeGoal.NOD):
            self.targetLookatPoint = self.getNextNodPoint(
                self.currentLookatPoint, self.targetLookatPoint)
            self.headGoal.min_duration = rospy.Duration(0.5)
            isActionPossiblyComplete = False

        elif (self.currentGazeAction == GazeGoal.SHAKE):
            self.targetLookatPoint = self.getNextShakePoint(
                self.currentLookatPoint, self.targetLookatPoint)
            self.headGoal.min_duration = rospy.Duration(0.5)
            isActionPossiblyComplete = False

        elif (self.currentGazeAction == GazeGoal.GLANCE_EE):
            self.targetLookatPoint = self.getNextGlancePoint(
                self.currentLookatPoint, self.targetLookatPoint)
            isActionPossiblyComplete = False

        self.currentLookatPoint = self.filterLookatPosition(
            self.currentLookatPoint, self.targetLookatPoint)
        if self.currentGazeAction is None:
            pass
        elif (self.isTheSame(self.point2array(self.headGoal.target.point),
                             self.point2array(self.targetLookatPoint))):
            if (isActionPossiblyComplete):
                # if (self.headActionClient.get_state() == GoalStatus.SUCCEEDED):
                self.isActionComplete = True
                #self.currentGazeAction = None
                # rospy.loginfo("Succeeded!!")
        else:
            self.headGoal.target.point.x = self.currentLookatPoint.x
            self.headGoal.target.point.y = self.currentLookatPoint.y
            self.headGoal.target.point.z = self.currentLookatPoint.z
            self.headActionClient.send_goal(self.headGoal)

        time.sleep(0.02)


if __name__ == '__main__':
    rospy.init_node('social_gaze')
    gaze = SocialGaze()
    while not rospy.is_shutdown():
        gaze.update()
