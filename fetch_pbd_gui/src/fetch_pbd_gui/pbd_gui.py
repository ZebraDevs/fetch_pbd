#!/usr/bin/env python

import roslib
roslib.load_manifest('fetch_pbd_gui')


import os
import time
from subprocess import call
import rospy, yaml
from std_msgs.msg import String
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import QWidget, QFrame
from python_qt_binding.QtGui import QGroupBox, QIcon, QTableView
from python_qt_binding.QtCore import Slot, qDebug, QSignalMapper, QTimer, qWarning, Signal
from fetch_pbd_speech_recognition.msg import Command
from fetch_pbd_interaction.msg import GuiCommand
from sound_play.msg import SoundRequest
from fetch_pbd_interaction.msg import ExperimentState
from fetch_pbd_interaction.srv import GetExperimentState


class ClickableLabel(QtGui.QLabel):
    def __init__(self, parent, index, clickCallback):
        QtGui.QLabel.__init__(self, parent)
        self.index = index
        self.clickCallback = clickCallback

    def mousePressEvent(self, event):
        self.emit(QtCore.SIGNAL('clicked()'), "Label pressed")
        self.clickCallback(self.index)


class ActionIcon(QtGui.QGridLayout):
    def __init__(self, parent, index, clickCallback):
        QtGui.QGridLayout.__init__(self)
        self.setSpacing(0)
        path = os.popen('rospack find fetch_pbd_gui').read()
        path = path[0:len(path)-1]
        self.notSelectedIconPath = path + '/icons/actions0.png'
        self.selectedIconPath = path + '/icons/actions1.png'
        self.selected = True
        self.actionIconWidth = 50
        self.index = index
        self.icon = ClickableLabel(parent, index, clickCallback)
        self.text = QtGui.QLabel(parent)
        self.text.setText(self.getName())
        self.updateView()
        self.addWidget(self.icon, 0, 0, QtCore.Qt.AlignCenter)
        self.addWidget(self.text, 1, 0, QtCore.Qt.AlignCenter)

    def getName(self):
        return 'Action' + str(self.index + 1)

    def updateView(self):
        if self.selected:
            pixmap = QtGui.QPixmap(self.selectedIconPath)
        else:
            pixmap = QtGui.QPixmap(self.notSelectedIconPath)
        self.icon.setPixmap(pixmap.scaledToWidth(self.actionIconWidth, QtCore.Qt.SmoothTransformation))


class PbDGUI(Plugin):

    exp_state_sig = Signal(ExperimentState)

    def __init__(self, context):
        super(PbDGUI, self).__init__(context)
        self.setObjectName('PbDGUI')
        self._widget = QWidget()

        self.speech_cmd_publisher = rospy.Publisher('recognized_command', 
                                                    Command,
                                                    queue_size=10)
        self.gui_cmd_publisher = rospy.Publisher('gui_command', 
                                                 GuiCommand,
                                                 queue_size=10)

        rospy.Subscriber('experiment_state', ExperimentState, self.exp_state_cb)
        rospy.Subscriber('robotsound', SoundRequest, self.robotSoundReceived)

        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        self.exp_state_sig.connect(self.update_state)

        self.commands = dict()
        self.commands[Command.CREATE_NEW_ACTION] = 'New action'
        self.commands[Command.TEST_MICROPHONE] = 'Test microphone'
        self.commands[Command.NEXT_ACTION] = 'Next action'
        self.commands[Command.PREV_ACTION] = 'Previous action'
        self.commands[Command.SAVE_POSE] = 'Save pose'
        self.commands[Command.OPEN_HAND] = 'Open hand'
        self.commands[Command.CLOSE_HAND] = 'Close hand'
        self.commands[Command.EXECUTE_ACTION] = 'Execute action'
        self.commands[Command.STOP_EXECUTION] = 'Stop execution'
        self.commands[Command.DELETE_ALL_STEPS] = 'Delete all'
        self.commands[Command.DELETE_LAST_STEP] = 'Delete last'
        self.commands[Command.RECORD_OBJECT_POSE] = 'Record object poses'

        self.currentAction = -1
        self.currentStep = -1
        self.current_ref_frames = []

        allWidgetsBox = QtGui.QVBoxLayout()
        actionBox = QGroupBox('Actions', self._widget)
        self.actionGrid = QtGui.QGridLayout()
        self.actionGrid.setHorizontalSpacing(0)
        for i in range(6):
            self.actionGrid.addItem(QtGui.QSpacerItem(90, 90), 0, i, QtCore.Qt.AlignCenter)
            self.actionGrid.setColumnStretch(i, 0)
        self.actionIcons = dict()
        actionBoxLayout = QtGui.QHBoxLayout()
        actionBoxLayout.addLayout(self.actionGrid)
        actionBox.setLayout(actionBoxLayout)

        actionButtonGrid = QtGui.QHBoxLayout()
        actionButtonGrid.addWidget(self.create_button(
                                        Command.CREATE_NEW_ACTION))
        self.stepsBox = QGroupBox('No actions created yet', self._widget)
        self.stepsGrid = QtGui.QGridLayout()

        self.model = QtGui.QStandardItemModel(self)
        self.view = self._create_table_view(self.model,
                                              self.row_clicked_cb)

        self.stepsGrid.addItem(QtGui.QSpacerItem(280, 10), 0, 0, 2, 3)
        self.stepsGrid.addItem(QtGui.QSpacerItem(10, 10), 0, 1, 2, 3)
        self.stepsGrid.addItem(QtGui.QSpacerItem(280, 10), 0, 2, 2, 3)

        self.stepsGrid.addWidget(QtGui.QLabel('Arm'), 0, 0)

        self.stepsGrid.addWidget(self.view, 1, 0)

        stepsBoxLayout = QtGui.QHBoxLayout()
        stepsBoxLayout.addLayout(self.stepsGrid)
        self.stepsBox.setLayout(stepsBoxLayout)

        stepsButtonGrid = QtGui.QHBoxLayout()
        stepsButtonGrid.addWidget(self.create_button(Command.SAVE_POSE))
        stepsButtonGrid.addWidget(self.create_button(Command.EXECUTE_ACTION))
        stepsButtonGrid.addWidget(self.create_button(Command.STOP_EXECUTION))
        stepsButtonGrid.addWidget(self.create_button(Command.DELETE_ALL_STEPS))
        stepsButtonGrid.addWidget(self.create_button(Command.DELETE_LAST_STEP))

        misc_grid = QtGui.QHBoxLayout()
        misc_grid.addWidget(self.create_button(Command.TEST_MICROPHONE))
        misc_grid.addWidget(self.create_button(Command.RECORD_OBJECT_POSE))
        misc_grid.addStretch(1)

        misc_grid3 = QtGui.QHBoxLayout()
        misc_grid3.addWidget(self.create_button(Command.OPEN_HAND))
        misc_grid3.addWidget(self.create_button(Command.CLOSE_HAND))
        misc_grid3.addStretch(1)

        misc_grid4 = QtGui.QHBoxLayout()
        misc_grid4.addWidget(self.create_button(Command.PREV_ACTION))
        misc_grid4.addWidget(self.create_button(Command.NEXT_ACTION))
        misc_grid4.addStretch(1)

        speechGroupBox = QGroupBox('Robot Speech', self._widget)
        speechGroupBox.setObjectName('RobotSpeechGroup')
        speechBox = QtGui.QHBoxLayout()
        self.speechLabel = QtGui.QLabel('Robot has not spoken yet')
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.blue)
        self.speechLabel.setPalette(palette)
        speechBox.addWidget(self.speechLabel)
        speechGroupBox.setLayout(speechBox)

        allWidgetsBox.addWidget(actionBox)
        allWidgetsBox.addLayout(actionButtonGrid)

        allWidgetsBox.addWidget(self.stepsBox)
        allWidgetsBox.addLayout(stepsButtonGrid)

        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        allWidgetsBox.addLayout(misc_grid)
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        allWidgetsBox.addLayout(misc_grid3)
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        allWidgetsBox.addLayout(misc_grid4)
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))

        allWidgetsBox.addWidget(speechGroupBox)
        allWidgetsBox.addStretch(1)

        # Fix layout and add main widget to the user interface
        QtGui.QApplication.setStyle(QtGui.QStyleFactory.create('plastique'))
        vAllBox = QtGui.QVBoxLayout()
        vAllBox.addLayout(allWidgetsBox)
        vAllBox.addStretch(1)
        hAllBox = QtGui.QHBoxLayout()
        hAllBox.addLayout(vAllBox)
        hAllBox.addStretch(1)

        self._widget.setObjectName('PbDGUI')
        self._widget.setLayout(hAllBox)
        context.add_widget(self._widget)

        rospy.loginfo('Will wait for the experiment state service...')
        rospy.wait_for_service('get_experiment_state')
        exp_state_srv = rospy.ServiceProxy('get_experiment_state',
                                                 GetExperimentState)
        rospy.loginfo('Got response from the experiment state service...')

        response = exp_state_srv()
        self.update_state(response.state)

    @staticmethod
    def loginfo(message):
        '''Because all other ROS logging has some kind of information
        about what's being emitted, and qWarning doesn't, we're going to
        wrap the function to provide a little bit of additional info for
        newbies.

        Args:
            message (str): The message to log
        '''
        qWarning('[INFO] [pbd_gui.py] ' + message)

    def _create_table_view(self, model, row_click_cb):
        proxy = QtGui.QSortFilterProxyModel(self)
        proxy.setSourceModel(model)
        view = QtGui.QTableView(self._widget)
        verticalHeader = view.verticalHeader()
        verticalHeader.sectionClicked.connect(row_click_cb)
        view.setModel(proxy)
        view.setMaximumWidth(250)
        view.setSortingEnabled(False)
        view.setCornerButtonEnabled(False)
        return view

    def get_uid(self, arm_index, index):
        '''Returns a unique id of the marker'''
        return (2 * (index + 1) + arm_index)

    def get_index(self, uid):
        '''Returns a unique id of the marker'''
        arm_index = uid % 2
        index = (uid - arm_index) / 2
        return index - 1

    def row_clicked_cb(self, logicalIndex):
        self.step_pressed(self.get_uid(0, logicalIndex))

    def create_button(self, command):
        btn = QtGui.QPushButton(self.commands[command], self._widget)
        btn.clicked.connect(self.command_cb)
        return btn

    def update_state(self, state):
        PbDGUI.loginfo('Received new state')        

        self.current_ref_frames = state.ref_frames

        n_actions = len(self.actionIcons.keys())
        if n_actions < state.n_actions:
            for i in range(n_actions, state.n_actions):
                self.new_action()

        if (self.currentAction != (state.i_current_action - 1)):
            self.delete_all_steps()
            self.action_pressed(state.i_current_action - 1, False)

        n_steps = self.n_steps()
        if (n_steps < state.n_steps):
            for i in range(n_steps, state.n_steps):
                self.save_pose()
        elif (n_steps > state.n_steps):
            n_to_remove = n_steps - state.n_steps
            self.model.invisibleRootItem().removeRows(state.n_steps,
                                                      n_to_remove)

        ## TODO: DEAL with the following arrays!!!
        state.gripper_states
        state.ref_frames
        state.objects

        if (self.currentStep != state.i_current_step):
            if (self.n_steps() > 0):
                self.currentStep = state.i_current_step
                index = self.get_index(self.currentStep)
                self.view.selectRow(index)

    def save_pose(self, actionIndex=None):
        nColumns = 9
        if actionIndex is None:
            actionIndex = self.currentAction
        stepIndex = self.n_steps(actionIndex)
        ref_frame = "base_link"
        rospy.loginfo("Step Index: {}".format(stepIndex))
        rospy.loginfo("Ref frames: {}".format(self.current_ref_frames))
        if self.current_ref_frames:
            if self.current_ref_frames[stepIndex] != '':
                ref_frame = self.current_ref_frames[stepIndex]

        step = [QtGui.QStandardItem('Step' + str(stepIndex + 1)),
                    QtGui.QStandardItem('Go to pose'),
                    QtGui.QStandardItem(ref_frame)]
        self.model.invisibleRootItem().appendRow(step)
        self.update_table_view()
        self.currentStep = stepIndex

    def update_table_view(self):
        self.view.setColumnWidth(0, 50)
        self.view.setColumnWidth(1, 100)
        self.view.setColumnWidth(2, 70)

    def n_steps(self, actionIndex=None):
        return self.model.invisibleRootItem().rowCount()

    def delete_all_steps(self, actionIndex=None):
        if actionIndex is None:
            actionIndex = self.currentAction
        n_steps = self.n_steps()
        if (n_steps > 0):
            self.model.invisibleRootItem().removeRows(0, n_steps)

    def n_actions(self):
        return len(self.actionIcons.keys())

    def new_action(self):
        nColumns = 6
        actionIndex = self.n_actions()
        for key in self.actionIcons.keys():
             self.actionIcons[key].selected = False
             self.actionIcons[key].updateView()
        actIcon = ActionIcon(self._widget, actionIndex, self.action_pressed)
        self.actionGrid.addLayout(actIcon, int(actionIndex/nColumns),
                                  actionIndex%nColumns)
        self.actionIcons[actionIndex] = actIcon

    def step_pressed(self, step_index):
        gui_cmd = GuiCommand(GuiCommand.SELECT_ACTION_STEP, str(step_index))
        self.gui_cmd_publisher.publish(gui_cmd)

    def action_pressed(self, actionIndex, isPublish=True):
        for i in range(len(self.actionIcons.keys())):
            key = self.actionIcons.keys()[i]
            if key == actionIndex:
                 self.actionIcons[key].selected = True
            else:
                 self.actionIcons[key].selected = False
            self.actionIcons[key].updateView()
        self.currentAction = actionIndex
        self.stepsBox.setTitle('Steps for Action ' + str(self.currentAction+1))
        if isPublish:
            gui_cmd = GuiCommand(GuiCommand.SWITCH_TO_ACTION, str(actionIndex+1))
            self.gui_cmd_publisher.publish(gui_cmd)

    def command_cb(self):
        clickedButtonName = self._widget.sender().text()
        for key in self.commands.keys():
            if (self.commands[key] == clickedButtonName):
                PbDGUI.loginfo('Sending speech command: '+ key)
                command = Command()
                command.command = key
                self.speech_cmd_publisher.publish(command)

    def robotSoundReceived(self, soundReq):
        if (soundReq.command == SoundRequest.SAY):
            PbDGUI.loginfo('Robot said: ' + soundReq.arg)
            self.speechLabel.setText('Robot sound: ' + soundReq.arg)

    def exp_state_cb(self, state):
        self.exp_state_sig.emit(state)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.speech_cmd_publisher.unregister()
        self.gui_cmd_publisher.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

