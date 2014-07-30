#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_pbd_gui')


import os
import time
from subprocess import call
import rospy, yaml
from std_msgs.msg import String
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import QWidget, QFrame
from python_qt_binding.QtGui import QGroupBox, QIcon, QTableView, QLineEdit
from python_qt_binding.QtCore import Slot, qDebug, QSignalMapper, QTimer, qWarning, Signal
from pr2_pbd_speech_recognition.msg import Command
from pr2_pbd_interaction.msg import GuiCommand
from sound_play.msg import SoundRequest
from pr2_pbd_interaction.msg import ExperimentState, ActionStep
from pr2_pbd_interaction.srv import GetExperimentState


class ClickableLabel(QtGui.QLabel):
    def __init__(self, parent, index, clickCallback):
        QtGui.QLabel.__init__(self, parent)
        self.index = index
        self.clickCallback = clickCallback
    
    def mousePressEvent(self, event):
        self.emit(QtCore.SIGNAL('clicked()'), "Label pressed")
        self.clickCallback(self.index)


class ActionIcon(QtGui.QGridLayout):
    def __init__(self, parent, index, name, clickCallback, is_experiment=False):
        QtGui.QGridLayout.__init__(self)
        self.setSpacing(0)
        path = os.popen('rospack find pr2_pbd_gui').read()
        path = path[0:len(path)-1]
        self.notSelectedIconPath = path + '/icons/actions0.png'
        self.selectedIconPath = path + '/icons/actions1.png'
        self.notSelectedIconPathExp = path + '/icons/experiments0.png'
        self.selectedIconPathExp = path + '/icons/experiments1.png'
        self.is_experiment = is_experiment
        self.selected = True
        self.actionIconWidth = 50
        self.index = index
        self.icon = ClickableLabel(parent, index, clickCallback)
        self.text = QtGui.QLabel(parent)
        self.text.setText(name)
        self.updateView()
        self.addWidget(self.icon, 0, 0, QtCore.Qt.AlignCenter)
        self.addWidget(self.text, 1, 0, QtCore.Qt.AlignCenter)
    
    def updateView(self):
        if self.selected:
            if self.is_experiment:
                pixmap = QtGui.QPixmap(self.selectedIconPathExp)
            else:
                pixmap = QtGui.QPixmap(self.selectedIconPath)
        else:
            if self.is_experiment:
                pixmap = QtGui.QPixmap(self.notSelectedIconPathExp)
            else:
                pixmap = QtGui.QPixmap(self.notSelectedIconPath)
        self.icon.setPixmap(pixmap.scaledToWidth(self.actionIconWidth, QtCore.Qt.SmoothTransformation))

    def removeView(self):
        self.icon.hide()
        self.text.hide()
        self.removeWidget(self.icon)
        self.removeWidget(self.text)
        self.setParent(None)


class PbDGUI(Plugin):

    exp_state_sig = Signal(ExperimentState)

    def __init__(self, context):
        super(PbDGUI, self).__init__(context)
        self.setObjectName('PbDGUI')
        self._widget = QWidget()
        
        self.speech_cmd_publisher = rospy.Publisher('recognized_command', Command)
        self.gui_cmd_publisher = rospy.Publisher('gui_command', GuiCommand)
        
        rospy.Subscriber('experiment_state', ExperimentState, self.exp_state_cb)
        
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        self.exp_state_sig.connect(self.update_state)
        
        self.commands = dict()
        self.commands[Command.TEST_MICROPHONE] = 'Test microphone'
        self.commands[Command.START_RECORDING] = 'Start recording'
        self.commands[Command.STOP_RECORDING] = 'Stop recording'
        self.commands[Command.REPLAY_DEMONSTRATION] = 'Replay demonstration'
        self.commands[Command.TAKE_TOOL] = 'Take tool'
        self.commands[Command.DETECT_SURFACE] = 'Detect surface'
        self.commands[Command.RELEASE_TOOL] = 'Release tool'
        
        self.currentAction = -1
        self.currentStep = -1
        self.current_experiment = -1


        allWidgetsBox = QtGui.QVBoxLayout()
        experimentBox = QGroupBox('Experiments', self._widget)
        self.experimentGrid = QtGui.QGridLayout()
        self.experimentGrid.setHorizontalSpacing(0)
        for i in range(6):
            self.experimentGrid.addItem(QtGui.QSpacerItem(90, 20), 0, i, QtCore.Qt.AlignCenter)
            self.experimentGrid.setColumnStretch(i, 0)
        experimentBoxLayout = QtGui.QHBoxLayout()
        experimentBoxLayout.addLayout(self.experimentGrid)
        self.experimentIcons = dict()
        experimentBox.setLayout(experimentBoxLayout)

        self.actionBox = QGroupBox('Demonstrations for current experiment', self._widget)
        self.actionGrid = QtGui.QGridLayout()
        self.actionGrid.setHorizontalSpacing(0)
        for i in range(6):
        #    self.actionGrid.addItem(QtGui.QSpacerItem(90, 90), 0, i, QtCore.Qt.AlignCenter)
            self.actionGrid.setColumnStretch(i, 0)
        self.actionIcons = dict()
        actionBoxLayout = QtGui.QHBoxLayout()
        actionBoxLayout.addLayout(self.actionGrid)
        self.actionBox.setLayout(actionBoxLayout)
        
        actionButtonGrid = QtGui.QHBoxLayout()
        self.stepsBox = QGroupBox('No demonstrations', self._widget)
        self.stepsGrid = QtGui.QGridLayout()
        
        self.l_model = QtGui.QStandardItemModel(self)
        self.l_view = self._create_table_view(self.l_model,
                                              self.l_row_clicked_cb)
        self.r_model = QtGui.QStandardItemModel(self)
        self.r_view = self._create_table_view(self.r_model,
                                              self.r_row_clicked_cb)

        self.stepsGrid.addItem(QtGui.QSpacerItem(280, 10), 0, 0, 2, 3)
        self.stepsGrid.addItem(QtGui.QSpacerItem(10, 10), 0, 1, 2, 3)
        self.stepsGrid.addItem(QtGui.QSpacerItem(280, 10), 0, 2, 2, 3)
        
        self.stepsGrid.addWidget(QtGui.QLabel('Left Arm'), 0, 0)
        self.stepsGrid.addWidget(QtGui.QLabel('Right Arm'), 0, 2)

        self.stepsGrid.addWidget(self.l_view, 1, 0)
        self.stepsGrid.addWidget(self.r_view, 1, 2)
        
        stepsBoxLayout = QtGui.QHBoxLayout()
        stepsBoxLayout.addLayout(self.stepsGrid)
        self.stepsBox.setLayout(stepsBoxLayout)

        stepsButtonGrid = QtGui.QHBoxLayout()
        stepsButtonGrid.addWidget(self.create_button(Command.REPLAY_DEMONSTRATION))
        
        motionButtonGrid = QtGui.QHBoxLayout()
        motionButtonGrid.addWidget(self.create_button(Command.START_RECORDING))
        motionButtonGrid.addWidget(self.create_button(Command.STOP_RECORDING))

        misc_grid = QtGui.QHBoxLayout()
        misc_grid.addWidget(self.create_button(Command.TAKE_TOOL))
        misc_grid.addWidget(self.create_button(Command.RELEASE_TOOL))
        misc_grid.addWidget(self.create_button(Command.DETECT_SURFACE))
        misc_grid.addWidget(self.create_button(Command.TEST_MICROPHONE))
        misc_grid.addStretch(1)
        
        misc_grid2 = QtGui.QHBoxLayout()
        self.tool_id_text = QtGui.QLineEdit()
        self.tool_id_text.setText("99")
        self.table_w_text = QtGui.QLineEdit()
        self.table_w_text.setText("0.4")
        self.table_h_text = QtGui.QLineEdit()
        self.table_h_text.setText("0.3")
        tool_btn = QtGui.QPushButton("Set fake tool ID", self._widget)
        tool_btn.clicked.connect(self.update_tool_id)
        table_btn = QtGui.QPushButton("Set fake table dimensions", self._widget)
        table_btn.clicked.connect(self.update_table_dimensions)
        misc_grid2.addWidget(self.tool_id_text)
        misc_grid2.addWidget(tool_btn)
        misc_grid2.addWidget(self.table_w_text)
        misc_grid2.addWidget(self.table_h_text)
        misc_grid2.addWidget(table_btn)
        misc_grid2.addStretch(1)
                
        misc_grid3 = QtGui.QHBoxLayout()
        exp_btn = QtGui.QPushButton("New experiment", self._widget)
        exp_btn.clicked.connect(self.new_experiment_command)
        misc_grid3.addWidget(exp_btn)
        misc_grid3.addStretch(1)

        misc_grid4 = QtGui.QHBoxLayout()
        c_btn = QtGui.QPushButton("Compute clusters", self._widget)
        c_btn.clicked.connect(self.compute_clusters_command)
        t_btn = QtGui.QPushButton("Compute trajectory", self._widget)
        t_btn.clicked.connect(self.compute_trajectory_command)
        misc_grid4.addWidget(c_btn)
        misc_grid4.addWidget(t_btn)
        misc_grid4.addStretch(1)

        speechGroupBox = QGroupBox('Interaction State', self._widget)
        speechGroupBox.setObjectName('RobotSpeechGroup')
        speechBox = QtGui.QHBoxLayout()
        self.stateLabel = QtGui.QLabel('Interaction state information not received yet')
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.blue)
        self.stateLabel.setPalette(palette)
        speechBox.addWidget(self.stateLabel)
        speechGroupBox.setLayout(speechBox)

        allWidgetsBox.addWidget(experimentBox)
        allWidgetsBox.addLayout(misc_grid3)
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        allWidgetsBox.addWidget(self.actionBox)
        allWidgetsBox.addLayout(actionButtonGrid)
        
        allWidgetsBox.addWidget(self.stepsBox)
        allWidgetsBox.addLayout(motionButtonGrid)
        allWidgetsBox.addLayout(stepsButtonGrid)
        
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        allWidgetsBox.addLayout(misc_grid)
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        allWidgetsBox.addLayout(misc_grid2)
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
        self.update_tool_id()
        self.update_table_dimensions()
        self.update_state(response.state)

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
    
    def update_tool_id(self):
        tool_id = self.tool_id_text.text()
        rospy.set_param('cleaning_tool_id', int(tool_id))

    def update_table_dimensions(self):
        table_w = self.table_w_text.text()
        table_h = self.table_h_text.text()
        rospy.set_param('table_w', float(table_w))
        rospy.set_param('table_h', float(table_h))

    def get_uid(self, arm_index, index):
        '''Returns a unique id of the marker'''
        return (2 * (index + 1) + arm_index)
    
    def get_arm_and_index(self, uid):
        '''Returns a unique id of the marker'''
        arm_index = uid % 2
        index = (uid - arm_index) / 2
        return (arm_index, (index - 1))

    def r_row_clicked_cb(self, logicalIndex):
        self.step_pressed(self.get_uid(0, logicalIndex))

    def l_row_clicked_cb(self, logicalIndex):
        self.step_pressed(self.get_uid(1, logicalIndex))

    def create_button(self, command):
        btn = QtGui.QPushButton(self.commands[command], self._widget)
        btn.clicked.connect(self.command_cb)
        return btn

    def update_state(self, state):
        qWarning('Received new state')
        
        nColumns = 6
        n_experiments = len(self.experimentIcons.keys())
        n_actions = len(self.actionIcons.keys())

        print 'n_actions', n_actions


        if (n_experiments < state.n_experiments):
            for i in range(n_experiments, state.n_experiments):
                self.new_experiment('experiment' + str(i))

        print 'self.current_experiment', self.current_experiment
        print 'state.i_current_experiment', state.i_current_experiment

        if self.current_experiment != state.i_current_experiment:
            # Clear all actions of this experiment

            for k in self.actionIcons.keys():
                
                #icon = self.actionIcons.pop(0)

                item = self.actionGrid.itemAtPosition(int(k/nColumns) + 1, k%nColumns)
                if item is not None:
                    item.removeView()
                    self.actionGrid.removeItem(item)
                    widget = item.widget()
                    if widget is not None:
                        self.actionGrid.removeWidget(widget)
                        widget.deleteLater()
                    else:
                        print 'widget none'
                    del item
                else:
                    print 'item none'
                
                #icon = self.actionGrid.takeAt(i + 6)
                #icon.remove()
                #del icon

            self.actionIcons = dict()
            self.experiment_pressed(state.i_current_experiment, False)
            
        n_actions = len(self.actionIcons.keys())
        if n_actions < state.n_actions:
            for i in range(n_actions, state.n_actions):
                self.new_action(state.action_names[i])

        self.action_pressed(state.i_current_action, False)

        if (self.currentAction != (state.i_current_action)):
            self.delete_all_steps()

        n_steps = self.n_steps()
        if (n_steps < state.n_steps):
            for i in range(n_steps, state.n_steps):
                self.save_pose(frameType=ord(state.frame_types[i]))
        elif (n_steps > state.n_steps):
            n_to_remove = n_steps - state.n_steps
            self.r_model.invisibleRootItem().removeRows(state.n_steps,
                                                      n_to_remove)
            self.l_model.invisibleRootItem().removeRows(state.n_steps,
                                                      n_to_remove)
        
        ## TODO: DEAL with the following arrays!!!
        state.r_gripper_states
        state.l_gripper_states
        state.r_ref_frames
        state.l_ref_frames
        state.objects

        self.stateLabel.setText(state.interaction_state)
            
        if (self.currentStep != state.i_current_step):
            if (self.n_steps() > 0):
                self.currentStep = state.i_current_step
                arm_index, index = self.get_arm_and_index(self.currentStep)
                if (arm_index == 0):
                    self.r_view.selectRow(index)
                else:
                    self.l_view.selectRow(index)

    def get_frame_type(self, fr_type):
        if (fr_type > 1):
            rospy.logwarn("Invalid frame type @ save_pose -> get_frame_type: " + str(fr_type))
        return ["Go to pose", "Maneuver"][fr_type]
    
    def save_pose(self, actionIndex=None, frameType=0):
        nColumns = 9
        if actionIndex is None:
            actionIndex = self.currentAction
        stepIndex = self.n_steps(actionIndex)
        
        r_step = [QtGui.QStandardItem('Step' + str(stepIndex + 1)),
                    QtGui.QStandardItem(self.get_frame_type(frameType)), 
                    QtGui.QStandardItem('Absolute')]
        l_step = [QtGui.QStandardItem('Step' + str(stepIndex + 1)),
                    QtGui.QStandardItem(self.get_frame_type(frameType)), 
                    QtGui.QStandardItem('Absolute')]
        self.r_model.invisibleRootItem().appendRow(r_step)
        self.l_model.invisibleRootItem().appendRow(l_step)
        self.update_table_view()
        self.currentStep = stepIndex
        
    def update_table_view(self):
        self.l_view.setColumnWidth(0, 50)
        self.l_view.setColumnWidth(1, 100)
        self.l_view.setColumnWidth(2, 70)
        self.r_view.setColumnWidth(0, 50)
        self.r_view.setColumnWidth(1, 100)
        self.r_view.setColumnWidth(2, 70)
        
    def n_steps(self, actionIndex=None):
        return self.l_model.invisibleRootItem().rowCount()
        
    def delete_all_steps(self, actionIndex=None):
        if actionIndex is None:
            actionIndex = self.currentAction
        n_steps = self.n_steps()
        if (n_steps > 0):
            self.l_model.invisibleRootItem().removeRows(0, n_steps)
            self.r_model.invisibleRootItem().removeRows(0, n_steps)

    def n_actions(self):
        return len(self.actionIcons.keys())

    def n_experiments(self):
        return len(self.experimentIcons.keys())

    def new_action(self, name):
        nColumns = 6
        actionIndex = self.n_actions()
        for key in self.actionIcons.keys():
             self.actionIcons[key].selected = False
             self.actionIcons[key].updateView()
        actIcon = ActionIcon(self._widget, actionIndex, name, self.action_pressed)

        #item = self.actionGrid.itemAtPosition(int(actionIndex/nColumns), 
        #                            actionIndex%nColumns)
        self.actionGrid.addLayout(actIcon, int(actionIndex/nColumns) + 1, 
                                  actionIndex%nColumns)
        #if item is not None:
        #    self.actionGrid.removeItem(item)

        self.actionIcons[actionIndex] = actIcon

    def new_experiment(self, name):
        nColumns = 6
        experimentIndex = self.n_experiments()
        for key in self.experimentIcons.keys():
             self.experimentIcons[key].selected = False
             self.experimentIcons[key].updateView()
        expIcon = ActionIcon(self._widget, experimentIndex, name, self.experiment_pressed, True)
        self.experimentGrid.addLayout(expIcon, int(experimentIndex/nColumns),
                                  experimentIndex%nColumns)
        self.experimentIcons[experimentIndex] = expIcon

    def step_pressed(self, step_index):
        gui_cmd = GuiCommand(GuiCommand.SELECT_ACTION_STEP, step_index)
        self.gui_cmd_publisher.publish(gui_cmd)

    def new_experiment_command(self):
        gui_cmd = GuiCommand(GuiCommand.NEW_EXPERIMENT, 0)
        self.gui_cmd_publisher.publish(gui_cmd)

    def compute_clusters_command(self):
        gui_cmd = GuiCommand(GuiCommand.COMPUTE_CLUSTERS, 0)
        self.gui_cmd_publisher.publish(gui_cmd)

    def compute_trajectory_command(self):
        gui_cmd = GuiCommand(GuiCommand.COMPUTE_TRAJECTORY, 0)
        self.gui_cmd_publisher.publish(gui_cmd)

    def action_pressed(self, actionIndex, isPublish=True):
        for i in range(len(self.actionIcons.keys())):
            key = self.actionIcons.keys()[i]
            if key == actionIndex:
                 self.actionIcons[key].selected = True
            else:
                 self.actionIcons[key].selected = False
            self.actionIcons[key].updateView()
        self.stepsBox.setTitle('Steps for Action ' + str(self.currentAction))
        if isPublish:
            gui_cmd = GuiCommand(GuiCommand.SWITCH_TO_ACTION, (actionIndex))
            self.gui_cmd_publisher.publish(gui_cmd)
        else:
            self.currentAction = actionIndex
        
    def experiment_pressed(self, expIndex, isPublish=True):
        for i in range(len(self.experimentIcons.keys())):
            key = self.experimentIcons.keys()[i]
            if key == expIndex:
                 self.experimentIcons[key].selected = True
            else:
                 self.experimentIcons[key].selected = False
            self.experimentIcons[key].updateView()
        
        if isPublish:
            gui_cmd = GuiCommand(GuiCommand.SWITCH_TO_EXPERIMENT, (expIndex))
            self.gui_cmd_publisher.publish(gui_cmd)
        else:
            self.current_experiment = expIndex

        self.actionBox.setTitle('Demonstrations for Experiment ' + str(self.current_experiment))

    def command_cb(self):
        clickedButtonName = self._widget.sender().text()
        for key in self.commands.keys():
            if (self.commands[key] == clickedButtonName):
                qWarning('Sending speech command: '+ key)
                command = Command()
                command.command = key
                self.speech_cmd_publisher.publish(command)
        
    def exp_state_cb(self, state):
        qWarning('Received new experiment state.')
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
